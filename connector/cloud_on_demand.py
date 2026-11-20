"""Jetson-side Plan A control logic. All methods run on the owning asyncio loop.

A listener supplies authenticated ControlSnapshots. The transport adapter receives
immutable targets, opens one relay socket per attempt, and follows active_channels
for local subscriptions. Its callback must return promptly (schedule network I/O).
"""
import asyncio
import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime


CHANNELS = frozenset({"telemetry", "video"})
MAX_LEASE_SECONDS = 30


@dataclass(frozen=True)
class ControlSnapshot:
    """Normalized control document. expires_at is UTC Unix seconds."""

    device_id: str
    desired_mode: str
    config_version: int
    demand_generation: int
    channels: frozenset[str]
    expires_at: float | None

    def __post_init__(self):
        if not isinstance(self.device_id, str) or not self.device_id:
            raise ValueError("Device ID required")
        if self.desired_mode not in ("on_demand", "ready_while_powered"):
            raise ValueError("Invalid connection mode")
        if any(type(n) is not int or n < 0 for n in (self.config_version, self.demand_generation)):
            raise ValueError("Versions must be non-negative integers")
        if not isinstance(self.channels, frozenset) or not self.channels <= CHANNELS:
            raise ValueError("Expected a frozenset of telemetry/video channels")
        if self.expires_at is None:
            if self.channels:
                raise ValueError("Requested channels require a lease expiry")
        elif type(self.expires_at) not in (int, float) or not math.isfinite(self.expires_at):
            raise ValueError("Lease expiry must be finite")

    @classmethod
    def from_document(cls, path: str, fields: Mapping) -> "ControlSnapshot":
        """Decode vehicles/{id}/control/current; identity comes from the document path."""
        parts = path.split("/")
        if len(parts) != 4 or parts[0] != "vehicles" or parts[2:] != ["control", "current"]:
            raise ValueError("Invalid control document path")
        try:
            channels = fields["channels"]
            if not isinstance(channels, list) or any(c not in CHANNELS for c in channels):
                raise ValueError("Invalid channel list")
            expiry = fields.get("expiresAt")
            if isinstance(expiry, datetime):
                if expiry.utcoffset() is None:
                    raise ValueError("Lease timestamp requires a timezone")
                expiry = expiry.timestamp()
            return cls(parts[1], fields["desiredMode"], fields["configVersion"],
                       fields["demandGeneration"], frozenset(channels), expiry)
        except (KeyError, TypeError) as error:
            raise ValueError("Invalid control document fields") from error


@dataclass(frozen=True)
class RelayTarget:
    """attempt=None releases all resources; empty active_channels gates local media."""

    attempt: int | None = None
    demand_generation: int | None = None
    requested_channels: frozenset[str] = frozenset()
    active_channels: frozenset[str] = frozenset()


class OnDemandUplink:
    """Reconcile one device's listener and relay events, including silent expiry.

    Call close() before disposing the adapter. A relay_lost() requires a fresh
    authoritative read after backoff before a new connection attempt can begin.
    """

    def __init__(self, device_id: str, on_change: Callable[[RelayTarget], None], *,
                 wall_time: Callable[[], float] = time.time):
        if not device_id:
            raise ValueError("Device ID required")
        self._device_id = device_id
        self._on_change = on_change
        self._wall_time = wall_time
        self._loop = asyncio.get_running_loop()
        self._snapshot: ControlSnapshot | None = None
        self._deadline = 0.0
        self._timer: asyncio.TimerHandle | None = None
        self._next_attempt = 1
        self._denied_generation: int | None = None
        self._target = RelayTarget()
        self._closed = False

    @property
    def target(self) -> RelayTarget:
        self._expire_if_due()
        return self._target

    def apply_control(self, snapshot: ControlSnapshot, *, server_confirmed: bool) -> None:
        """Apply current authorized state, ignoring cache, foreign devices and replay."""
        self._expire_if_due()
        if self._closed or server_confirmed is not True or snapshot.device_id != self._device_id:
            return
        previous = self._snapshot
        if previous is not None:
            if (snapshot.config_version < previous.config_version or
                    snapshot.demand_generation < previous.demand_generation):
                return
            if snapshot.config_version == previous.config_version and snapshot.desired_mode != previous.desired_mode:
                raise ValueError("Mode change requires a new configuration version")
            if snapshot.demand_generation == previous.demand_generation:
                if snapshot.channels != previous.channels:
                    raise ValueError("Channel change requires a new demand generation")
                if (snapshot.config_version == previous.config_version and snapshot.channels
                        and snapshot.expires_at < previous.expires_at):
                    return

        self._snapshot = snapshot
        if not snapshot.channels:
            self._idle()
            return

        # Only a new lease establishes a new monotonic deadline. Re-delivering
        # the same document after a wall-clock adjustment cannot revive it.
        new_lease = (previous is None or snapshot.demand_generation != previous.demand_generation
                     or snapshot.expires_at != previous.expires_at)
        remaining = min(snapshot.expires_at - self._wall_time(), MAX_LEASE_SECONDS)
        candidate = self._loop.time() + remaining
        deadline = candidate if new_lease else min(self._deadline, candidate)
        deadline_changed = deadline != self._deadline
        self._deadline = deadline
        if (snapshot.desired_mode != "on_demand" or self._deadline <= self._loop.time()
                or snapshot.demand_generation == self._denied_generation):
            self._idle()
            return
        if self._timer is None or deadline_changed:
            self._cancel_timer()
            self._timer = self._loop.call_at(self._deadline, self._expire)

        attempt = self._target.attempt
        if attempt is None:
            attempt = self._next_attempt
            self._next_attempt += 1
        if self._target.demand_generation != snapshot.demand_generation:
            # Even on an existing socket the new generation needs relay confirmation.
            self._publish(RelayTarget(attempt, snapshot.demand_generation, snapshot.channels))

    def relay_confirmed(self, attempt: int, generation: int, channels: frozenset[str]) -> None:
        """Apply the relay's grant for this attempt. An empty grant releases the socket."""
        current = self.target
        if current.attempt is None or attempt != current.attempt or generation != current.demand_generation:
            return
        if not isinstance(channels, frozenset) or not channels <= current.requested_channels:
            raise ValueError("Relay grant exceeds device demand")
        if not channels:
            self._denied_generation = generation
            self._idle()
            return
        self._publish(RelayTarget(attempt, generation, current.requested_channels, channels))

    def relay_lost(self, attempt: int) -> None:
        """Release readers; late closes from superseded sockets are harmless."""
        if self._target.attempt == attempt:
            self._idle()

    def close(self) -> None:
        self._closed = True
        self._idle()

    def _expire(self) -> None:
        self._timer = None
        # asyncio may deliver timers one clock tick early.
        if self._loop.time() < self._deadline:
            self._timer = self._loop.call_at(self._deadline, self._expire)
        else:
            self._idle()

    def _expire_if_due(self) -> None:
        if self._target.attempt is not None and self._loop.time() >= self._deadline:
            self._idle()

    def _cancel_timer(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _idle(self) -> None:
        self._cancel_timer()
        self._publish(RelayTarget())

    def _publish(self, target: RelayTarget) -> None:
        if self._target != target:
            self._target = target
            self._on_change(target)
