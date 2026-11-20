import asyncio
from dataclasses import replace
from datetime import datetime, timezone
import unittest
from unittest.mock import patch

from cloud_on_demand import ControlSnapshot, OnDemandUplink, RelayTarget


TELEMETRY = frozenset({"telemetry"})
BOTH = frozenset({"telemetry", "video"})
IDLE = RelayTarget()


class DocumentTests(unittest.TestCase):
    def test_firestore_fields(self):
        fields = dict(desiredMode="on_demand", configVersion=1, demandGeneration=2,
                      channels=["telemetry"], expiresAt=datetime.fromtimestamp(90, timezone.utc))
        path = "vehicles/A/control/current"
        self.assertEqual(ControlSnapshot.from_document(path, fields),
                         ControlSnapshot("A", "on_demand", 1, 2, TELEMETRY, 90))
        for bad in (dict(channels="telemetry"), dict(channels=["gps"]), dict(configVersion=True),
                    dict(expiresAt=float("nan")), dict(expiresAt=datetime(2026, 1, 1)),
                    dict(expiresAt=None), dict(desiredMode="unknown"), dict(demandGeneration=-1)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                ControlSnapshot.from_document(path, {**fields, **bad})
        with self.assertRaises(ValueError):
            ControlSnapshot.from_document("vehicles/A/status/current", fields)


class UplinkTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.events = []
        self.wall = 1000.0
        self.uplink = OnDemandUplink("A", self.events.append, wall_time=lambda: self.wall)
        self.addCleanup(self.uplink.close)
        self.wake = ControlSnapshot("A", "on_demand", 2, 5, TELEMETRY, self.wall + 15)

    def apply(self, **changes):
        self.uplink.apply_control(replace(self.wake, **changes), server_confirmed=True)

    async def test_idle_wake_confirm_change_channels_and_release(self):
        self.apply(demand_generation=4, channels=frozenset(), expires_at=None)
        self.assertEqual(self.events, [])
        self.apply()
        first = self.uplink.target.attempt
        self.assertEqual(self.events, [RelayTarget(first, 5, TELEMETRY)])
        self.uplink.relay_confirmed(first, 5, TELEMETRY)
        self.apply()
        self.apply(expires_at=self.wall + 25)
        self.assertEqual(len(self.events), 2)
        self.assertEqual(self.uplink.target.active_channels, TELEMETRY)
        self.apply(demand_generation=6, channels=BOTH)
        self.assertEqual(self.uplink.target, RelayTarget(first, 6, BOTH))
        self.uplink.relay_confirmed(first, 5, TELEMETRY)
        self.assertFalse(self.uplink.target.active_channels)
        self.uplink.relay_confirmed(first, 6, BOTH)
        self.apply(demand_generation=7, channels=frozenset(), expires_at=None)
        self.uplink.relay_confirmed(first, 6, BOTH)
        self.assertEqual(self.uplink.target, IDLE)

    async def test_reject_cached_foreign_stale_and_denied_wakes(self):
        self.uplink.apply_control(self.wake, server_confirmed=False)
        self.apply(device_id="B")
        self.apply(demand_generation=4, expires_at=self.wall - 1)
        self.assertEqual(self.events, [])
        self.apply()
        first = self.uplink.target.attempt
        with self.assertRaises(ValueError):
            self.uplink.relay_confirmed(first, 5, BOTH)
        self.uplink.relay_confirmed(first, 5, frozenset())
        self.apply()
        self.apply(demand_generation=4)
        self.apply(config_version=1, demand_generation=6)
        self.assertEqual(self.uplink.target, IDLE)
        self.apply(demand_generation=6)
        self.assertNotEqual(self.uplink.target.attempt, first)
        self.assertIsNotNone(self.uplink.target.attempt)

    async def test_reconnect_and_delayed_ack_after_close(self):
        self.apply()
        first = self.uplink.target.attempt
        self.uplink.relay_lost(first)
        self.assertEqual(self.uplink.target, IDLE)
        self.apply()
        second = self.uplink.target.attempt
        self.uplink.relay_lost(first)
        self.uplink.relay_confirmed(first, 5, TELEMETRY)
        self.assertEqual(self.uplink.target, RelayTarget(second, 5, TELEMETRY))
        self.uplink.close()
        self.apply(demand_generation=6)
        self.uplink.relay_confirmed(second, 5, TELEMETRY)
        self.assertEqual(self.uplink.target, IDLE)

    async def test_mode_change_and_out_of_order_updates(self):
        self.apply()
        self.uplink.relay_confirmed(self.uplink.target.attempt, 5, TELEMETRY)
        with self.assertRaises(ValueError):
            self.apply(channels=BOTH)
        with self.assertRaises(ValueError):
            self.apply(desired_mode="ready_while_powered")
        self.apply(config_version=3, desired_mode="ready_while_powered")
        self.apply()
        self.assertEqual(self.uplink.target, IDLE)
        self.apply(config_version=4)
        self.assertEqual(self.uplink.target.requested_channels, TELEMETRY)

    async def test_silent_expiry_without_events(self):
        expired = asyncio.Event()

        def record(target):
            if target == IDLE:
                expired.set()

        uplink = OnDemandUplink("A", record, wall_time=lambda: self.wall)
        self.addCleanup(uplink.close)
        uplink.apply_control(replace(self.wake, expires_at=self.wall + .02), server_confirmed=True)
        uplink.relay_confirmed(uplink.target.attempt, 5, TELEMETRY)
        await asyncio.wait_for(expired.wait(), timeout=2)
        self.assertEqual(uplink.target, IDLE)

    async def test_duplicate_cannot_extend_monotonic_deadline_after_clock_rollback(self):
        loop = asyncio.get_running_loop()
        start = loop.time()
        with patch.object(loop, "time", return_value=start) as now:
            self.apply(expires_at=self.wall + 5000)
            ack = self.uplink.target.attempt
            self.wall -= 3600
            now.return_value += 31
            self.uplink.relay_confirmed(ack, 5, TELEMETRY)
            self.apply(expires_at=6000)
            self.assertEqual(self.uplink.target, IDLE)

    async def test_renewal_keeps_grant_and_uses_remaining_lease(self):
        loop = asyncio.get_running_loop()
        with patch.object(loop, "time", return_value=loop.time()) as now:
            self.apply()
            attempt = self.uplink.target.attempt
            self.uplink.relay_confirmed(attempt, 5, TELEMETRY)
            now.return_value += 10
            self.wall += 10
            self.apply(expires_at=self.wall + 17)
            self.apply()  # delayed original snapshot
            now.return_value += 16
            self.assertEqual(self.uplink.target.active_channels, TELEMETRY)
            now.return_value += 2
            self.assertEqual(self.uplink.target, IDLE)
