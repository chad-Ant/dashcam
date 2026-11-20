# Jetson cloud uplink core

C++17 implementation of the [Plan A device lifecycle](../../connector/CLOUD_ON_DEMAND_PLAN.md).
Use one `dashcam::cloud::OnDemandUplink` per enrolled vehicle in the native
host uplink service. The library uses the C++ standard library and has no
worker threads. Firestore authentication, WebSocket I/O and local connector
subscriptions belong to the host adapter.

## Build and verify

From the repository root:

```sh
make -C lib/libclouduplink check
make -C lib/libclouduplink sanitize
```

The regular build produces `lib/libclouduplink/build/libclouduplink.a`.
Include [libclouduplink.h](libclouduplink.h) and link the archive after the
consumer objects. The top-level aliases are `make clouduplink` and
`make clouduplink_test`.

## Connect the adapters

1. Own the controller on one uplink event loop. Post listener and socket
   callbacks onto that loop. Normalize a control document into
   `ControlSnapshot` and call `applyControl` with its actual document path
   and authenticated listener metadata. Convert the expiry to UTC Unix
   seconds and range-check signed versions before conversion. Accepted fields
   correspond to `desiredMode`, `configVersion`, `demandGeneration`,
   `channels` and `expiresAt` in the Python implementation.
2. After **every** event, read `target()` and reconcile resources against its
   latest value. A nonzero new attempt opens one authenticated relay socket.
   Register its requested channels and demand generation. On the response,
   pass the **originating** attempt and generation to `relayConfirmed`.
   Only `active.telemetry` and `active.video` authorize local readers.
   An attempt of zero releases all three resources.
3. On each reconciliation, arm or replace an event-loop timer for the absolute
   steady-clock time returned by `nextDeadline()`. Cancel the timer when the
   optional is empty. Reconcile again in the timer callback, including during
   a blocked network handshake. A renewal can move the deadline without
   changing the socket or readers. Use the latest `target()` before sending
   asynchronously prepared data.
4. Call `relayLost(attempt)` on disconnect. Back off and obtain fresh
   authoritative demand before retrying. A relay refusal keeps that generation
   idle. A newer generation can request another attempt. A policy change to
   `ReadyWhilePowered` releases the Plan A target; the shared supervisor owns
   the future Plan B handoff.
5. On shutdown, revocation or document deletion, call `close()`, reconcile
   the idle target, and cancel/drain callbacks before destroying the
   controller. Route completions to the controller that opened the socket.
   Attempt numbers are scoped to that controller's lifetime.

`UpdateResult::Accepted` means the input was processed; expired or empty
demand can produce an idle target. `Ignored` covers cached, foreign,
outdated or closed-session input. `Invalid` indicates a malformed or
inconsistent update. For every result, resources still follow the current
target because a lease may have expired while the event was in flight.
