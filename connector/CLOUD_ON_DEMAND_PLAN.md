# Plan A — On demand

**Owner setting:** `on_demand`. A powered Jetson watches one Firestore document directly. A viewer request wakes its uplink. Only active viewing holds a Cloud Run device socket open. This plan optimizes for infrequent viewing and gives the owner a measurable tradeoff between cold-start latency and compute usage.

Use the enrollment, prerequisites, client authentication, data formats and owner toggle defined in the [shared plan](CLOUD_MONITORING_PLAN.md). This is one selectable behavior of the final product. The viewer uses the same Connect and Camera actions in either mode.

## Sequence from boot to release

1. On network availability, the uplink authenticates as the enrolled Jetson. Read the current policy, then attach a real-time listener directly to that device's Firestore control document. The first listener snapshot closes the read/subscribe race. Wait for a server-confirmed, current value before opening an uplink. Refresh device credentials and reattach on reconnect.
2. At rest, only listener traffic and optional minute-level presence updates leave the Jetson. Publish presence directly to a separate restricted record if enabled. Local capture, recording and LAN monitoring run normally. Do not keep a background viewer socket open on the phone or dashboard landing page. [Firestore real-time listeners](https://firebase.google.com/docs/firestore/query-data/listen)
3. The viewer taps Connect. Cloud Run verifies access and registers a bounded viewer lease for that vehicle. Write the aggregate demand to Firestore, advancing a generation if the wanted channels change. If the Jetson is offline, expire the pending viewer request after a short UI deadline; a vehicle powering up hours later must not honor it.
4. Firestore delivers demand to the Jetson. Validate the authorized device path, generation, policy and expiry. Open an outbound authenticated WebSocket to the relay and register with device ID, session generation and capabilities. The relay confirms the current demand before the uplink opens its local telemetry subscription. Send the first fresh snapshot as soon as available. The clients distinguish online-without-recorder-data from waiting-for-device.
5. The first camera viewer adds video demand over the active connection. Open one local video subscription, forward fresh frames and share them among authorized viewers. One viewer leaving cannot cancel another viewer's subscription. Keep only the latest frame during congestion. Timestamp locally received frames before decoding or queuing.
6. Explicit stop or page/app background releases each viewer's channels. Once all demand ends, close the local subscriptions and the device relay socket. Keep the direct Firestore listener while the Jetson is powered. Close the pending viewer request if the browser abandons a cold start. The next Connect requires a new valid lease.

## Implementation milestones

| Deliverable | Implementation and verification |
|---|---|
| Jetson listener spike | Prove a device-scoped, refreshable identity can receive an initial snapshot and subsequent changes on ARM64. Test listener reconnect, token expiry, loss of network and denial for another vehicle. Use time-limited requests; stream startup cannot depend on a periodic poll. |
| Viewer demand service | Store desired channels and lease expiry transactionally. Renew active leases over the established connection. Refresh the durable wake-up record as needed to make reconnect safe. Use absolute server expiry plus device-local monotonic deadlines; Firestore document deletion/TTL timing is not a stream-control timer. |
| Wake and media integration | Log a correlation ID with click, authorization, committed demand, device receipt, uplink registration, first fresh snapshot and first decoded JPEG. Use elapsed-time measurements on the same clock for latency; cross-device timestamps require clock synchronization or separate spans. Avoid logging locations or images. |
| Client connection states | Show **Connecting to vehicle** on a pending request. Allow cancellation. Only show **Live** after valid data. A temporarily unavailable or powered-off vehicle must leave the user able to select the other Jetson. |
| Resource controls | Configure Cloud Run with minimum zero. Measure cold starts, listener reads after reconnect, presence writes and billable relay time. A long-open viewer socket on an offline device can itself keep Cloud Run active, so bound pending requests and close abandoned sockets. [Cloud Run WebSocket billing](https://docs.cloud.google.com/run/docs/triggering/websockets) · [Firestore pricing](https://firebase.google.com/docs/firestore/pricing) |

## Recovery cases

| Event | Required behavior |
|---|---|
| Cached demand arrives after LTE recovery | Re-read or await authoritative current demand. Discard old generations and expired leases, even if the local listener emits a cached snapshot first. Resume only when a viewer remains authorized. |
| Relay dies during live video | Stop upstream media when the socket closes or its local lease expires. Await or reconcile renewed demand before reopening. The camera-demand file will cease renewal when the local viewer leaves. |
| Viewer cancels as the Jetson connects | Relay rechecks aggregate demand at registration and rejects an obsolete generation or returns zero channels. The uplink immediately returns to idle listening. |
| Firestore is temporarily unavailable | Show degraded remote readiness. Retry with bounded backoff. An existing authorized stream can continue for the duration of a valid lease; inability to confirm authorization or renew demand ends it. Continue local recording. |
| Owner selects Ready while powered | Treat it as a configuration change even when demand is zero. Register the replacement control socket before declaring the change applied. Reuse any healthy active socket and its subscriptions. See the [toggle contract](CLOUD_MONITORING_PLAN.md#toggle-contract). |

## Completion criteria

- A user can wake either simulated device and then the real device through both clients. Report the p50 and p95 tap-to-telemetry measurements and separate cold-start results; include the first request after the relay has scaled down.
- Ten idle minutes show no telemetry/video uploads from the car and no continuously open device socket to Cloud Run. Report Firestore traffic separately.
- Five viewers can share one car uplink. Switching all viewers away releases the camera and the idle device relay socket. The other Jetson's policy and demand remain independent.
- Test stopping a session from the UI and losing a phone without an orderly close. In the latter case, measure the viewer timeout plus the remaining lease lifetime as the bound for unnecessary upload.
- Compare measured latency against the shared three-second target. If cold starts miss it, expose the already planned Ready while powered choice, and use the measurements in the owner's guidance.
