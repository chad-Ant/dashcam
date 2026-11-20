# Plan B — Ready while powered

**Owner setting:** `ready_while_powered`. A Jetson starts an outbound authenticated control socket at boot or on network recovery. The request wakes Cloud Run. The relay can route the first viewer's request over that socket immediately. The car sends telemetry and camera frames only while those channels have viewer demand.

Use the prerequisites, per-vehicle access rules, shared relay, viewer APIs and owner toggle in the [shared plan](CLOUD_MONITORING_PLAN.md). No direct internet route into the car or permission to start/stop cloud compute is needed on the Jetson.

## Sequence from boot to release

1. Start the cloud uplink with the local monitoring service. Network readiness can lag boot: retry DNS/TLS and authentication failures appropriately, with jittered backoff for transient outages. If enrollment is revoked, stop sending and report that re-enrollment is needed.
2. Read the current desired policy and open the relay socket. Authenticate the device, negotiate capabilities and receive a new relay session generation. Record the vehicle as **Ready** only after registration succeeds. Last-seen timestamps from earlier boots can inform the UI while registration is pending.
3. When no one is watching, send a small heartbeat every 60 seconds and expect acknowledgement. Summarize receipt at minute-level granularity if durable presence is required. Inbound valid device traffic also refreshes the relay's liveness deadline. No local telemetry or video reader is needed while idle. Choose the WebSocket and intermediary timeout settings to accommodate a quiet minute.
4. A viewer taps Connect. The relay verifies access and pushes channel demand down the registered socket at once. Valid demand starts the existing local telemetry feed. Camera demand starts the existing preview feed. Renew short media leases during viewing. Count subscribers separately by vehicle and channel. Keep control messages deliverable even when video bandwidth drops.
5. When the last viewer leaves, close local data subscriptions and retain the registered control socket. The next viewer avoids relay startup and device authentication latency as long as the session is healthy.
6. On orderly shutdown, close the socket and mark only its current generation offline. On silent power or LTE loss, use the 150-second liveness deadline from the shared plan; a lost socket may be detected earlier. Mark displayed samples stale independently within the existing client freshness window. Close or release viewer sockets for the absent device. When another Jetson remains online, retain its service. After the last device and viewer request have gone, Cloud Run becomes eligible to scale to zero.

## Implementation milestones

| Deliverable | Implementation and verification |
|---|---|
| Device registry and control socket | Register each authenticated socket by vehicle and relay-issued generation. A replacement connection supersedes the old one. A late close or heartbeat from the old generation cannot erase the new registration. At boot and reconnect, reconcile the desired policy before forwarding data. |
| Timers and reconnect | Separate the 60-second heartbeat, 150-second presence timeout, short media-demand lease and two-second data freshness timer. Explicitly configure Cloud Run request timeout up to 60 minutes. Handle planned rollover, service deployment, modem handoff and unexpected disconnect. Resubscribe current authorized viewers after re-registration. [Cloud Run connection timeouts](https://docs.cloud.google.com/run/docs/triggering/websockets) |
| Dashboard integration | Show **Ready** with last contact before viewing and **Live** only on fresh telemetry. Enable immediate camera start on viewer request. Clear frames on device change, connection loss or stale data. Reconnect on return to the foreground; an inactive client cannot hold a camera subscription. |
| Cost instrumentation | Count unique wall-clock intervals that at least one device socket is active and compare with actual billable instance time. Also measure outbound GiB and viewer minutes. Assuming one serving instance, overlapping Jetson sessions share relay time. A heartbeat without media still keeps that instance active. [Cloud Run billing](https://cloud.google.com/run/pricing) · [WebSocket billing](https://docs.cloud.google.com/run/docs/triggering/websockets) |

## Recovery cases

| Event | Required behavior |
|---|---|
| A powers off while B is serving viewers | Disconnect A's subscribers, preserve B's registration and B's streams. Expire A's outstanding demand so it cannot start an old session on its next boot. |
| Relay restarts while both Jetsons are online | Both devices reconnect with backoff. Rebuild the live registry from authenticated registrations. Reject stale frames and duplicate registrations. Viewer demand can resume only after authorization and fresh data. |
| A browser remains open on an offline vehicle | Deliver offline status and close its streaming requests. Use explicit reconnect or bounded short status requests. An old dashboard must not hold a streaming socket indefinitely after the final device leaves. |
| A phone disappears mid-stream | Expire the viewer lease or socket, reduce the viewer count and release the camera if it was the last camera viewer. The device control connection remains ready. |
| Owner selects On demand during LTE loss | Persist the desired version in the cloud and show **Pending**. The next device registration reads that version and arms the Firestore listener. While connected, deliver the change on the ready socket, and verify listener readiness before dropping the last idle relay connection. See the [toggle contract](CLOUD_MONITORING_PLAN.md#toggle-contract). |

## Completion criteria

- Boot from cold with no viewers. Confirm automatic registration and a ready indication without user action. Verify idle network capture contains control traffic and no snapshots or images.
- With the device ready, request telemetry just after a heartbeat. Confirm the request arrives immediately and record tap-to-first-fresh-sample and first decoded frame on cellular.
- Exercise two Jetsons, five viewers, orderly shutdown, physical network loss, relay restart and a session longer than the Cloud Run WebSocket timeout. Verify no duplicate camera reader and recovery after each event.
- Confirm local recording continues when both cloud paths fail. Record actual time to stale display, offline detection, socket release and zero serving instances as separate measurements. Cloud Run controls the precise scale-down time.
- Run a representative week and compare powered-on relay hours against viewing-only hours. Use the measured difference and the selected region's pricing to explain this option's speed/cost tradeoff to the owner.
