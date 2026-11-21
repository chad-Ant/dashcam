# Dashcam Connect Android and web client review

Reviewed on 21 November 2026 against commit `f0d00d2133d3b484c8c20ea1d21aad4405869f42` on the `camera` branch. The review covers on-demand live video, its ten-minute limit, reconnection, and the cost of rendering telemetry, camera frames, and maps. Source line references refer to this revision. All proposed fixes and acceptance checks below are outstanding.

## Summary

The review identified four correctness issues and five performance opportunities. **Prioritize the two browser session issues together** because both come from allowing a video connection to use cached pairing information. On Android, **resolve ownership of displayed bitmaps before tuning the pool**. The strongest measured optimization is smaller in scope. Updating just the web camera card when a JPEG finished loading reduced binding evaluations by **51.7%** in the five-second comparison and displayed the same 48 frames.

| Finding | User-visible consequence or opportunity | Evidence |
|---|---|---|
| C1 · Android bitmap ownership | A delayed draw can use pixels that the decoder has already overwritten | Source inspection and Android rendering documentation |
| C2 · Web identity check on resume | A video request can start under the previous device’s live-view selection | Reproduced in headless Firefox with a delayed identity response |
| C3 · Web session rejection | An expired or signed-out browser can keep retrying video and showing “Stop live view” | Reproduced with a local server returning HTTP 401 |
| C4 · Android telemetry reconnect | An ordinary TLS transport error can leave telemetry offline with a certificate warning | Source inspection of the telemetry and video retry paths |
| P1 · Web rendering | A decoded JPEG evaluates every dashboard binding | Measured in Firefox with the current client and a temporary comparison |
| P2 · Offscreen video | Scrolling past the camera continues to use bandwidth and decode frames | Measured on web, confirmed in the Android selection logic |
| P3 · Android idle updates | Two streams of clock updates continue while monitoring is idle | Source inspection of repository lifetime and timer cadence |
| P4 · Android telemetry formatting | Clock and unrelated data changes can repeat formatting and sorting | Source inspection of Compose derived state and card implementations |
| P5 · Android map requests | A pinch can start tile requests for intermediate zoom levels | Source inspection of Android gesture and tile loading paths |

C1–C4 were rated **P2, medium priority**, in the code review. P1–P5 above are identifiers for performance opportunities, ordered by suggested implementation priority.

## Correctness findings

### C1. Android can reuse a bitmap while the renderer still holds it

**What happens.** A background coroutine decodes a JPEG, wraps its mutable bitmap with `asImageBitmap()`, and publishes the frame to a `StateFlow`. Once publication succeeds, `pool.advance()` timestamps the preceding slot. The three-slot pool allows the decoder to supply that bitmap to `BitmapFactory` as `inBitmap` once 100 ms have elapsed since it was superseded. Compose collects the flow and draws on a separate schedule. The pool receives no signal that Compose has displayed the replacement or that an outstanding draw has released the preceding bitmap. [Frame publication](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L368) · [Bitmap reuse decision](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L427) · [Camera picture](../android/app/src/main/java/com/dashcam/connector/MainActivity.kt#L297)

**Failure conditions and impact.** Suppose the screen is showing frame A when the UI thread stalls. The decoder can continue publishing B and C, then rotate back to A’s slot after the reuse delay. The screen or a pending hardware draw can still reference A when the decoder writes new pixels into that allocation. Possible symptoms include a partially updated picture or pixels from one frame shown with another frame’s caption. Android documents that hardware-accelerated drawing can retain bitmap references across draw passes. The race is visible in the ownership logic. A visual artifact was not captured on the A54 during this review. [Android bitmap lifetime guidance](https://developer.android.com/reference/android/graphics/Bitmap#reconfigure(int,int,android.graphics.Bitmap.Config))

**Suggested fix.** The simplest safe baseline is to publish a fresh bitmap for each frame and let the runtime release it when all consumers finish. If profiling justifies reuse, give the producer an explicit release signal that covers the rendering lifetime, including in-flight draws. Increasing the fixed delay or the pool size changes the timing of the race but cannot establish ownership.

**Acceptance check.** Use a test feed that prints a large sequence number inside each JPEG and exposes the same number as frame metadata. Delay UI consumption while decoding continues for several pool rotations. Track the identities of published and reused buffers, then verify that each retained frame keeps its original pixels until release. On the A54, also measure frame time and allocation rate with continuous playback, stop/start, and navigation away from Live.

### C2. The web client can open video before confirming the device after resume

**What happens.** Hiding the tab closes its sockets but retains the selected device and the live-view window. When the tab becomes visible, the handler calls the asynchronous `start()` and immediately calls `syncVideo()`. While `/v1/identity` is pending, the cached identity still satisfies `videoWanted()`. The client can therefore open `/v1/video` before it learns which device is serving the origin. The later response can clear live view, after the request has already been made. The periodic tick also calls `syncVideo()`, so changing only the order of the two calls in the visibility handler would leave another route through the same gate. [Visibility handler](web/app.js#L875) · [Identity refresh](web/app.js#L622) · [Video gate](web/app.js#L708) · [Periodic check](web/app.js#L687)

**Observed sequence.** In headless Firefox, the fixture identified itself as device A. With live view on, hiding the tab closed video. The fixture then changed its identity to B and held the next identity response open. Returning to the tab created **one new video socket before the response was released**. Once B’s identity arrived, the client closed the socket and restored “Start live view.” The fixture established request ordering. Whether a different real device would deliver frames also depends on its certificate, session, and response timing.

**Suggested fix.** Mark identity verification as pending for each new connection attempt. Gate every path that opens a video socket on successful verification for the current attempt. Allow automatic resume only if the confirmed device matches the device selected at Start, it still advertises video, and the original deadline is still in the future. Discard late responses from obsolete attempts before they update shared identity state. [Device and deadline checks](web/app.js#L728) · [Identity assignment](web/app.js#L605)

**Acceptance check.** Hold the identity response open across multiple 250 ms ticks after a hide/show cycle. Expect zero video requests while verification is pending. Release the response with A, B, an unpaired response, and a device without video in separate runs. Only A with a valid session and remaining time should resume automatically. Repeat with the deadline expiring during verification.

### C3. The web client keeps retrying rejected video while telemetry is healthy

**What happens.** The video socket’s close handler clears the frame and schedules backoff. A failed browser WebSocket upgrade does not give this handler a reliable HTTP status. The handler keeps using the cached identity. Meanwhile, an existing telemetry connection can continue to deliver snapshots. A practical example is signing out in a second tab. The server clears the shared cookie, while an already-open socket checks the expiry captured when it connected. New video handshakes can fail before the first tab’s telemetry path detects a session change. [Video close handler](web/app.js#L789) · [Retry scheduling](web/app.js#L741) · [Server sign-out](service.py#L489) · [Socket expiry check](service.py#L452)

**Observed sequence.** A local server accepted the initial identity request, kept sending telemetry, and returned HTTP 401 to each video upgrade. Within about 2.3 seconds Firefox had attempted video **twice** and made only **one identity request**, the initial one. The button read “Stop live view” and the caption reported a retry with ten minutes remaining. The server rejected the requests. The user-facing failure is continued retries and a misleading on-state until another path clears the session or the timer ends.

**Suggested fix.** After an unsuccessful video upgrade, revalidate through the ordinary HTTP identity endpoint, where the response status is available. A confirmed 401 or 403 should clear the session’s live-view selection and cancel its retry. A verified device without video should clear demand and show the capability message. An identity that remains valid should retain bounded backoff for transient failures, including a full viewer pool. Share an in-flight identity check with telemetry so two failing sockets cannot start parallel checks. [HTTP identity outcomes](web/app.js#L605)

**Acceptance check.** Keep telemetry flowing and make the next video upgrade fail. Return 401 from the identity probe and verify that the toggle resets and that no further video handshakes occur. Repeat with identity returning 200 and a temporary viewer-capacity failure, then restore video. In that case the client should recover within the original live-view window. Also exercise sign-out in a second tab with the first tab still open.

### C4. Android treats a transient telemetry TLS failure as a permanent certificate error

**What happens.** The telemetry `SSLException` handler always publishes “Certificate mismatch or expired certificate. Check pairing.” and exits the retry coroutine. It checks `isCertificateFailure()` only when deciding whether to disable live view. The video path already uses that classifier to distinguish certificate errors from ordinary TLS connection loss and retries the latter. [Telemetry exception handler](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L157) · [Certificate classifier](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L120) · [Video exception handler](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L289)

**Failure conditions and impact.** A transport reset surfaced as an `SSLException` without a certificate cause can stop telemetry retries while the same Wi-Fi network remains available. The banner directs the user to check pairing. Telemetry can remain offline until a lifecycle event, network callback, or manual reconnect restarts the repository. The review established the control flow. It did not reproduce this exception on the phone or attribute the historical Wi-Fi reconnect delay to this defect.

**Suggested fix.** Take the terminal telemetry branch only for a verified certificate failure on the current connection generation. For other TLS errors, clear freshness, show the ordinary connection-loss status, and continue through the existing backoff. Keep certificate pinning, certificate validity checks, and authentication rejection behavior intact.

**Acceptance check.** Feed the telemetry connection one transient TLS failure followed by success while keeping the selected device and Wi-Fi network unchanged. Expect automatic retry and fresh snapshots. Separately exercise a pin mismatch and an expired certificate. Both should produce the pairing warning and stop retries. Switch devices during a pending failure and verify that its late result cannot change the new device’s status.

## Performance opportunities

### P1. Web camera frames trigger full-dashboard work

Each completed JPEG calls `render()`. That function creates a view snapshot and evaluates every registered binding, including hidden pages. The map binding calls `draw()`, which reads the map element’s dimensions before returning for a hidden map. The temperature binding can sort sensor names and compare their serialized signature even while the System page is hidden. Existing text comparisons reduce DOM writes, but the binding calculations still run. [Frame completion](web/app.js#L802) · [Global render loop](web/app.js#L89) · [Map draw](web/app.js#L274) · [Temperature binding](web/app.js#L188)

**Measured comparison.** Headless Firefox loaded the real client against a local fixture serving telemetry at a nominal 5 Hz and the repository’s JPEG test card at a nominal 10 Hz. Live was visible and the map was on the hidden Vehicle page. Counters recorded full render calls, binding evaluations, map draw function calls, and successful image loads. The temporary comparison changed just the frame-completion path to call the camera binding. Both runs used the same feed and approximately five-second observation window.

| Five-second observation | Current client | Camera-only update on JPEG completion |
|---|---:|---:|
| Successful JPEG loads | 48 | 48 |
| Full-dashboard render calls | 93 | 44 |
| Binding evaluations | 4,743 | 2,292 |
| Hidden-map draw function calls | 93 | 44 |

The current client evaluated 51 bindings on every full render. The comparison eliminated 2,451 evaluations, a **51.7% reduction**, with the same frame count. The remaining full renders include telemetry and timer updates. A “map draw” here means entry into the JavaScript function. These counters establish how much application work can be removed. CPU time, browser paint time, energy use, and real-device FPS require separate measurements.

**Suggested fix and acceptance.** Give frame completion an explicit camera update function. Then group telemetry bindings by visible page and refresh the newly selected page immediately on navigation. Continue updating shared state such as the last-known GPS fix even when its map is hidden. Preserve stale-picture removal on silence, stale telemetry on disconnect, countdown expiry, and immediate clearing on Stop. Repeat the fixture with the same counters, then record a browser performance trace to quantify scripting and layout time on the laptop or phone.

### P2. Both clients stream while the camera is scrolled out of view

The browser checks the selected page and document visibility. Android checks the selected page and foreground state. Scrolling within Live leaves those conditions true even after the camera has completely left the viewport. The Android repository owns the socket and decoder independently of whether its lazy-list camera item is currently visible. [Browser video gate](web/app.js#L708) · [Android page gate](../android/app/src/main/java/com/dashcam/connector/MainActivity.kt#L88) · [Android decoder launch](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L275)

**Measured behavior and savings estimate.** In Firefox the camera box’s bottom edge was above the viewport, yet **14 more JPEGs decoded in 1.5 seconds**. The camera-only render comparison produced the same offscreen frame count. Avoiding global rendering therefore leaves the network and decode cost in place. In the earlier physical soak, video averaged 78 KiB/s on the A54 and 80 KiB/s on the laptop. At those rates, 600 fully offscreen seconds correspond to **45.7–46.9 MiB per viewer**, calculated as mean KiB/s × 600 ÷ 1,024. Actual savings depend on scene size and how much of the ten-minute session remains when the user scrolls away. [Physical soak measurements](VALIDATION.md#L153)

**Suggested fix and acceptance.** Add viewport visibility to the existing demand decision. The browser can observe the camera box with an `IntersectionObserver`. Android can use camera-item visibility from the lazy list. A short grace period can absorb brief scrolls without repeatedly opening TLS sockets. Keep the original Start deadline running while offscreen. Verify that sustained scrolling closes only the video socket, telemetry stays live, and scrolling back resumes video only within the same valid session. Exercise partial visibility, rapid scroll reversals, and expiry while offscreen.

### P3. Android publishes clock updates while monitoring is idle

The repository starts a loop that runs for the lifetime of its coroutine scope. Every 500 ms it copies both `MonitorState` and `VideoState` with a new `nowMs`. That is nominally two timer callbacks and four state publications per second while the process is scheduled, including when the streams have no fresh data or the activity is in the background. Lifecycle-aware UI collection reduces UI work, but the producer still updates both flows. The view model also collects telemetry state for its map. A separate one-second ticker operates during a live-view window. [Repository clock](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L60) · [Live-view ticker](../android/app/src/main/java/com/dashcam/connector/MonitorRepository.kt#L102) · [Map state collector](../android/app/src/main/java/com/dashcam/connector/DashboardModel.kt#L31)

**Suggested fix.** Update each clock only while it can affect a visible age, freshness transition, or active deadline. A camera with no frame and live view off needs no periodic video-state copy. An inactive repository can evaluate elapsed time on resume before reopening a stream. Keep a scheduled freshness transition for the case where a previously healthy connection stops sending data. The reported rate is a count from the code. Power and CPU savings on the A54 remain unmeasured.

**Acceptance check.** Count producer emissions during idle, telemetry-only, live video, and background operation. Verify that a silent camera still goes stale, silent telemetry still expires, age labels remain accurate, and backgrounding for more than ten minutes prevents automatic video resume. Compare CPU time or wakeups over equal-length foreground and background intervals.

### P4. Android still formats unchanged telemetry while deciding which cards to recompose

The per-card Compose changes narrow the scope of recomposition. Each card derives a presentation value from the shared `MonitorState`. A clock-only change or a new snapshot with identical sensor values can still rerun the calculation. For example, memory rows read the JSON, convert bytes to GiB, call `String.format`, and allocate a list of row values. The temperature card enumerates and sorts names, formats values, and constructs its row list. Structural equality can suppress a subsequent UI update after that work has already occurred. This cost applies to cards whose derived state is being observed. [Derived state helper](../android/app/src/main/java/com/dashcam/connector/MainActivity.kt#L156) · [Formatting](../android/app/src/main/java/com/dashcam/connector/MainActivity.kt#L166) · [Memory and temperature rows](../android/app/src/main/java/com/dashcam/connector/MainActivity.kt#L265)

**Suggested fix.** Start by separating age-only updates from data and freshness changes. The connection banner needs elapsed time. Temperature and memory cards need their own values and the fresh/stale transition. If profiling still shows formatting pressure, compare each card’s typed inputs before constructing display strings and retain the sensor ordering while the sensor set stays the same. Scope caches to the active device so changing devices or losing freshness clears the presentation immediately.

**Acceptance check.** With fixed sensor values and an advancing clock, count formatter calls and temperature sorts as well as recompositions. Then change one value, add or remove a sensor, lose freshness, and reconnect. Verify that the correct rows update, stale readings become “—”, and units, rounding, and sensor order stay consistent. Capture allocations and main-thread time on the A54 before expanding the change across all cards.

### P5. Android can defer tile requests during fast pinch zooms

The map recomputes visible placements as its center or zoom changes. When the resulting tile keys change, `LaunchedEffect(keys)` requests the new set. The loader cancels requests that left the wanted set and starts work for newly wanted tiles. A pinch crossing several integer tile zoom levels can therefore queue and cancel downloads for intermediate levels. HTTP concurrency is already capped at four. The remaining opportunity is avoiding work for tiles that are visible only briefly. The web client already waits 200 ms after a tile zoom change before starting new downloads. [Android viewport and requests](../android/app/src/main/java/com/dashcam/connector/MapCard.kt#L124) · [Android tile cancellation](../android/app/src/main/java/com/dashcam/connector/TileLoader.kt#L95) · [Web zoom settling](web/app.js#L326)

**Suggested fix.** Keep pan, zoom, marker movement, and cached-tile drawing responsive during a gesture. Defer new network requests for intermediate zoom levels and request the final visible set promptly when the gesture settles. Reuse the existing cached parent-tile fallback. The web client’s 200 ms interval is a reasonable comparison point for a device test.

**Acceptance check.** Run the same multi-level pinch with a cold cache, a warm cache, and a slow connection. Compare requests started, requests cancelled, bytes received, and time to complete the final viewport. Check single-step zoom, normal following while the car moves, and leaving the Vehicle page during the delay. The source shows the cancellation path. No tile-traffic reduction was measured during this review.

## Validation record and limits

| Check | Result |
|---|---|
| Android debug APK | `:app:assembleDebug` completed successfully |
| Android JVM tests | 24 passed on rerun, comprising 7 live-view, 7 protocol, and 10 tile-math tests |
| Android lint | 0 errors and 15 warnings |
| Browser baseline | Headless Firefox passed off-on-load, manual Start, manual Stop, hide-to-close, and expiry with an advanced test clock |
| Browser session checks | A delayed identity response exposed the premature video open. A real HTTP 401 on the local fixture exposed the rejected-session retry |
| Browser rendering comparison | Local WebSocket telemetry and JPEG feed, with counters in temporary copies of the client |
| Android physical performance | The A54 graphics counter contained zero rendered frames in the available capture. There is no usable frame-time or jank measurement for this revision from this review |

The repository’s earlier A54 soak reported a 19 ms median frame time and 89% janky frames for the rendering implementation that preceded this change. Use those figures as historical context. A comparison with this revision needs the same phone, similar workload, and a capture window with active frames. The browser fixture and earlier soak also used different video content, so the soak bandwidth estimate and the browser binding counts answer different questions. [Soak context and rendering caveat](VALIDATION.md#L143)

## Suggested implementation order

1. **Fix browser connection ownership.** Address C2 and C3 with one current-attempt identity check. Cover resume, sign-out, rejection, and viewer-capacity recovery before changing render scheduling.
2. **Resolve the Android correctness issues.** Establish safe bitmap ownership for C1 and classify transient telemetry TLS failures for C4. Capture an A54 playback baseline with a nonzero frame count.
3. **Land the measured web render improvement.** P1 has a small first change and a repeatable workload. Verify stale-state handling and navigation, then measure the additional benefit of skipping hidden-page bindings.
4. **Reduce unused video demand on both clients.** P2 has the clearest bandwidth payoff. Measure socket churn during short scrolls and preserve the original ten-minute deadline.
5. **Profile the remaining Android work separately.** Use idle state emissions for P3, formatter calls and allocations for P4, and tile traffic during a fixed gesture for P5. Separate measurements make each change’s effect attributable.
