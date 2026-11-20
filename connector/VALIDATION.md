# Connector validation — 2026-10-29

## Passed

- Android debug APK and instrumentation APK assembled with AGP 9.1.1,
  Gradle 9.3.1, SDK 37 and JDK 25. Gradle distribution SHA-256 matches the
  checksum published by Gradle and is included in wrapper properties.
- Five Android protocol tests: pairing parsing/validation, profile JSON,
  source/schema identity, stale runtime/bridge suppression and monotonic expiry.
- Android lint: zero errors. Remaining warnings include newer dependency
  versions, the intentionally pinned custom trust manager, and a KTX style
  suggestion. Dependencies remain pinned to the tested versions.
- Android 17 emulator: live TLS WebSocket data from the Python simulator,
  matched device identity, background/disconnect invalidation and rejection of
  the wrong certificate pin. Exercised phone portrait and tablet landscape.
- Portrait and landscape screenshots inspected for layout, navigation and
  clearly labelled simulated speed data.
- Ten Python API/state/health tests passed on both Windows and Jetson's
  Python 3.10, including independent expiry of stalled health collection.
- Native ARM64 build of `dashcam_v0_4`, `monitoring_test` and `config_test`
  inside the Jetson's `l4t-ml-gpio` image. Compiler flags include `-Wall -Wextra`.
- C++ monitoring test: finite/null serialization, source validity, actual Unix
  socket delivery and 10,000 nonblocking sends with a stalled/missing receiver.
- All 15 existing configuration suites, including the new monitoring fields'
  round trip, passed on the Jetson.
- Real end-to-end Jetson smoke test: v0.4 (no camera/serial devices) → Unix
  datagram → host Python agent → pinned TLS WebSocket; missing vehicle readings
  remain null, host memory counters are present, and stopping the runtime clears
  recorder data while host health remains available.

## Test isolation

Jetson: `192.168.0.15`, ARM64, L4T R36.5.2. Source and build outputs were staged
under `~/.cache/dashcam-connector-validation`, separate from the deployed
checkout. Build and smoke-test containers had no host camera/serial devices
exposed. No production config, service, recording binary or camera state was
changed. Test processes were stopped after validation.

## Browser client (local CA) — 2026-10-29, on the Jetson

- `test_service.py`: 30 tests pass (Python 3.10, aiohttp 3.14.3, cryptography 46.0.3): the original 10 plus the
  CA/leaf profile, chain verification with the `ssl` module (CA file, hostname check by IP SAN), SAN validation and
  the fixed name constraints, `cert` / `ca` / `pair`, atomic writes, static whitelist and security headers, the
  session cookie (creation, attributes, tampered/expired/other-token rejection, sign-out), Host/Origin checks
  (cross-site WebSocket refused) and the unchanged Android bearer path.
- OpenSSL 3: the chain verifies by IP and DNS SAN and fails for another IP; a leaf for `www.google.com` / `8.8.8.8`
  forged with the real CA key fails with "permitted subtree violation". Leaf validity is exactly 825 days.
- Headless Firefox 157 (Marionette) against `serve --simulate` over TLS, 19/19 checks: pairing from the browser link
  strips the token from the address bar, the cookie is HttpOnly and nothing is stored in Web Storage; live data on
  Live/Vehicle/System/Connection at 1280x800 and 412x915; the session survives a reload and an agent restart; a
  WebSocket and a credentialed fetch from another origin (same host, other port) get no data; the page blanks when
  the agent stops and 2.15 s after it freezes with the socket held open (SIGSTOP), then recovers; sign-out returns
  to the not-paired state; pasting the Android link also pairs. Those runs accepted the certificate through
  `acceptInsecureCerts`. Separately, with overrides off and the CA added to a fresh profile's own store (privileged
  Marionette call to `nsIX509CertDB.addCertFromBase64`, trust `C,,`), Firefox 157 loaded the deployed agent at
  `https://192.168.0.15:8443/` with no warning: its verifier (mozilla::pkix) accepts the chain and name constraints.
- Firefox 158 on Windows 10 (the user's laptop) warned after the CA was installed with the Windows certificate wizard
  (Current User store): Firefox does not read that store. Importing it under Firefox's Authorities is required
  (README, "Browser access").
- `smoke_jetson.py` passes again against build `bin/build_20261029_092154` (it now picks the newest timestamped
  build; ad-hoc `build_kz3_*` directories had sorted after the dated ones and selected a pre-monitoring binary).
- Not yet done: installing the CA and pairing a browser on the real phone/tablet/laptop (Chrome, Samsung Internet,
  Safari).

## Camera video — 2026-10-29, on the Jetson

- Built with plain `make` (`bin/build_20261029_143117`): `monitoring_test` (header layout, demand-file freshness,
  missing/stalled/refused agent, byte-exact delivery, a camera JPEG through a real VideoPreview to one 640x360
  datagram only while demanded), `config_test` (16 tests incl. the new keys and the fps fallback),
  `videopreview_test` and `dashcam_v0_4 --self-test` pass. `test_service.py`: 46 tests pass (video validation,
  latest-frame-only, `/v1/video` auth identical to `/v1/stream`, binary header, 10 fps cap, stale frames, separate
  4-viewer limit, 1008 on client messages, demand file only while viewers, CSP `blob:`).
- Real camera end to end, 14/14: the UGREEN at 1080p30 recorded by v0.4 (isolated container, only the UGREEN
  device nodes, temp config/footage/socket dir) → agent → `/v1/video`: no demand file and nothing converted with no
  viewer; 116 frames in 12 s at a steady 9.8 fps, all 640x360 complete JPEGs (median 10.7 KB, ≈ 0.85 Mbit/s), age
  at send median 106 ms (max 158 ms), sequence increasing; headless Firefox showed a 640x360 frame on Live within
  1.0 s as a blob: URL; after the viewers left the demand expired and v0.4 logged "no viewer — idle"; v0.4 exited 0
  on SIGTERM with 199 frames converted, 199 sent, 0 dropped, conversion p95 27 ms; recording continued (30 fps).
  v0.4 container CPU rose from ~7.6 % to ~35 % of one core while a browser watched. The frames were checked
  numerically only (the bench camera may see people).
- The Android camera card (`MainActivity.kt`, `MonitorRepository.kt`, `Models.kt`) was reviewed but **not compiled**:
  this Jetson has no JDK/Android SDK. Build and run `:app:testDebugUnitTest :app:lintDebug` on the Windows machine.

## Deployed agent, browser and Android on real devices — 2026-11-20

Agent installed as `dashcam-monitor.service` (venv, hardened unit: peers limited to localhost, 192.168.0.0/24 and
mDNS; Docker/jtop sockets and the CA key hidden), v0.4 publishing through the `/run/dashcam-monitor` mount. Jetson at
192.168.0.138 (leaf re-issued for it, same CA). No camera and no C3 attached: recorder not recording, vehicle stale.

- **Firefox 158, Windows 10 laptop** (CA in Firefox's Authorities): no certificate warning, pairing, all four tabs,
  phone width, reload; simulator round with test-card video (640x360, 9.9 fps) and agent freeze/stop recovery; then
  real host health and recorder state.
- **mDNS on the real router:** the laptop resolved `dashcam-372d29b6.local` to 192.168.0.138 (no client isolation).
- **Samsung S25 Ultra**, debug APK built on Windows (`assembleDebug testDebugUnitTest lintDebug` successful):
  - Pairing from a terminal-copied link failed ("Could not save pairing ... device storage") because the copy was
    wrapped; it worked with the line breaks removed. Fixed in `3d3d177` (whitespace dropped, link errors reported as
    such, three unit tests); the rebuilt APK is installed.
  - Live (status, Camera "No video · Waiting for camera frames", speed "Waiting for a fresh vehicle source"),
    Vehicle (all values "–"), System (CPU, GPU, uptime, RAM 1.99/7.43 GiB, footage disk 359.1/936.4 GiB free,
    six thermal zones about 51 °C, ADAS "Not available"), Connection (paired "Car Jetson" 192.168.0.138:8443).
  - Agent frozen 15 s: "Data stale · waiting for Jetson"; agent SIGKILL: "Connection lost · retrying" (video card
    too), systemd restarted it in 3 s (`Restart=on-failure`) and the app returned to Live without pairing again;
    recorder telemetry fresh again without restarting v0.4 (the kept `/run/dashcam-monitor` works).
  - Landscape, background/foreground (stays connected), Wi-Fi off ("Waiting for Wi-Fi") and on (reconnects),
    "Find Jetsons" (1 device found by mDNS), local-network permission removed and granted again.
- **Graceful exit with monitoring on:** isolated `timeout --preserve-status -s TERM` run exits 0 with the preview
  converter active; real `systemctl stop` 0.45 s, `ExecMainStatus=0`.

## Vehicle map and review fixes — 2026-11-20 (software)

A Map card on the Vehicle page of the web client and the Android app, above GNSS position (`README.md`, Vehicle map):
OSM tiles fetched by the phone/browser itself, follow mode, free pan/zoom, Recentre, last known position.

- **Web, headless Firefox against the simulator** (Marionette harness, 22 checks, all pass): no tiles or OSM requests
  before a fix or off the Vehicle page, card position and width, tiles load with `strict-origin-when-cross-origin`,
  fresh marker with heading, attribution, drag/free mode/Recentre, +/− and wheel zoom, phone width without page
  scroll, touch drag and pinch, requests only to the agent and `tile.openstreetmap.org` (Referer, no Cookie),
  bounded tile volume, no console errors or CSP violations, stale data keeps a grey "Last known position". The bench
  router blocks openstreetmap.org DNS, so the harness used DNS over HTTPS.
- **Review of the map change and the clients** (6 findings, all confirmed, all fixed):
  - Web: a link pasted into Connection → Pair for another host:port is refused before the token is sent (harness:
    0 requests for three wrong-target links, 1 for a wrapped link for this page).
  - Agent: open browser telemetry/video sockets close with 1008 when the session cookie expires; Android (bearer)
    sockets are unaffected (new agent test).
  - Android: forgetting a device that is not shown no longer switches or reconnects the shown one; video frames are
    stamped on receipt and dropped as stale before and after decode (`VIDEO_STALE_MS`).
  - Web: a failed tile is transparent, so tiles cached from nearby zooms stay visible; the marker uses the shortest
    wrapped distance across ±180° (harness: one world width of panning at zoom 3 returns the marker to its pixel).
- **Agent crash loop found on the bench:** with the Jetson on another network, 192.168.0.138 was not assigned and
  zeroconf raised ENODEV at start, so the unit restarted 435 times. mDNS is now best-effort (an address probe before
  zeroconf; the agent logs "mDNS advertisement off" and serves). New agent test; the deployed agent recovered.
- 48 agent tests pass. The Android map and fixes are **not compiled** here (no JDK/Android SDK): build and run
  `:app:assembleDebug :app:testDebugUnitTest :app:lintDebug` on the Windows machine (adds `TileMathTest`).

## Remaining physical acceptance

- Vehicle map with real tiles: the S25 Ultra app (after the Windows build) and laptop Firefox with internet.
- Tab S10 FE (app and browser); browsers on the phone/tablet (CA install). The S25 Ultra app and laptop Firefox
  passed on 2026-11-20 (above), as did mDNS on the router.
- CAN/GNSS source-loss transitions, reconnect after router/Jetson reboot and
  accurate storage-path configuration on the deployed system.
- Compare recording FPS/CPU/memory before and during two physical viewers.
- Enable the monitoring socket directory mount and XML settings in the deployed
  recording launcher, then verify with live vehicle telemetry.

ADAS, remote commands, background monitoring and a
distribution signing key are outside this first connector release.
