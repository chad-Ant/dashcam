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

## Remaining physical acceptance

- Installation and pairing on the actual Samsung S25 Ultra and Tab S10 FE (app and browser, after installing the CA).
- mDNS discovery on the actual router, including client-isolation behavior.
- CAN/GNSS source-loss transitions, reconnect after router/Jetson reboot and
  accurate storage-path configuration on the deployed system.
- Compare recording FPS/CPU/memory before and during two physical viewers.
- Enable the monitoring socket directory mount and XML settings in the deployed
  recording launcher, then verify with live vehicle telemetry.

ADAS, maps, camera preview, remote commands, background monitoring and a
distribution signing key are outside this first connector release.
