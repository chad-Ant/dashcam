# ES3C28P navigation firmware implementation

Implemented 2026-10-14 against the current NAVIGATION_PLAN.md and the already
frozen Es3MapProtocol.h. The navigation, video and voice plans are unchanged.
This record describes the firmware changes; it does not supersede the plan.

## Board behavior

- Six tabs: LIVE=0, TRIP=1, CAM=2, SYS=3, SET=4, MAP=5. Drawing, hit testing
  and calibration-test targets use the same boundaries:
  0, 53, 106, 160, 213, 266, 320. PAGE accepts index 5 and STATUS reports it.
- After HELLO, 0A returns version 1, six pages, map page 5, viewport
  (0,0,320,224), JPEG limit 65,536 bytes and flags 1. Missing image memory
  returns NotReady while ordinary control commands remain usable.
- 0B validates the frozen 27-byte START_MAP request before changing state.
  The receiver binds source, opaque host generation, limits and fresh token
  under one mutex and replies with that token and initial credit 1.
- A camera session rejects START_MAP, and a map session rejects START_VIDEO,
  with NotReady. Same-source replacement preserves the existing video start
  behavior and receives a fresh token. Hosts stop the old source before switching.
- STOP_MAP and STOP_VIDEO are source- and token-specific. A stale token or a
  token belonging to the other source changes nothing. HELLO, USB loss or
  session expiry resets either source through the existing link reset path.
- Every start emits VISIBILITY based on the latest graphics view. Maps are
  active only on MAP; camera frames only on LIVE. Calibration or map-information
  overlays suspend image demand. Opening the map never selects a tab remotely.
- The existing core-0 JPEGDEC worker and two JPEG/two RGB565 PSRAM slots serve
  both sources. No second image pool or network stack was added to the board.
  Active decode/draw leases survive resets until released, and old-token
  completions cannot publish or return credit to the new session.
- Decode jobs also carry a local visibility epoch: leaving a view invalidates
  a pending decode even if the user returns before it finishes. Fresh frames
  can resume under the same session without displaying that old decode.
- The application loop remains the sole TFT/touch/I2C owner. It draws at most
  eight image rows per turn, with map images ending at y=223 above the tabs.

Map images already contain the Jetson's position dot/arrow, tiles, GPS status
and attribution. Firmware does not reinterpret GPS or render a compass. Camera
preview gating and continued recording belong to the existing Jetson coordinator;
board VISIBILITY and source ownership provide the corresponding firmware behavior.

## Local display states

The MAP tab initially shows a waiting/offline message, or an image-memory
unavailable message if the image service failed to initialize. No SD card is
required for map frames; existing touch calibration still needs persistence
or completion of the calibration wizard.

With an active map session, the board honors negotiated stale/placeholder
times (normally 2 s / 5 s). Stale, unavailable and disconnected notices only
paint y=0..15. Map pixels and the streamed provider/OSM attribution at y=208..223
remain intact. An ended session is immediately qualified as stale; heartbeat
traffic alone cannot refresh an image. The next complete frame restores the
host's status strip. A source change or page exit clears the old surface.

Tap the attribution strip to show local OSM/ODbL information and
`openstreetmap.org/copyright`; tap the content again to dismiss, or select
another tab. The overlay suppresses image demand and preserves tab access.
Local placeholder/info screens use an explicit copyright bitmap rather than
assuming a font's extended character encoding. Streamed attribution is not
replaced by a firmware credit overlay.

The frozen header and existing E3 28/E3 29 framing, CRCs, reasons and golden
vectors were not changed. The Python helper adds map start/stop/capability
methods and permits PAGE 5 only after a valid map capability reply, preserving
older firmware behavior. PAGE updates text; it does not select a tab or replace
the Jetson-rendered map content.

## Validation

- `tests/run_tests.sh`: existing calibration/control/video regressions,
  C++11/C++17 map protocol and surface tests, byte-identical host/firmware
  header checks, and 38 Python tests including capability-gated page 5.
- Surface tests cover every tab pixel, visibility, invalid map settings,
  exact frame dimensions, cross-source exclusion, stale/wrong-source stops,
  HELLO-style reset, decode/draw leases across replacement, and a rapid
  hide/show during decode. The new surface test also passes ASan + UBSan.
- `tests/run_decoder_tests.sh`: adds a 320x224 baseline 4:4:4 JPEG fixture to
  the existing bounds, RGB565 color, malformed and progressive-image tests.
  Wrapper/tests run with ASan + UBSan; pinned upstream JPEGDEC runs with ASan
  only, for the previously documented upstream alignment/arithmetic behavior.
- Build commands remain `build_and_upload.sh` and `BuildAndUpload.cmd`, with
  the new local es3Map library included. Benchmark builds also include it.
  PlatformIO discovers it from lib/; PlatformIO/Windows execution is untested.
- Arduino ESP32 3.2.0 / JPEGDEC 1.8.4 builds in `l4t-ml-gpio:latest` pass with
  `--warnings all` and no warnings: HWCDC uses 555,038 bytes flash / 54,212 bytes
  static RAM; TinyUSB uses 591,526 bytes flash / 73,372 bytes static RAM. Dynamic
  PSRAM image allocations remain the existing 438,272 bytes for either source.

## Hardware acceptance still required

No device was flashed or physically tested for this change. Connect the existing
Jetson navigation implementation and check:

1. HELLO/CAPABILITIES/MAP_CAPABILITIES, all six touch targets, page 5 in STATUS,
   and a map shown at (0,0) with its status, attribution and tabs intact.
2. LIVE -> MAP stops preview traffic, MAP -> LIVE resumes fresh footage,
   recording continues, and other tabs pause the bound surface. Repeat 100 times,
   including switches while a JPEG is decoding or only partly drawn.
3. Attribution overlay opening/closing pauses/resumes images, SET calibration
   still works, and a held touch does not repeatedly toggle or select tabs.
4. Stop image production while keeping PINGs alive: stale/unavailable notices
   appear at negotiated deadlines and never erase attribution. Disconnect USB,
   then reconnect and verify a fresh handshake/token and image.
5. No GPS, unreliable course/low speed (dot), network outage/cache gaps, missing
   camera, preview disabled and unavailable PSRAM. GPS and tile states are
   supplied by the Jetson; loss of display service must not stop recording.
6. Measure first-map and update latency, sustained 4 fps, tearing, readability,
   touch responsiveness and a 30-minute recording/transition/outage soak.

Build and portable tests establish software checks, not panel performance or
physical USB throughput.
