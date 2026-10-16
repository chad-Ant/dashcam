# ES3C28P navigation firmware implementation

Follow-up: [2026-10-16 review/refactor](REVIEW_2026-10-16.md) records startup,
resource and parsing fixes, graphics source separation, and current validation.

Implemented 2026-10-14 against the current NAVIGATION_PLAN.md and the already
frozen Es3MapProtocol.h. The navigation, video and voice plans are unchanged.
This record describes the firmware changes; it does not supersede the plan.

## Panning extension v2 (2026-10-14)

The board now supports the additive protocol already implemented by the Jetson.
The shared protocol headers and all three plans were preserved unchanged during
this firmware update. Whole-frame START_MAP and camera START_VIDEO still work.

- PAN_CAPABILITIES (0D) advertises 512x384 maximum backgrounds, 16-pixel top
  and bottom strips and a maximum pose input rate of 30 Hz. START_PAN (0E)
  validates the exact frozen request and opens a map session; STOP_MAP ends it.
  Map/panning replacement and camera exclusion retain their existing semantics.
- Background JPEGs and 320x32 strip JPEGs share the credited E3 29 receiver.
  The background START timestamp is retained as its packed world-pixel origin.
  Dimensions select the image kind. Waiting images supersede only their own
  kind; a new background cannot discard a pending attribution/status image.
- MAP_POSE accepts only valid, increasing sequences for the current visible
  panning token. Bad fields, duplicates, old sequences, other tokens and poses
  outside panning mode are ignored. Pose records require no credit and generate
  no reply. They may arrive between JPEG chunks without changing assembly state.
- The board applies the latest host pose directly; smoothing remains on the
  Jetson. Integer crop placement uses the shared computePanView helper. Marker
  rendering uses the specified blue/white dot, rotated arrow, or gray last-known
  ring, clipped to the map area. A fixed-point sine table handles rotation;
  no further GPS interpolation, extrapolation or geographic projection is done.
- Each redraw freezes one pose and the retained image leases until its strips
  finish. New poses are coalesced for the following redraw. Drawing copies at
  most eight rows per loop, preserving touch/control responsiveness. Unchanged
  crops/markers do not redraw; unchanged chrome is retained on the panel.
- A valid pose or newly decoded image refreshes image freshness. Thus regular
  poses keep a stationary background live. On stale/disconnect, only the top
  status strip gets a local notice. A fresh pose restores the retained host
  status strip without needing a new strip JPEG. Before required assets/pose
  arrive, the local waiting notice remains visible.
- Background and chrome buffers remain leased after their first display.
  PRESENTED is emitted once per received image, not on each pose redraw. Release
  after presentation is silent. Hiding the map or resetting/replacing a session
  releases these leases; the host resends assets when visibility returns.

PSRAM allocation is bounded and performed before the worker starts:

| Buffers | Bytes |
| --- | ---: |
| Two 65,536-byte compressed inputs | 131,072 |
| Two 512x384 RGB565 backgrounds (also used by camera/whole frames) | 786,432 |
| Two 320x32 RGB565 chrome buffers | 40,960 |
| Total image buffers with panning | 958,464 |

The startup upgrade temporarily coexists with the original small pixel buffers
(peak image allocation 1,265,664 bytes). If the optional upgrade fails, its
allocations are freed and the original 438,272-byte whole-frame pipeline stays
available. 0D/0E return NotReady, allowing the existing Jetson fallback. No runtime
heap allocation is introduced in the receive/decode/draw path. JPEGDEC validates
both the requested dimensions and explicit output capacity before decoding.

Additional validation:

- C++11/C++17 pan wire vectors and receiver/raster tests, plus the existing
  control/video/map suite and 38 Python tests.
- ASan/UBSan receiver and raster checks: interleaved poses/chunks, ignored poses,
  token and mode replacement, same-kind supersession, retained decode/draw leases,
  temporary buffer contention, crop/source alignment, edge clipping, marker
  directions/colors and output-buffer canaries.
- JPEGDEC fixtures include 512x384 and 511x383 baseline 4:2:0 backgrounds and
  320x32 baseline 4:4:4 strips, with output-capacity and malformed-header checks.
- Arduino ESP32 3.2.0 / JPEGDEC 1.8.4 builds pass with `--warnings all` and no
  warnings: HWCDC uses 559,370 bytes flash / 54,740 bytes static RAM; TinyUSB
  uses 595,914 bytes flash / 73,900 bytes static RAM. The PSRAM image allocations
  above are additional to static RAM.

The earlier build sizes below describe the original whole-frame implementation.
Panning bandwidth, redraw rate, tearing and end-to-end smoothing still require
hardware measurement; protocol acceptance of 30 poses/s is not a measured panel
refresh-rate guarantee. The existing UART presented-frame count measures received
images, not local pose redraws.

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
- The existing core-0 JPEGDEC worker and credited receiver serve both sources.
  Panning upgrades pixel buffers as described above; no network stack was added.
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
  static RAM; TinyUSB uses 591,526 bytes flash / 73,372 bytes static RAM.
  These are the pre-panning build figures; see the extension record above for
  the updated buffer allocation.

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
7. With panning enabled, verify 0D/0E negotiation, 512x384 backgrounds and
   320x32 chrome, then a long pose-only interval (20 Hz) without stale notices.
   Turn through cardinal directions, stop (dot), lose GPS (last-known ring),
   cross a background boundary and change status/attribution independently.
   Verify old markers leave no trails and no decoded background overwrites a
   strip buffer. Stop poses as well as images to test the stale deadlines.
8. Force optional allocation failure and verify whole-frame fallback; switch
   panning -> whole-map -> camera repeatedly while decoding. Measure actual
   redraw cadence and USB bytes against the host's simulated-drive figures.

Build and portable tests establish software checks, not panel performance or
physical USB throughput.
