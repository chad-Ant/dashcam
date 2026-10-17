# ES3C28P video firmware implementation

Follow-up: [2026-10-16 review/refactor](REVIEW_2026-10-16.md) records startup,
resource and parsing fixes, graphics source separation, and current validation.

Implemented against the **Frozen protocol v1** and **Review additions** in
[VIDEO_STREAMING_PLAN.md](VIDEO_STREAMING_PLAN.md). That plan was not modified.
The Jetson application and camera capture changes are outside this firmware work.

## Tasks and ownership

- Core 0, priority 2: USB task owns native CDC, the multiplexing parser, pending
  control reply and partial writes. RX is 8 KiB; each turn drains at most 4 KiB
  then yields one RTOS tick. A bounded application-command queue preserves the
  existing one-outstanding-control-request contract. Control replies take
  priority over queued video notifications. A 1 s TX stall or 5 s heartbeat
  expiry resets the session. Video traffic does not substitute for heartbeats.
- Core 0, priority 1: JPEG worker owns JPEGDEC **1.8.4**, decodes outside the
  state mutex, and publishes only a complete, viewport-sized baseline image.
  It validates bounded JPEG marker segments and SOF dimensions before opening
  the decoder. Progressive files, dimension mismatches, missing EOI, failed
  decodes and incomplete output are rejected. No per-frame heap allocation.
- Core 1, Arduino loop: owns TFT, touch, shared I2C, SD and audio. It draws one
  eight-row strip per iteration from an internal-RAM staging buffer, servicing
  controls/touch/audio between strips. The former 10 ms delay is now 1 ms.
  PRESENTED follows the last synchronous TFT transfer; it is not a physical
  panel-refresh acknowledgement.

`VideoState.h` contains portable assembly and ownership logic. `Es3Video`
serializes its state transitions with a mutex; copying one bounded USB chunk
and its CRC update happen under the lock, while JPEG decode and SPI writes do
not. JPEG slots follow FREE/RECEIVING/READY/DECODING; output slots follow
FREE/DECODING/READY/DRAWING. Decode and display ownership are distinct. A reconnect never frees a buffer
still leased by a decoder or the TFT owner. Returning an old lease frees its
slot without granting credits or emitting notifications into the new session.

The two 64 KiB JPEG slots plus two 320×240 RGB565 images use **438,272 bytes of
PSRAM**. The decoder, task stacks, queues and 5,120-byte strip buffer use
additional internal RAM. PSRAM/pool/task failure leaves controls usable and
CAPABILITIES returns NotReady. Pools and worker tasks live for the firmware's
lifetime; STOP_VIDEO resets a session, it does not destroy tasks or pools.

## Wire and failure behavior

The frozen protocol headers and golden vectors are unchanged. Capabilities
advertise 320×240, baseline JPEG, 65,536-byte frames, 1,024-byte chunk data and
**one credit**. START_VIDEO validates against these limits and returns a new
nonzero token plus initial grant 1. Both hardware CDC and TinyUSB builds are
supported; hardware CDC remains the default pending throughput measurements.

Assembly checks exact next offsets, declared byte counts, viewport dimensions,
CRC32, increasing frame IDs and negotiated assembly deadlines. Stale-token
records are ignored. A credited rejected START returns its credit; NoCredit
and duplicate START do not mint credits. Assembly errors/ABORT release once;
decode completion returns credit when its JPEG slot becomes reusable, even on
decode failure. READY output can be superseded, DRAWING output cannot.

RESULT Ok acknowledges assembly. PRESENTED acknowledges software display;
a later RESULT reports decode/visibility/supersession failure. A fixed 32-event
queue never silently drops a grant: overflow faults the session and forces
HELLO/negotiation again. USB loss, HELLO and heartbeat expiry invalidate the
session. STOP_VIDEO with an old token is a no-op. Commands handed to the loop
carry a connection generation so queued old-session commands/replies are
rejected; a command already executing when USB disconnects may still finish.
Alerts are never replayed by this firmware.

## Display behavior

For a negotiated 16:9 image the host supplies viewport **(0,30,320,180)**. The
board preserves these dimensions, clears the bars on entry and keeps the tabs
at y=224. Video replaces the LIVE splash and suppresses its light diagnostic.
Page text remains stored without clearing an active video image on each update.

The device emits VISIBILITY after negotiation and on changes: active on LIVE,
hidden on another tab, calibrating throughout calibration and verification.
Hidden/calibrating demand discards waiting input/output and prevents new images
from appearing over other screens. If video overlaps the tab strip (such as
320×240), the first tap reveals tabs for three seconds, and a second selects
one. Video resumes without stretching or cropping.

Without a presented frame for the negotiated stale interval (normally 500 ms),
the indicator marks the image stale; after the placeholder interval (normally
2 s), it displays “Video unavailable.” USB loss retains the last image until
these deadlines. The next successfully decoded image replaces the placeholder.
For a viewport with no top bar, the stale notice temporarily overlays stopped
footage and is replaced by the next frame.

## Focused streaming review and refactor (2026-10-16)

Reviewed the USB receiver/credit flow, JPEG worker, retained map images and
whole-frame display path after commit `6fdb0c0`. The design plans and shared
wire headers remain unchanged. Jetson service changes remain outside this work.

Findings and changes:

- **P2 — session replacement between view and acquire could discard a new
  image.** Graphics previously acquired whichever session was current, then
  released a token mismatch as Hidden. If USB replaced the session between
  those calls, the new session's first decoded image was consumed. Acquisition
  now accepts the expected view token and checks it under the same mutex as
  the slot transition. A mismatch leaves the ready image queued; graphics
  does not acquire while its view token is zero. A regression simulates a
  snapshot from session 6 followed by a ready image in session 7.
- **P2 — lease return APIs trusted indices and identity.** A duplicate
  `finishJob()` could return a second credit and reclaim a buffer that had
  already moved to drawing. A stale copied image handle could free a reused
  slot; an out-of-range release could index outside the pool. The decoder and
  display now have distinct DECODING/DRAWING states. Returns validate indices,
  token/frame identity, expected state, and (for images) buffer address before
  mutating anything. The duplicate-completion regression was compiled against
  the previous implementation and failed at the credit-count assertion.
  These are internal API robustness cases, not evidence that normal USB input
  made the existing single worker call completion twice.
- Acquisition refuses to overwrite a live image handle, and job acquisition
  refuses to overwrite a live decoder handle. Presentation is acknowledged
  once, only for an owned image in the current active session. Hidden release
  reports Hidden for an image not previously presented. Old-session valid
  leases can still be returned safely without notifications to the new session.
- Mutex lifetime is expressed with a noncopyable scoped guard. Worker take and
  finish each have their own explicit scope; decoding and yielding remain
  outside the lock. No additional task, lock, queue or per-frame allocation.
- `PixelCopy.h` isolates decoder block bounds/accounting and whole-frame strip
  copying from hardware APIs. It rejects invalid coordinates/strides and an
  exhausted output budget before writes, clips padded bottom MCU rows, and
  requires the image and negotiated viewport to match before copying a TFT
  strip. A failed strip copy releases the image without claiming PRESENTED.
  Real JPEGDEC fixture tests continue to verify odd dimensions and RGB565 colors.

Validation includes the native C++11/C++17 suite, 39 Python tests, ASan/UBSan
receiver/map/raster/resource/pixel checks, and the pinned JPEGDEC fixtures.
New cases cover replayed completions, reused slots, malformed handles,
acquisition into a live handle while another image is ready, hidden
presentation, replacement-session acquisition, pixel canaries, extreme callback
coordinates, padded rows, viewport mismatch and insufficient strip capacity.
The pixel count is a completion budget, not a proof of spatial coverage for
arbitrary synthetic overlapping callbacks; JPEGDEC remains responsible for
its block traversal and entropy parsing.

Image PSRAM remains 958,464 bytes with panning (startup peak 1,265,664 bytes),
or 438,272 bytes in whole-frame fallback. The 5,120-byte TFT staging buffer,
8 KiB worker/USB stack reservations, and eight-row draw budget are unchanged.
Both Arduino 3.2.0 builds passed with all warnings enabled and no warnings:

| USB mode | Flash bytes | Static RAM bytes |
| --- | ---: | ---: |
| HWCDC | 544,194 | 54,748 |
| TinyUSB | 580,786 | 73,908 |

Static RAM increased by 8 bytes from `6fdb0c0`. All 22 host display-link
integration scenarios passed against the current workspace, including the
firmware receiver, reconnect/credit handling and navigation panning (0 failures).
Hardware throughput, tearing, display timing and stack high-water marks still
require bench measurement. No board was flashed.

## Validation and measurements

```sh
peripherals/es3c28p/tests/run_tests.sh
# Requires a native C++ compiler, Pillow, and the pinned JPEGDEC source:
JPEGDEC_SRC=/root/Arduino/libraries/JPEGDEC/src \
  peripherals/es3c28p/tests/run_decoder_tests.sh
peripherals/es3c28p/build_and_upload.sh
ES3_USB_MODE=default peripherals/es3c28p/build_and_upload.sh
```

Validation on 2026-10-13 with Arduino ESP32 3.2.0 and `--warnings all`:

| Build | Flash bytes | Static RAM bytes |
| --- | ---: | ---: |
| Dashboard hardware CDC | 552,302 | 54,036 |
| Dashboard TinyUSB CDC | 588,770 | 73,196 |
| JPEG-from-SD bench | 483,318 | 46,508 |
| USB sink hardware CDC | 334,394 | 21,132 |
| USB sink TinyUSB CDC | 372,246 | 40,316 |

All builds completed without compiler warnings. The portable C++ checks and
17 Python tests passed, as did the native codec fixtures. Static RAM figures
exclude runtime PSRAM pools and task/queue allocations. PlatformIO and the
Windows wrapper were updated but were not executed in this Linux validation.

The portable suite checks golden vectors/header-copy identity, parser recovery,
Python pseudo-terminal behavior, credit recovery, CRC failures, hidden demand,
timeouts, buffer ownership through resets, decode failure, supersession, and
bounded notification overflow. Codec fixtures cover 4:2:0/4:2:2 RGB bars,
grayscale, odd edges, progressive/oversized/truncated/malformed files and output
guards. The wrapper/tests use ASan+UBSan; upstream JPEGDEC uses ASan alone because
its ARM path emits alignment, shift and arithmetic UBSan findings. Those
upstream findings are not fixed by this firmware; only vendor UBSan
instrumentation is excluded. This does not test S3 SIMD code.

Every five seconds UART0 reports cumulative assembled/decoded/presented frames,
presented FPS×10, incoming JPEG byte rate, last JPEG size, drops, CRC/timeout/
decode failures, last decode/blit durations and free heap/PSRAM. The host can
measure round-trip presentation latency with the frozen PRESENTED fields;
board timestamps are not subtracted from the unsynchronized Jetson clock.

[bench/README.md](bench/README.md) describes the standalone USB sink and
JPEG-from-SD sketches, building both USB modes, color checks and combined soak.
Actual 10–15 fps, p95 latency, touch/audio responsiveness, USB reconnect,
S3 SIMD color output, scan tearing and stable memory require the physical board.
No hardware performance claim follows from compilation or native tests.
