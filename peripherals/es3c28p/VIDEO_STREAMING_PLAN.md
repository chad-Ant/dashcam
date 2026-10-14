# Ugreen → Jetson → ES3C28P USB video preview

Status: **planned**, recorded on 2026-10-13. No streaming code is implemented
by this document. This is the implementation handoff for the ES3C28P board.

The agreed target is a live preview at **10–15 fps**, with the full camera
image preserved using letterboxing. Use the ES3C28P's existing native USB CDC
connection. Recording remains the primary workload.

## Display and performance contract

- The display canvas is 320×240. A 16:9 Ugreen source fits as **320×180**, with
  30-pixel black bars above and below. This is the precise interpretation of
  “240p” on this 4:3 panel. Do not stretch or crop the camera image.
- Show video on LIVE. For this 16:9 layout, retain the 16-pixel tab strip inside
  the lower black bar. Put connection/stale indicators in the bars, and suppress
  the current light-sensor diagnostic overlay inside the video rectangle.
- Negotiate the viewport from the actual source aspect ratio. If a source uses
  the whole panel (for example 4:3), use tap-to-reveal navigation rather than
  obscuring or distorting the image. Calibration always takes over the screen.
- Begin with 10 fps and make 15 fps configurable after measurement. Target
  capture-to-presentation-ack p95 latency below 250 ms at the supported rate;
  this is an acceptance target, not a measured capability.
- Pause video demand outside LIVE and during calibration. Keep touch, audio,
  page/status commands and heartbeats working while video is running.
- After 500 ms without a new presented frame, visibly mark the image stale;
  after two seconds, show a “Video unavailable” placeholder. Resume with the
  newest complete frame after recovery.

## Findings from this repository and device

1. The connected camera is available at
   `/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0`.
   Read-only V4L2 enumeration confirms MJPEG modes including 1080p30/60,
   640×360p30/60 and 4K30. It offers 320×240 only as YUYV. Production preview
   must use the recording capture, rather than opening another camera stream
   or changing the recording format to get a small image.
2. `src/dashcam_v0_4.cpp` uses **SegmentedRecorder**. The older `Recorder` has
   `setCompressedFrameCallback()`, but SegmentedRecorder currently only has
   the separate low-rate luma tap. Add the preview hook to the active recorder.
3. The USB control protocol has 128-byte payloads, one request/reply at a time,
   and a loop containing `delay(10)`. It is suitable for controls, but its
   service policy must change for video; increasing a baud setting will not
   solve this.
4. The installed Adafruit ILI9341 library uses 40 MHz SPI for ESP32. A full
   320×240 RGB565 transfer has a theoretical wire time of 30.72 ms at that
   clock, before software overhead. Measure the existing display driver before
   considering a replacement or a higher SPI clock.
5. The container has the libjpeg-compatible development headers from
   libjpeg-turbo, but no `turbojpeg.h`. The first implementation can use its
   installed `jpeglib.h` interface; do not assume the TurboJPEG API is installed.

## Review additions (2026-10-13)

Checked against the code and measured on the Jetson with the UGREEN attached
(ES3C28P hardware absent). The firmware compiles in `l4t-ml-gpio` with
`--warnings all` and no warnings (516,150 B app, 25,420 B globals), and
`tests/run_tests.sh` passes (7 tests). Where this section conflicts with the
text below, this section wins.

**1. Jetson conversion, measured.** 60 frames of UGREEN 1080p30 MJPEG (indoor,
static desk scene, AE settled; frames kept out of the repo), converted to
320×180 baseline JPEG with libjpeg-turbo through Pillow on one A78AE core:

| Path | Decode p50 | Total p50 / p95 | Output p50 |
| --- | --- | --- | --- |
| Full 1920×1080 decode, resize, q65 4:2:0 | 15.3 ms | 25.3 / 25.5 ms | 8.1 KiB |
| DCT-scaled 1/4 decode (480×270), resize, q65 4:2:0 | 5.1 ms | 6.8 / 6.9 ms | 8.0 KiB |
| DCT-scaled 1/4, q65 4:2:2 | 5.1 ms | 6.9 / 7.1 ms | 8.7 KiB |
| DCT-scaled 1/4, q50 4:2:0 | 5.1 ms | 6.8 / 6.9 ms | 6.6 KiB |

Source frames were 162 KiB, baseline. Consequences:
- Make DCT scaling mandatory, not optional: `cinfo.scale_num = 1;
  cinfo.scale_denom = 4` (libjpeg), then a 480×270 → 320×180 resize. It costs
  about 7 % of one core at 10 fps instead of 25 %.
- Do not decode in the GStreamer graph: `jpegdec` has no DCT-scaling option,
  so it would pay the 15 ms full decode on a streaming thread. The appsink
  hands compressed bytes to the worker, which does the scaled decode.
- The quality and subsampling choices barely change cost, so they are purely a
  size/quality trade. 4:2:2 is +9 % bytes; start with 4:2:0.
- 8 KiB is a static indoor scene. Night noise and motion will be larger:
  re-measure with drive footage before fixing `MaxJpegBytes` or the FPS. At
  15 fps the measured size is 120 KiB/s.
- 8 MiB input slots (Capture integration) are about 50× a 1080p frame. Use
  1 MiB, and report "source frame too large" if the camera mode changes.

**2. Use the recorder's existing tap pattern.** `SegmentedRecorder` already has
a `rectee` tee with a leaky, rate-limited `luma*` branch (`libsegment.cpp`). Its
bus handler treats errors from elements named `luma*` as exposure-only
failures, not recording failures. Give the preview branch the same treatment:
name every element `preview*`, extend that prefix check, and use `queue
leaky=downstream max-size-buffers=1` + `videorate drop-only=true` + appsink.
Today the tee sits *after* the `recrate` videorate. The preview branch must
come before it, as this plan requires, or a recording FPS cap also caps the
preview.

**3. USB mode is unverified for video.** The build uses `USBMode=hwcdc`: the
S3's USB Serial/JTAG peripheral, which has 64-byte endpoint buffers that the
CPU refills per packet. Its sustained throughput is not measured here, and
its practical rate may be well below the 12 Mbit/s line rate. The feasibility
bench must measure hwcdc and TinyUSB CDC (`USBMode=default`) separately.
TinyUSB trade-offs: a crashed firmware takes the USB port with it (reflash
via the BOOT button); `es3UsbLink.cpp` has an `#error` requiring
`ARDUINO_USB_MODE`; and re-check the VID/PID/serial the udev rule matches.
Stay on hwcdc if it sustains about 3× the measured video rate.

**4. Host writes: the kernel queue defeats priority.** Bytes already written to
the tty sit in the kernel buffer and cdc-acm URBs, and a control frame cannot
overtake them. Before each video chunk, the host I/O worker should check
`ioctl(fd, TIOCOUTQ)` and keep at most one chunk queued. Use `poll()` with
timeouts, never a blocking `write()`, so a stalled board cannot block
heartbeats or reconnect handling.

**5. ESP32-S3 core placement and stall budget.** No radio is used, so core 0 is
free. Run the USB service and JPEG decode tasks on core 0; the Arduino loop
(TFT, touch, SD, audio) stays on core 1. The audio DMA queue is 8 × 128
frames: 128 ms at 8 kHz, 64 ms at 16 kHz. That is the hard ceiling for any
loop stall. A whole 320×180 blit is 115,200 B, about 23 ms on the wire at
the Adafruit library's 40 MHz. That is within budget, so strips mainly buy
touch latency.

**6. Start with a credit window of 1.** The cumulative-grant protocol supports
it unchanged. A frame then costs transfer + decode + blit; the blit alone is
about 23 ms, so this plausibly still reaches 10–15 fps with simpler failure
analysis. Raise the window to 2 only if the measured stages need overlapping.

**7. Two bench sketches for the day the board arrives,** before any protocol work:
- *JPEG-from-SD:* JPEGDEC decodes a 320×180 JPEG made by the Jetson converter
  from the SD card, in a loop. It logs decode and blit ms, and checks byte
  order with the colour and gradient test images. This needs no host code.
- *USB sink:* the firmware discards incoming bytes and reports bytes per
  second over `Serial0`. The host sends 1 KiB chunks at full speed, in both
  USB modes. This bounds item 3.

**8. Toolchain.** JPEGDEC is not installed in `l4t-ml-gpio` (the image only has
the Adafruit display libraries). Pin it in `docker_dev/Dockerfile` next to
Adafruit ILI9341, and in `platformio.ini` `lib_deps` and both build scripts.
The libjpeg headers needed on the Jetson side are present.

## Encoding choice

Send independent **baseline JPEG frames** over USB, then decode to RGB565 on
ESP32-S3. Initially encode only the visible image (320×180 for 16:9); the screen
owner paints black bars locally. Start at JPEG quality 65 and benchmark both
4:2:0 and 4:2:2 compatibility before pinning the encoder settings.

For comparison, an uncompressed 320×240 RGB565 canvas is 153,600 bytes, or
1.536–2.304 MB/s at 10–15 fps. Even the 10 fps case exceeds the USB controller's
12 Mbit/s theoretical line rate. JPEG frame sizes are scene-dependent: use
8–20 KiB as an initial measurement budget, not a promise. At 15 fps that is
about 120–300 KiB/s before framing. Cap a compressed preview frame at 64 KiB;
reduce quality within configured bounds or drop an oversized frame.

Use libjpeg-turbo on Jetson with scaled decoding before final resize, then
baseline JPEG encoding at preview resolution. Run this on a worker, sampling
only frames needed for the preview rate. This design uses CPU JPEG processing
and does not depend on an H.264 encoder/NVENC. Evaluate NVIDIA JPEG/VIC paths
only if the CPU measurements justify the extra integration.

Use a pinned, tested version of **JPEGDEC** on the ESP32-S3. Decode from RAM
into owned RGB565 output, not directly to the screen: only publish an image
when the complete decode succeeds. Validate JPEG dimensions and supported
format before decoding, and test S3 color conversion against known images.

References:
- [ESP32-S3 USB controller speed](https://documentation.espressif.com/esp32-s3_technical_reference_manual_en.pdf)
- [libjpeg-turbo](https://github.com/libjpeg-turbo/libjpeg-turbo)
- [JPEGDEC formats, RAM input and ESP32-S3 support](https://github.com/bitbank2/JPEGDEC)

## Implementation components

| Component | Planned responsibility |
| --- | --- |
| `lib/libvideopreview/` | Jetson JPEG sampling, resize/encode, bounded newest-frame queue, preview configuration and metrics |
| `lib/libes3display/` | One Jetson USB owner for controls and video, handshake, capabilities, scheduling, credits and reconnect |
| `lib/librecord/libsegment.cpp` and `lib/librecord/librecord.h` | Optional compressed preview tap on SegmentedRecorder |
| `peripherals/es3c28p/lib/es3Video/` | Frame assembly, validation, JPEG decode worker and owned image buffers |
| `peripherals/es3c28p/lib/es3UsbLink/` | Multiplex video records with existing controls; separate USB servicing from slow display work |
| `peripherals/es3c28p/lib/es3Graphics/` | Display video strips using the existing TFT instance and handle LIVE/navigation/stale state |
| `src/dashcam_v0_4.cpp`, config and build files | Optional preview integration, lifecycle, device/FPS/quality settings and dependencies |
| `peripherals/es3c28p/host/es3_display.py` | Bench playback and protocol diagnostics; production owns the port through the C++ link |

Suggested host API: `start(config)`, `submitJpeg(bytes, size, captureTime)`,
`setVisible(bool)`, `stats()` and `stop()`. Submission must immediately reject
unsupported/oversized input or replace the waiting frame in a preallocated
pool; it must never wait for decode, USB or a display acknowledgement.

### Proposed API contracts

These are design signatures, not declarations already available to callers.
Keep conversion, transport and graphics independently testable.

```cpp
// Jetson: libvideopreview. Caller retains ownership of input bytes.
bool start(const PreviewConfig& config, Es3DisplayLink& link);
SubmitResult submitJpeg(const uint8_t* bytes, size_t size,
                       uint32_t sourceWidth, uint32_t sourceHeight,
                       uint64_t captureMonotonicUs);
void setVisible(bool visible);
PreviewStats stats() const;
void stop();

// Jetson: libes3display. One owner of the USB device, including controls.
bool start(const DisplayLinkConfig& config);
bool enqueueControl(const ControlRequest& request);
bool offerFrame(EncodedFrameLease&& frame);
DisplayLinkStats stats() const;
void stop();

// ESP32: es3Video. Only the USB task calls the record-ingestion methods.
bool begin(const VideoLimits& limits);
ReceiveResult acceptRecord(const VideoRecord& record);
void expireIncompleteFrames(uint32_t nowMs);
bool acquireLatestImage(ImageLease& image);  // application/graphics task
void releaseImage(ImageLease& image);
void resetSession(uint32_t sessionToken);
```

`SubmitResult` distinguishes accepted, replaced-waiting-frame, inactive,
rate-limited, oversized, and busy. A false `start()` disables preview without
preventing recording. `offerFrame()` transfers a lease only on success; a
failed offer leaves ownership with the caller. Every lease identifies one
fixed pool slot and is released exactly once. No API stores a caller's borrowed
camera bytes beyond `submitJpeg()`.

Protect callback submission with a try-lock or an equivalent bounded slot
handoff. If the worker owns the slot metadata, drop the submitted preview
frame rather than wait on that worker. Stop accepting work before shutdown,
cancel timed I/O, and join workers before freeing their pools. Destructors and
`stop()` must be safe after partial startup failure.

### Proposed configuration

Add an optional `<DisplayPreview>` section to the host configuration. These
defaults are provisional until the feasibility bench passes.

| Setting | Default / bound | Meaning |
| --- | --- | --- |
| Enabled | false | Opt in after the board is flashed and its USB path is configured |
| Device | `/dev/dashcam-display` | Exact display alias or by-id path; do not probe the C3 |
| TargetFps | 10; supported range 1–15 | Nominal ceiling; cannot exceed arriving source frames |
| Fit | letterbox | Preserve aspect ratio inside the negotiated display canvas |
| JpegQuality | 65 | Initial preview encoder setting |
| MinimumJpegQuality | 45 | Floor for one bounded retry after an oversized encode |
| MaxJpegBytes | 65536 | Hard cap; negotiate the smaller host/device limit |
| ChunkDataBytes | 1024 | Starting transport chunk size; limited by capabilities |
| AssemblyTimeoutMs | 500 | Deadline from START until END for one incoming frame |
| StaleAfterMs | 500 | Mark the last displayed frame stale |
| PlaceholderAfterMs | 2000 | Replace stale footage with the unavailable screen |
| HeartbeatMs | 1000 | Send even while video is paused; current session timeout is 5 s |
| ReconnectMs | 1000 | Retry the explicitly configured USB device |

Validate settings once at startup, report rejected values, and retain hard
compile-time firmware ceilings regardless of host configuration. Camera
exposure or source recovery may lower the arriving FPS; report source-limited
performance separately from a transport or display bottleneck.

### Capture integration

Create a leaky compressed preview branch upstream of any recording-only FPS
cap. Keep the recording branch and current exposure/luma behavior intact.
Set the preview callback before starting each recording session, and reattach
it during camera recovery. Pass source dimensions, format and a monotonic
capture timestamp. Support the Ugreen MJPEG path initially; reject other
codecs with a clear preview status while recording continues.

Copy retained compressed bytes into a bounded pool inside the callback,
without waiting for a free slot. Input buffers have an explicit size cap
(initially 8 MiB per slot; two slots). Retain only one waiting input frame in
addition to work already in progress. Do not retain unlimited GStreamer
buffer references or allocate on each callback.

Preview is available whenever the recorder owns and captures the Ugreen.
Preview-only capture while recording is stopped is a separate future mode;
show “Camera inactive” in that case. No preview failure should restart the
recording pipeline. Verify error propagation and branch teardown as well as
queue leakage—leaky queues alone do not isolate pipeline errors.

### USB framing and scheduling

Keep the existing `E3 28` control framing and commands. Add a capabilities
request advertising video support, codec, maximum dimensions/frame/chunk
sizes and receive credits. A host must negotiate before starting video; older
firmware remains usable for controls and reports video unsupported.

Add separately identifiable `E3 29` video records on the same byte stream,
with a version and 16-bit payload length. Start with 1024-byte chunks. The
parser must recognize both record types without treating JPEG data as control
messages. Define explicit START, CHUNK, END and ABORT messages carrying a
32-bit session token and frame ID. START carries image size, total bytes and
source timestamp; CHUNK carries an offset. Use record CRC16 and whole-frame
CRC32; reject invalid lengths, offsets, sessions or dimensions before copying.

Allow at most two credited compressed frames in flight. Replenish credits
when the corresponding receive buffer is reusable, not at every chunk.
Report frame acceptance and presentation separately. Drop incomplete/old
frames on timeout; do not retransmit stale video. Reset credits and session
identity on reconnect and discard all queued old-session data.

One host I/O worker owns the tty. Schedule control/heartbeat/audio commands
between video chunks with priority over more video bytes. The Python bench
client and production C++ publisher cannot independently open the same tty.
Replace the firmware's control-specific one-frame-per-10-ms servicing policy
with bounded, continuously serviced USB I/O and larger fixed RX buffers.

### Draft protocol details to freeze before implementation

Reserve control commands 07 for CAPABILITIES, 08 for START_VIDEO and 09 for
STOP_VIDEO; commands 01–06 retain their existing behavior. CAPABILITIES returns
supported video protocol/codec versions and hard limits. START_VIDEO selects
the negotiated parameters and returns a new nonzero session token. Capability
or negotiation failure leaves the control channel operational. An older peer's
unknown-command reply means “video unsupported”, not a reason to retry forever.

Proposed video record layout (all multibyte integers are little-endian):

```text
E3 29 | version:u8 | type:u8 | payload_length:u16 |
session_token:u32 | frame_id:u32 | payload | crc16:u16
```

The fixed overhead is 16 bytes. `payload_length` excludes header and CRC.
CRC-16/CCITT-FALSE covers version through the end of the payload. A CHUNK's
payload is a four-byte offset plus at most 1024 image bytes, giving a maximum
record size of 1044 bytes with the initial configuration. Encode fields
explicitly; do not send native C++ structs with implicit padding.

| Video record | Draft payload / behavior |
| --- | --- |
| START | width u16, height u16, JPEG byte count u32, source timestamp u64; consumes one credit |
| CHUNK | byte offset u32 followed by JPEG data; require the exact next offset |
| END | CRC32 of the complete JPEG; publish only after exact byte count and checksum match |
| ABORT | reason u8; discard the identified incomplete frame |
| RESULT | result/reason u8 for the frame ID; distinguishes accepted assembly from decode/drop failures |
| PRESENTED | emitted after the last display strip; identifies the displayed frame and carries decode/blit durations |
| CREDIT | cumulative granted-frame count u32, with frame ID zero |
| VISIBILITY | demand u8 (active, hidden, calibrating), with frame ID zero |

Assign video type values and reason codes in a shared portable header during
the protocol milestone, then freeze them with golden-vector tests. Specify
whole-JPEG CRC as CRC-32/ISO-HDLC: reflected polynomial EDB88320, initial
FFFFFFFF, final XOR FFFFFFFF. Include the `123456789` check value CBF43926 in
both host and firmware tests.

Frame IDs begin at one and increase within a session; renegotiate before
wrapping. Track credits as a cumulative grant, initially two. The sender's
number of frames started must never exceed that grant. Repeated CREDIT records
are harmless: use the latest grant rather than adding the advertised value.
Each released JPEG slot advances the grant exactly once. A START rejected
before slot allocation must explicitly reconcile its consumed grant or force
session resynchronization; it must not silently lose a credit and stall forever.

On a lost or ambiguous control exchange, restart negotiation. On an incomplete
video frame, discard/abort it and continue with newer footage. Do not retry
audio notifications during reconnect. A new HELLO, session timeout or USB loss
invalidates the video session and all its grants. Reject old-session records
even when their frame IDs match new-session frame IDs.

Use a multiplexing parser with bounded record lengths and assembly deadlines.
The bytes `E3 28` and `E3 29` may occur inside JPEG data and are ordinary bytes
inside a valid CHUNK. Never dispatch commands found by scanning within an
already accepted payload. Test recovery from malformed records independently
of the command handler and JPEG decoder.

### Frozen protocol v1 (2026-10-13)

The draft above is frozen as video protocol version 1. The reference is
`peripherals/es3c28p/lib/es3UsbLink/Es3VideoProtocol.h` (with `Es3Protocol.h`).
Both headers are byte-identical copies in `lib/libes3display/`, and
`tests/run_tests.sh` compares them with `cmp`. The Python mirror is in
`host/es3_display.py`. The vectors are in
`peripherals/es3c28p/tests/video_golden_vectors.txt`, checked by
`tests/video_protocol_test.cpp` (C++11 and C++17) and `tests/test_usb_client.py`.
Change these bytes only with a version bump.

- **Record:** `E3 29 | 01 | type | len u16 | token u32 | frame u32 | payload |
  crc16`. Fields are little-endian. The CRC is CRC-16/CCITT-FALSE over version
  through payload. The token is never zero. `frame` is 1, 2, ... for every type
  except CREDIT and VISIBILITY, which use 0.
- **Types:** START `01` {w u16, h u16, jpeg u32, ts_us u64}. CHUNK `02`
  {offset u32, 1..N data}. END `03` {crc32 u32}. ABORT `04` {reason u8}.
  RESULT `81` {reason u8}. PRESENTED `82` {decode_us u32, blit_us u32}.
  CREDIT `83` {cumulative grant u32}. VISIBILITY `84` {0 active, 1 hidden,
  2 calibrating}.
- **Reasons (u8):** 00 Ok, 01 BadLength, 02 BadOffset, 03 BadCrc,
  04 StaleSession, 05 NoCredit, 06 TooLarge, 07 BadDimensions,
  08 AssemblyTimeout, 09 DecodeFailed, 0A Superseded, 0B Hidden, 0C Busy,
  0D BadFrameId, 0E Malformed, 0F Cancelled, 10 Internal. A RESULT of Ok
  means "assembled, queued for decode".
- **Control:** these use E3 28 frames.
  - `07` CAPABILITIES: empty request. The 18-byte reply is {version u8,
    codec_mask u8 (bit0 = baseline JPEG), canvas w/h u16, max w/h u16,
    max_jpeg u32, max_chunk u16, max_credits u8, flags u8 = 0}.
  - `08` START_VIDEO: 23-byte request {version u8, codec u8 (0 = baseline JPEG),
    viewport x/y/w/h u16, max_jpeg u32, chunk u16, fps u8, assembly/stale/
    placeholder ms u16}. The reply is {token u32 ≠ 0, initial grant u32}. An
    out-of-limit request gets BadPayload.
  - `09` STOP_VIDEO: request {token u32}, empty reply. A stale token is a no-op.
  - If CAPABILITIES gets BadCommand or NotReady, video is unsupported: stay
    control-only.
- **Limits (wire ceilings):** CHUNK data 4096 (1024 initially), record 4116 B,
  JPEG 65,536 B, credits 2, fps 30 (config 1–15), timeouts 10 s, partial-record
  gap 250 ms. START must match the viewport.
- **Parser drops (no reply):** wrong version or type, length invalid for the
  type, zero token, wrong frame-id rule, or a bad CRC. Fields that are wrong in
  a well-framed record get a RESULT with the matching reason.
- **One RESULT per START:** a credited START that is rejected still advances
  the grant. A NoCredit START does not. ABORT is answered by RESULT with the
  same reason.

### ESP32 ownership and memory

The USB service task owns Serial and framing, and queues validated commands
for the application loop. It must not call graphics, SD or audio methods from
that task. A decode worker owns the JPEG decoder. The existing application
loop remains the only owner of TFT, touch, SD and audio operations.

Allocate two 64 KiB JPEG slots and two maximum 320×240 RGB565 buffers in
PSRAM during startup: 438,272 bytes in total, plus decoder state and small
internal-RAM strip buffers. Reject video startup cleanly if allocation or
PSRAM is unavailable. Use explicit buffer states and ownership handoffs so
receive, decode and draw never modify the same buffer concurrently.

Blit in bounded strips (initially 8–16 rows), yielding to touch/audio/control
work between strips. Stage pixels into internal RAM as required by the chosen
SPI path; do not assume PSRAM buffers are directly DMA-compatible. Hold the
image being drawn until the whole image is finished, then select the newest
ready image. Avoid full-screen clears on every frame. Confirm RGB565 byte
order using primary-color and gradient test images.

### State transitions and application lifecycle

JPEG slots move through `FREE → RECEIVING → READY → DECODING → FREE`.
Incomplete, rejected or obsolete slots return to FREE through one cleanup path.
Decoded image slots move through `FREE → DECODING → READY → DRAWING → FREE`.
Only READY images may be replaced by newer images; never overwrite DRAWING or
DECODING slots. A decoder takes a free image slot before consuming a JPEG slot,
or drops stale waiting input if none is available. Every ownership transition
must synchronize buffer contents as well as the state flag.

Host link states are `DISCONNECTED → HELLO → CAPABILITIES → NEGOTIATED`, then
ACTIVE or PAUSED according to display demand. USB loss returns to DISCONNECTED;
unsupported video retains a control-only state. Hidden tabs and calibration
stop new preview conversion and transmission, not just TFT writes. Publish a
fresh visibility state when entering a new session so an old hidden-state
notification cannot suppress video permanently.

Integration order in `dashcam_v0_4.cpp`:

1. Read preview settings and start the optional USB owner and converter.
2. Register the preview callback before each `SegmentedRecorder::start()`.
3. On camera recovery, invalidate pending source frames and metadata; update
   source format/dimensions before allowing a new frame through.
4. On USB loss, stop conversion demand and release queued video; recording
   continues while the dedicated host link worker retries the display.
5. On application shutdown, disable submission, stop the capture callback
   producer, cancel preview I/O, join workers, and release buffers only after
   all callbacks and workers have finished. Preserve recording finalization.

The 320×180 landscape render occupies x=0..319, y=30..209. Clear its black bars
when entering video mode or changing geometry, not on each frame. End one SPI
transaction per strip so input and audio servicing can run between strips.
The presentation ACK means software completed the last strip transfer; it
does not claim the LCD panel has completed its physical refresh. Atomic buffer
ownership prevents mixed-frame software writes, but visible scan tearing still
requires a bench check on this panel.

Update both Arduino CLI scripts and `platformio.ini` for the pinned decoder
and new local libraries. Add the host libraries and JPEG linkage to the
Makefile and record new container dependencies if the implementation needs
any. Keep Python support for protocol diagnostics; do not require a Python
process in the production capture/convert/send path.

## Delivery sequence and acceptance

1. **Feasibility bench:** measure USB sustained payload throughput and control
   latency, 320×180/320×240 SPI blit time, JPEG decode time, and Jetson conversion
   cost using representative day/night frames. Establish budgets before fixing
   FPS/quality. Test transport without the LCD and LCD without transport first.
2. **Protocol and host link:** implement capability negotiation, video framing,
   bounded credits, prioritization and reconnect. Add C++/Python golden vectors,
   split/coalesced-packet, bad-length/CRC, timeout and stale-session tests.
3. **Firmware renderer:** receive and display generated patterns and JPEGs;
   test malformed/oversized JPEGs, memory exhaustion, byte order, tab changes,
   calibration, and alert playback during streaming.
4. **Jetson converter:** implement rate-limited decode/resize/encode and the
   bounded newest-frame policy. Test aspect ratios, buffer exhaustion,
   shutdown/cancellation and slow receivers using recorded JPEG fixtures.
5. **Recorder integration:** add the SegmentedRecorder tap and wire the optional
   publisher into v0.4 startup/recovery/shutdown. Preserve capture settings,
   original recording bytes, luma processing and segment finalization.
6. **Combined soak:** run for at least 30 minutes at 10 fps, then 15 fps. Check
   stable memory, recording frame/segment health relative to a preview-off
   baseline, no preview-induced camera restarts, and healthy touch/audio.
   Test repeated USB unplug/replug, camera recovery, host exit, absent SD,
   scene-noise bursts and navigation away from LIVE.

Expose sent/received/decoded/presented FPS, byte rate, frame size, dropped-frame
reasons, CRC/decode failures, reconnects, and encode/decode/blit durations.
Measure source-callback-to-presentation-ACK latency on the host clock; report
it as an upper bound that includes the return ACK, without subtracting
unsynchronized Jetson and ESP32 timestamps. Aim for p95 under 250 ms and
control/touch response under 100 ms during video. Hardware results determine
whether 15 fps is offered or the default remains 10 fps.

## Implementation checklist

- [ ] Measure USB, decode, conversion and SPI throughput separately; save results.
- [x] Jetson conversion cost measured on static indoor frames (Review additions, item 1); drive/night frames still to do.
- [ ] Run the JPEG-from-SD and USB-sink bench sketches; measure hwcdc against TinyUSB CDC.
- [ ] Pin JPEGDEC in the Dockerfile, `platformio.ini` and both build scripts.
- [x] Freeze capabilities, video record types, errors, credit accounting and golden vectors (Frozen protocol v1).
- [ ] Implement bounded host transport with control priority and explicit device selection.
- [ ] Implement firmware assembly, session reset and credit return on every error path.
- [ ] Integrate the pinned decoder, fixed PSRAM pools and image-ownership handoffs.
- [ ] Add strip rendering, LIVE demand, calibration takeover and stale indicators.
- [ ] Add the host converter and newest-frame policy with bounded encode retry.
- [ ] Add the SegmentedRecorder tap upstream of recording FPS limiting.
- [ ] Wire configuration, camera recovery, USB reconnect and ordered shutdown.
- [ ] Pass parser/codec/queue tests, including malicious dimensions and embedded framing bytes.
- [ ] Verify recording/sidecar/segment behavior with preview disabled and enabled.
- [ ] Complete 10 fps and 15 fps hardware soak runs with touch and audio active.
- [ ] Document measured limits, selected quality/subsampling and the supported default FPS.

The measured Jetson conversion and the protocol freeze are complete. Existing USB
control tests and firmware compilation do not establish video performance.
