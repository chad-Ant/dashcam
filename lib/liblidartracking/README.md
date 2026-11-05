# GRF-250 + IMX296 vehicle ranging prototype

Separate from `libstereoprototype` and `libstereocam`; neither is replaced or
modified. This library estimates a selected vehicle's **LiDAR slant range** and
camera-frame forward distance, associates returns with camera tracks, and emits
pan requests for a **separate single-servo/encoder library**. It never drives a
motor, CAN bus, throttle or brake. Not a collision-avoidance / AEB system.

## Delivered and intentionally not deployed

- Orin Python library: rectified camera geometry, detector adapter, bounded IoU
  tracks, explicit target selection, conservative beam/box association, temporal
  gates, alpha-beta range/closing-speed filter, and encoder-feedback contract.
- Portable allocation-free C++ `peripherals/mkr_zero/libraries/GRF250`: bounded
  CRC parser and nonblocking configuration/recovery state machine.
- Complete `peripherals/mkr_zero/helper_scripts/GRF250Bridge` bench sketch:
  dedicated UART, timestamped/CRC-protected USB samples and clock-sync replies.
- Headless synthetic demo, JSONL replay and host tests. No model downloaded,
  service installed, firmware flashed, or existing camera/USB stream opened.

**Not yet production-integrated.** The bench sketch replaces CAN/GPS/IMU
telemetry if flashed. Do not flash it to the active rig without a bench session.
Production MKR USB already carries raw CAN; do not open a second reader there.
Merge LiDAR records into the existing single-owner USB multiplexer/recorder in a
later integration, after hardware acceptance. The existing C3 wire protocol is
unchanged. Python objects are single-thread-owned; hand off through bounded
queues if capture and inference run on different threads.

## Hardware: your 3.3 V full-duplex module

The supplied photo shows separate `A/R+`, `B/R-`, `Y/T+`, `Z/T-` terminals, which
is the correct *topology* for a full-duplex link. It does not identify the fitted
IC or establish isolation/level shifting. Use the ordered **3.3 V variant** only
after verifying its chip marking, supply specification and UART output voltage.
A genuine Analog Devices MAX490 is a **5 V** part, not a 3.3 V variant. A module
advertised under that name may fit another transceiver. Never power a genuine
MAX490 below spec or connect its 5 V receiver output directly to MKR inputs.
Use proper 3.3 V transceivers or a verified level-shifting buffer if necessary.
See [MAX490 manufacturer specifications](https://www.analog.com/en/products/max490.html).

GRF-250 exposes **TTL UART**, not native RS-422: use **two** modules, one near
each endpoint. The electrical conversion does not change its UART protocol.

```text
MKR extra UART — 3.3 V module == two RS-422 twisted pairs == module — GRF UART
       │                                                       │
       └──── native USB → Orin                                  └─ regulated 5 V
IMX296 ─── CSI → existing frame producer → Orin association library
                                 separate encoder-servo library ↔ pan interface
```

| Connection | Endpoint |
|---|---|
| MKR A3 / D18 / PA04 | UART TX → module **driver input (DI)** |
| MKR A6 / D21 / PA07 | UART RX ← module **receiver output (RO), 3.3 V maximum** |
| Module at MKR `Y/T+`, `Z/T-` | Module at GRF `A/R+`, `B/R-`, respectively |
| Module at GRF `Y/T+`, `Z/T-` | Module at MKR `A/R+`, `B/R-`, respectively |
| GRF UART TX | Remote module DI |
| GRF UART RX | Remote module RO, verified compatible logic voltage |

Do **not** infer DI/RO from TXD/RXD silk alone: vendors label from different
perspectives, and this photo's arrows warrant checking the board schematic or
continuity to the IC pins. Confirm terminal polarity too. Use a reference-ground
conductor for nonisolated modules, with ground-offset/common-mode limits checked;
prefer an isolated interface for differing vehicle/Orin power grounds. Terminate
each twisted pair at its receiver with the cable's characteristic impedance
(typically 120 ohms), accounting for resistors already fitted. Do not add four
terminators blindly. The photo does not prove galvanic isolation.

MKR UART allocation: SERCOM0, ALT mux, TX pad0 / RX pad3; confirmed against the
pinned MKR Zero core variant. Serial1 remains reserved for C3. CAN, I2C, IMU and
A4/A5 servo-reserved pins are not reused. These connections are NOT on Serial1.

Power the GRF from a protected, regulated 5 V supply (not MKR's 3.3 V pin).
Provide a local 3.3 V regulator for the exterior 3.3 V transceiver if needed.
Servo motor power must be separate from logic/sensor rails, with appropriately
sized regulator, fuse, decoupling and grounding. Protect vehicle power against
reverse polarity and transients. Use sealed strain-relieved connectors and an
enclosure: GRF's full unit is IP00, despite its front face being IP67. Windows
need manufacturer-compatible 905 nm optics and reflection control; don't put
arbitrary clear plastic in front of the laser. Do not aim at people or view
through magnifying optics; follow Class 1M precautions.

Electrical/protocol source: [user-supplied GRF guide Rev 3, pp. 6–8, 20, 25–35](https://www.mouser.com/datasheet/3/4829/1/GRF-250-Product-Guide-Rev3.pdf).
Cross-checked against [manufacturer guide Rev 5.2](https://lightwarelidar.com/wp-content/uploads/2025/12/GRF-250-Product-Guide-v5.2.pdf).

## Sensor and wire contracts

115200 baud, 8N1, no flow control, full duplex. Protocol: AA, little-endian
flags (payload length in bits 6–15, write flag bit0), command/data, CRC16-XMODEM
(initial zero, polynomial 0x1021), CRC low byte first. Supported payload <=33
bytes, frame gap timeout 20 ms. Reserved flag bits are ignored.

Startup checks product name and reads firmware; it stops streaming, writes and
reads back output mask `0x2D` (first raw cm, first strength dB, last raw cm, last
strength dB), lost-signal count 1, rate 20 Hz, then stream mode 5 / command 44.
First UART command can be consumed by interface selection: bounded retry handles
this. No flash Save, reset, upgrade or laser-enable command is sent. If a unit
boots with laser disabled, inspect its configuration explicitly; this driver
does not silently turn it on. Defaults / changed firmware require bench checks.
Five-return ID45 is not implemented: both cited guides list a 44-byte result
but enumerate 40 bytes of fields. Do not guess a parser for it.

`-10` lost-return sentinel is preserved; the host never substitutes the previous
range. At 500 ms without the expected stream, the FSM fails closed and restarts
configuration after a 2 s cooldown, with five attempts per command. UART service
must be frequent: the bench sketch discards buffered data and reconnects after
a main-loop pause over 10 ms. This is not a hardware arrival timestamp or an
unattended production firmware watchdog implementation.

USB protocol (ASCII, LF, uppercase hex, maximum 62 bytes per measurement):

```text
L1 sequence micros first_cm first_db last_cm last_db crc16
Q1 nonce                          # host request: exactly 8 hex digits
T1 nonce micros crc16              # MKR reply
S1 micros ready firmware faults usb_drops parse_errors crc16  # 1 Hz health
```

Every field except prefix/CRC is exactly eight hex digits; signed values are
two's-complement int32. CRC covers ASCII before the final space. Sequence counts
all parsed samples, including USB drops; no queue preserves stale ranges. Writes
are <=63 bytes and only when the CDC IN bank is idle (the stock core's
`availableForWrite()` alone is not sufficient). `SerialBridge.status` exposes
the latest S1 health record with host reception time; check its freshness. The
firmware field is the raw ID2 word (patch byte0, minor byte1, major byte2).
USB drops count refused measurement/clock/health writes. These are bench
diagnostics, not yet integrated into production telemetry.

Host clock sync uses nonce-matched round trips, uint32 micros wrap handling,
expiry after 5 s and a conservative drift allowance. T1 is timestamped when it
is read, so once per second `poll()` blocks up to `sync_wait_s` (default 20 ms,
max 50 ms) for the reply; otherwise the round trip would be at least the
caller's poll period (a 20 Hz loop would never sync). A reply arriving after
the wait is still matched by a later `poll()`, with its honest, larger error.
Clock synchronization is
**not sensor acquisition synchronization**: ID44 has no acquisition timestamp.
`LatencyBounds` must be measured for the actual rate/firmware/transport before
setting it. Without those bounds, `timing_verified=False` and no vehicle range
is accepted. On reconnect/reboot, recreate the decoder, ranger and tracker and
explicitly reselect a target; don't carry IDs or filter state across boots.
`BridgeDecoder.generation` increments on a backward sequence jump; treat a
change as a reset notification, not as ordinary missing samples. A disconnect
also requires a reset even if the next sequence happens to look continuous.

## Camera / encoder / association API

No independent CSI owner is introduced. Feed existing IMX296 BGR frames to
`CameraAdapter.process(image, exposure_midpoint_s, uncertainty_s)`. It rectifies,
letterboxes and runs a caller-supplied COCO YOLOv8 640x640 ONNX model (no embedded
NMS, output `[1,84,8400]`), then assigns bounded IoU track IDs. Its OpenCV CPU
backend is for prototyping, not a performance claim. OpenCV model compatibility
depends on the export; test your actual model in the running container. A
TensorRT detector/tracker can instead construct `CameraFrame` directly. See
[OpenCV's YOLO model contracts](https://docs.opencv.org/4.x/da/d9d/tutorial_dnn_yolo.html).

Boxes must be from the **rectified**, calibrated image resolution. Do not mix
resized detector coordinates with raw camera intrinsics. The included JSON
calibration is a **placeholder**, `verified:false`; it cannot produce valid
ranges until intrinsics, mount geometry and uncertainty are measured.

```python
from lidartracking import FrameHistory, VehicleRanger
from lidartracking.camera import CameraAdapter

camera = CameraAdapter(calibration, original_K, distortion, model_path)
ranger = VehicleRanger(calibration)
history = FrameHistory()                     # bounded; clear it on any reset
ranger.select(selected_track_id)             # explicit; never auto-switches
frame, rectified = camera.process(bgr, exposure_s, exposure_uncertainty_s)
history.add(frame)
request = ranger.aim(frame, now_s)
if request is not None:
    accepted = servo.request_pan(request)     # future servo library
    if not accepted:
        servo.stop()
feedback = servo.feedback()                  # measured encoder interval
pair = history.bracket(sample.time_s)        # None until the frame after it exists
if pair is not None:                         # keep the sample until then
    result = ranger.update(pair[0], sample, feedback, now_s, next_frame=pair[1])
# Display only if result.valid AND current_monotonic_s <= result.expires_s.
# On missing target, stale frames, serial loss or expiry: clear distance and stop
# requesting motion; the servo driver's own command-expiry watchdog must hold.
```

`ServoPort` has `request_pan(ServoCommand) -> bool`, `feedback()`, and `stop()`.
Angles are radians, positive toward camera image-right for the identity mount;
zero is the calibrated mount reference, **not necessarily servo PWM neutral**.
The future driver owns encoder zero/sign, command-to-angle mapping, limits,
homing, backlash, watchdog and motor power. Encoder must measure output angle
or include gearing/backlash in `error_rad`. Command expiry is mandatory.

`ServoFeedback` includes angle, worst-case angular error, sample time, settled
flag and **observed** stable interval `[settled_since_s, valid_until_s]`. The
interval must cover the whole LiDAR acquisition uncertainty interval. Future
promises or the commanded angle are not feedback. Reconstruct this interval
from encoder history; a single position sample is insufficient. One horizontal
servo cannot correct vertical misalignment, pitch/bumps or targets outside the
laser height; these produce invalid range, not fabricated distance.

By default, association requires the entire uncertainty-expanded beam footprint to fit
inside the central 60% of the selected box and not intersect another box
(`association="first_return"`, below, relaxes this for range). It
includes mount translation, rotating lever arm, optical divergence, encoder
error, range uncertainty and time-skew apparent-motion margin. Verify the
configured `motion_bound_px_s`; it is not estimated or guaranteed by IoU.

**Pass the two frames around the sample (`next_frame=`).** LiDAR (20 Hz) and
camera are unsynchronized. With one frame, the margin is speed bound × skew
(`motion_bound_px_s`, default 2000 px/s), so a 10 ms gap costs 20 px. That
limits a real 1.8 × 1.5 m car to roughly 10–16 m. With the frames either side,
boxes (target and others) are interpolated to the LiDAR time per axis. The
error then scales with `motion_accel_bound_px_s2` (default 10000 px/s², an
unverified estimate: 1° pitch at 2 Hz is about 3200 px/s² at fx=1200) times the
frame interval squared. Measured with `fx=1200`, 0.001 rad boresight, centred car:

| Camera | One frame | Frame pair |
|---|---|---|
| 30 fps | 11 m | 44 m |
| 60 fps | 16.5 m | 50.5 m |

At 0.005 rad boresight the pair reaches 31.5–34.5 m; at 40000 px/s², 27.5 m (30 fps).
The remaining ~50 m ceiling is the beam (0.5° plus pointing error) outgrowing
the central 60% of the car at this focal length: a lens/calibration limit, not
a timing one. A low car (1.3 m tall) stops at 38 m (30 fps).

**Longer reach: `VehicleRanger(..., association="first_return")`.** Opt-in.
The beam may spill sideways or above the car but never below the box's bottom
edge (ground contact, minus 10% of box height for a loose box). The beam
*centre*, with pointing error, must lie inside the box's central 60% width and
below its top 10%. Range is the first return. Other detected vehicles in the
spill are still rejected as ambiguous. A farther last return (background seen
through the spill) is allowed, but a last return nearer than the first, or
invalid, is rejected.

Why this is safe on flat road: a ray that, at the car's range, is above the
ground-contact row was above the road the whole way there. So a nearer
surface can enter the beam only below the box. Everything the spill can reach
(farther road, background, sky) returns *later*. This assumes the box bottom
is the vehicle's real ground contact, locally flat road, and the LiDAR above
the road. Undetected nearer obstacles (poles, pedestrians, spray) defeat both
modes alike. **The first-return bias on a partly covered target is
unmeasured.** The guide only says returns closer than ~20 m apart merge.
Bench-test it before relying on this mode.

Best-case reach for a 1.8 m wide car, measured with the code at 30 fps with a
frame pair and 3 ms total timing uncertainty. Each row has the beam at the best
height on the car, i.e. the mount pitched for that range. "Measured-ish"
assumes the rig proves a 1° pitch at 2 Hz bound (3200 px/s² at fx=1200,
scaled with focal length):

| fx | Pointing (boresight + encoder) | 1.3 m car | 2.0 m car |
|---|---|---|---|
| 1200 | 2 mrad | 64 m (78 measured-ish) | 94 m (120) |
| 1200 | 1 mrad | 74 m (92) | 114 m (142) |
| 2400 | 1 mrad | 83 m (108) | 128 m (166) |
| 3600 | 1 mrad | 89 m (117) | 137 m (180) |

At 60 fps, fx=2400 and 1 mrad, a 1.3 m car reaches 106 m with the default
acceleration bound (120 m measured-ish). Contained mode stops at 38–59 m for
the same cars. **100 m on a 1.3 m car therefore needs about 2× the current
focal length, 1 mrad total pointing error, and either 60 fps or a measured
acceleration bound.** It also needs the detector to find a car that is about
31 px tall at 100 m.

The servo only pans, so the beam's height on the car is fixed by the mount and
changes with range. A level beam at windscreen height (~1.2–1.3 m) runs along
the roof line of a 1.3 m car. Mount the LiDAR lower, or pitch it down, so the
beam crosses the lower body over the range band you need. Waiting for the next frame adds up to one frame interval of
latency; `max_skew_s` (0.1 s) must still cover it plus inference. Frame-time
errors are treated as common-mode (one camera clock). Detector jitter is not
motion and is left to the central-box shrink; interpolation never amplifies it.
Rejects stale/out-of-order/future data, unknown calibration/timing, lost target,
missing encoder, low strength, sentinel/out-of-range returns and differing
first/last returns (in first-return mode: only a nearer or invalid last return). Defaults: 10–120 m, matching conservative Rev3 minimum.

Bounding boxes can contain background/windows; equal first/last returns do not
prove a car hit. The prototype is a hypothesis generator, not certified object
association. IoU tracking cannot guarantee identity through crossings. Closing
speed is a smoothed relative radial derivative, not an ego-speed estimate.
Forward distance is camera-frame Z, not bumper clearance. No ±1 m guarantee at
100 m is made: the vendor accuracy is a 1-sigma specification, and moving car
reflectivity, beam placement, latency and weather need independent validation.
At 100 m a 0.5-degree full-angle beam spans about 0.87 m. With a wide-angle
IMX296 lens, a distant car may occupy too few pixels for the conservative
central-box/uncertainty gates to accept it. A 120 m software range limit is not
a claim of that tracking distance. Synthetic test boxes are deliberately large
and do not validate distant-car visibility; check the real lens and targets.

## Run and verify (from repository root, no hardware mutation)

```bash
make -C lib/liblidartracking test
make -C lib/liblidartracking sanitize
make -C lib/liblidartracking demo

docker run --rm --runtime=runc --network=none \
  -e PYTHONDONTWRITEBYTECODE=1 -e OPENBLAS_NUM_THREADS=1 \
  -v "$PWD:/work:ro" -w /work/lib/liblidartracking \
  l4t-ml-gpio:latest make test

docker run --rm --runtime=runc --network=none -v "$PWD:/work:ro" \
  l4t-ml-gpio:latest arduino-cli compile --warnings all \
  --fqbn arduino:samd:mkrzero --build-path /tmp/grf250-build \
  --library /work/peripherals/mkr_zero/libraries/GRF250 \
  /work/peripherals/mkr_zero/helper_scripts/GRF250Bridge
```

Headless replay: `cd lib/liblidartracking` then
`python3 -m lidartracking --calibration config.json --replay events.jsonl`.
Each event contains `selected_id`, `camera` (CameraFrame fields with a detection
list), `range` (RangeSample fields), `servo` (ServoFeedback or null), `now_s`.
All timestamps must share Orin monotonic seconds. Malformed records fail the
replay, not get silently converted to a good range. Synthetic mode uses invented
calibration, timing and encoder feedback; it demonstrates software only.

For dedicated bench firmware only, `SerialBridge('/dev/serial/by-id/...',
bench_firmware=True, latency=None)` and frequent `poll()` produce raw samples.
Close in `finally`. Select the MKR by USB identity, not `/dev/ttyACM0` guesses.
Stop existing consumers first: advisory/exclusive locks cannot evict an already
open reader. This library never auto-flashes or stops a running service.

## Acceptance sequence / next integration

1. Confirm actual module chip and logic levels; check both endpoints/polarities
   unpowered, then run a differential loopback with MKR disconnected. Verify
   termination and power during servo stall/load before connecting sensor I/O.
2. Bench-connect GRF, check reported firmware and a measured target at 10, 20,
   50 and 100 m. Verify cm units, lost-signal sentinel, first/last behavior,
   reconnect recovery, strength and USB loss. Capture raw bytes as fixtures.
3. Calibrate IMX296 intrinsics at its actual focus/resolution. Fit laser mount
   rotation, pivot, encoder zero/sign and lever arm from measured targets over
   several distances and pan positions. Validate on held-out targets; determine
   angle-error bounds including servo backlash. Don't merely flip `verified`.
4. Measure sensor/camera exposure timing and transport-delay envelopes. Record
   encoder history; check that stable intervals cover the measurement windows.
5. Test a stationary parked car, then controlled motion in a closed private area
   with an independent distance reference. Test occlusion, adjacent cars,
   reflective plates, target loss, glare, vibration and pitch. Record rejection
   rate as well as error; invalid output is not a successful range measurement.
6. Integrate into the existing MKR scheduler and single USB mux with measured
   CAN/IMU timing budgets, transport health/epoch records and watchdog support.
   Feed frames from existing CSI capture and replace CPU inference with a
   benchmarked TensorRT adapter if necessary. The Orin Nano has no NVENC/DLA;
   do not spend the recorder's CPU budget without profiling.
