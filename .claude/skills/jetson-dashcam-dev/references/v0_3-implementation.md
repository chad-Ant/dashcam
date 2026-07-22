# v0.3 implementation — the actual codebase

What this repo *actually builds today*, so a session doesn't reinvent (or contradict)
the existing architecture. v0.3 = compressed UVC recording + IMX296 lane detection +
driver drowsiness/fatigue monitoring, headless, in the `l4t-ml-gpio` container. The
entry point is `src/dashcam_v0_3.cpp`; everything else is a `lib/lib*` static component
linked per the `Makefile`.

Read this before editing v0.3 code, adding a camera role, touching a detector, or
answering "how does the dashcam do X". It reflects the code as of the `camera` branch;
verify against the headers (`lib/*/*.h`) if a detail matters — they are the source of truth.

## Component map (what is wired into v0.3)

v0.3 links exactly six libraries. Each is a namespace under `dashcam::` and takes an
injected `dashcam::log::LogCallback` (never writes to stdout itself).

| Library | Namespace | Role in v0.3 |
|---|---|---|
| `liblog` | `dashcam::log` | Async spdlog file+console logger. `init()` once, `getCallback()` everywhere, `shutdown()` last. |
| `libconfig` | `dashcam::config` | Self-describing XML config (`dashcam.xml`) via `ConfigVar<T>`. `AppConfig` aggregates every tunable. |
| `libcamera` | `dashcam::camera` | Discovery (`getCameraList`) + `Camera_CSI` (Argus) / `Camera_USB` (V4L2) over a shared `Camera_GST` tee/branch base. |
| `librecord` | `dashcam::record` | UVC **compressed passthrough** recorder → MKV + `.ass` telemetry sidecar. No encode. |
| `liblanedetector` | `dashcam::lane` | UFLD v2 lane detection (TensorRT) as a leaky NVMM branch on the IMX296. |
| `libdriverstate` | `dashcam::driver` | YuNet face-crop + ResNet18 drowsiness (TensorRT) + `FatigueScorer`, as a leaky branch on a driver-facing UVC cam. |

**Present in the repo but NOT in v0.3** (planned / production `dashcam` target): `libsigndetector`
(sign reading — `<Detection>` has `signTargetHz` etc., but v0.3 instantiates no sign detector),
`libstereocam` (VPI stereo rangefinder — needs `libnvvpi-dev`, only in the `dashcam` target),
and the peripheral libs `libcan`/`libgpio`/`libi2c`/`libmidi`/`libspi`/`libuart` (GPS/CAN/alarm
wiring for later versions). Don't assume these run in v0.3.

## Camera abstraction (`libcamera`)

- `getCameraList(std::vector<cameraInfo>&, log)` scans `/dev/videoN`, classifies each as
  `CAMERA_TYPE::CSI` (driver `tegra-video`/`vi`) or `USB` (`uvcvideo`), and enumerates every
  discrete `(pixelFormat, w, h, fps)` into `cameraInfo::videoFormats`. CSI cameras get their
  Argus `sensor-id` in `deviceId`. **Sensor identity** (e.g. "which node is the IMX296") is read
  from `/sys/class/video4linux/<node>/name` — see `sensorNameContains()` in the app.
- `Camera_GST` is a template-method base: subclasses supply `buildPipelineString()`,
  `cameraTypeTag()`, `pipelineError()`. It builds `source ! caps ! tee name=srctee`, with a
  default BGR `appsink` (`captureFrame()` source) plus any number of **branches**.
- **Branches** (`addBranch(name, bin, leaky, initialEnabled)`) each get a `queue` + `valve`:
  - `leaky=true` → 2-buffer leaky queue: drops old frames under backpressure, never blocks the
    tee. **Inference branches (lane, driver) use this.**
  - `leaky=false` → blocking queue: for recording branches where frame loss is unacceptable.
  - `setBranchEnabled(name, bool)` toggles the valve at runtime without rebuilding.
- `setCaptureEnabled(false)` idles the BGR capture converters when the camera is consumed only
  through branches — v0.3 calls this on both the lane and driver cameras (branch-only consumers).
- `setOutputResolution(w,h,fps)` inserts a VIC (`nvvidconv`) downscale + `videorate` **before the
  tee** — honoured by `Camera_CSI` only (cheap, keeps NVMM); `Camera_USB` ignores it (a v4l2
  source can't rescale — downscale per-branch instead).
- Lifecycle: `open() → setCameraVideoFormat(idx) → start() → … → stop() → close()`. A
  `stop()`/`start()` cycle **clears all branches** — re-`createBin()` and re-`addBranch()` before
  each restart. There's no GLib main loop; `checkBusErrors()` polls the bus from
  `getCameraStatus()` so a recording-only (no `captureFrame()`) camera still surfaces pipeline death.

## Recording (`librecord`) — no encoder, by design

`dashcam::record::Recorder` records the camera's **already-compressed** MJPEG/H.264 stream
straight into Matroska. On Orin Nano (no NVENC) this is the whole point: recording costs container
muxing only, not software x264.

```
v4l2src device=/dev/videoN ! image/jpeg,w,h,fps  (or video/x-h264 ! h264parse)
  [! videorate max-rate=N drop-only=true]   # MJPEG only; dropping intra JPEG frames is safe
  ! queue ! matroskamux ! filesink
```

- Owns its **own** pipeline — it does **not** attach to a `Camera_GST` tee. A V4L2 device is
  exclusive, so **the record camera and the driver-monitor camera can never be the same device**
  (this constraint drives the whole configuration setter below).
- `startRecording()` **rejects any non-compressed format** (raw would need the encoder this design
  removes). CSI/Argus sources are unsupported (raw only).
- Telemetry is an **`.ass` sidecar** next to the MKV (`clip.mkv → clip.ass`), not burned in:
  four-corner layout (speed/accel, heading, lat/lon/alt, live wall clock), styled by
  `OverlayConfig`. A subtitle+bus thread samples `OverlayData` at `SubtitleRateHz` and watches for
  bus errors. `setOverlayData()` is thread-safe from any thread; the clock stays live even when
  motion/GPS telemetry is stale (renders `-`). No GPS source is wired in v0.3, so only the clock moves.

## Lane detection (`liblanedetector`)

- Ultra-Fast-Lane-Detection **v2**, CULane ResNet18, engine `models/culane_res18_fp16.engine`,
  model input **1600×320**, four heads (`loc_row`/`loc_col`/`exist_row`/`exist_col`) validated
  against config at load.
- Attaches as a **leaky branch** on the IMX296 (`Camera_CSI`), `sourceIsNVMM=true`. Internal
  inference thread: appsink → `cudaMemcpyAsync` → CUDA preprocess kernel (crop+resize+normalise) →
  TRT `enqueueV3` → UFLD v2 decode → `latestResult_`. `poll()` is thread-safe; nothing else is.
- **Crop geometry** (default `<Detection>`): `inputCropTop=0.5` + `inputCropBottom=0.2059` on the
  1088-row IMX296 keeps rows [544,864) — a 320-row band that matches the model height, so the
  preprocess does **no vertical resample**. `nvvidconv` does the crop+convert in one VIC pass.
- `LaneResult`: `numLanes`, `currentLaneIndex` (0=leftmost, −1=off-road sentinel),
  `lateralOffset` (0 centred, ±1 on a boundary), `lateralValid`, `laneAllowedDirections`. The app
  bridges `lateralOffset`/`lateralValid` into the fatigue scorer via `DriverStateDetector::setLaneOffset()`
  each loop — the two libraries are deliberately decoupled (no compile-time dependency); the app is
  the only place both exist.

## Driver monitoring (`libdriverstate`)

Two independent alert layers on a driver-facing UVC camera at **2 Hz**:

1. **YuNet face detect (OpenCV `FaceDetectorYN`, CPU)** crops to the largest face
   (`face_detection_yunet_2023mar.onnx`, `faceDetectScale=0.5`, `faceHoldSec=3` grace for a
   head-droop), then **ResNet18 drowsiness** (`models/drowsiness_resnet18_fp16.engine`, 224×224,
   ImageNet norm, single logit → sigmoid). **`positiveIsDrowsy=false`** — `sigmoid(logit)=P(natural)`,
   so the lib reports `1−sigmoid` as `P(drowsy)`; this was set empirically and **contradicts the
   model card**, so re-verify with a bench check if the engine is ever retrained. `DriverStateResult`:
   `state` (NATURAL/DROWSY), `drowsyProbability`, `faceDetected`, `valid`, plus the fatigue fields.
2. **`FatigueScorer`** — a pure, deterministic, timestamp-driven state machine (100 fresh → ≤0
   fatigued). `−drowsyChunkPenalty` per completed drowsy chunk, `+awakeChunkReward` per awake chunk
   (asymmetric — fatigue builds faster than it heals), `−laneDriftPenalty` per drowsiness-correlated
   lane drift (the "drift-and-jerk-back" signature via the bridged `lateralOffset`), and a score
   **cap** that decays `capDecayPerHour` with time-on-task (floored at `capDecayFloor`). Tiers:
   `OK/CAUTION/WARNING/FATIGUE`; `FATIGUE` requires the score to hold ≤`fatigueScore` for
   `fatigueSustainSec` (default 5 min) — the high-confidence "pull over" alarm. Bad config is
   **clamped, never rejected** (a typo must not disable monitoring). `noFaceFreezes=true` freezes
   the score when the camera can't see a face.

The app keeps the **acute** micro-sleep alert (sustained instantaneous DROWSY ≥ `AcuteAlertSec`)
separate from — and never replaced by — the long-horizon score. `SIGUSR1` → `resetScore()` ("I took
a break"; a GPIO button later).

## The camera configuration setter (init phase)

`resolveCameraConfiguration()` in `src/dashcam_v0_3.cpp` is the init-phase step that inspects the
discovered cameras and assigns each a **role**, returning a `CameraConfiguration` and logging the
recognised **profile**. This is where the "what runs where" decision lives — extend it, don't
scatter role logic back into `main()`.

**Assignment order is load-bearing** (recorder and driver-monitor can't share a device):
1. **Lane** ← first CSI whose sysfs name contains `imx296` (native 1456×1088).
2. **Driver** (reserved *before* recording): a pinned `<Camera name="cabin" type="USB">` with a
   non-empty `<Device>` claims that node; `<Enabled>false</Enabled>` disables monitoring.
3. **Record** ← first precompressed-capable (MJPG/H264) USB **not** claimed by the driver.
4. **Driver (auto)** ← if unpinned, the first *remaining* YUYV-capable USB after recording picks.

`pickUsbRecordFormat()` prefers H264 > MJPG, largest area ≤1080p at 24–31 fps. `pickDriverFormat()`
prefers raw YUYV 640×480 (square-crops well for 224×224), slowest native rate (branch drops to 2 fps).

**Recognised profiles** (`describeConfiguration()`), logged as one `camera configuration: …` line
plus per-role detail. The two operator-facing anchors:

| Cameras present | Profile | Roles |
|---|---|---|
| 1 USB only | `RECORD-ONLY` | recording on the USB |
| 1 USB + IMX296 | `RECORD + LANES` | recording on the USB, lane detection on the IMX296 |

…plus `RECORD + DRIVER-MONITOR`, `RECORD + LANES + DRIVER-MONITOR` (full v0.3), and the degraded
`LANES-ONLY` / `…-MONITOR-ONLY` / `NONE`. **Design rule for role-absence logging:** recording is the
primary function (absence = `ERROR`); lane detection and driver monitoring are degradable
secondaries whose absence in a smaller rig is *by design* (absence = `INFO`) — a lone USB camera
must produce a clean `RECORD-ONLY` startup with **no** spurious error. Every subsystem degrades
independently; the app aborts only when *no* role can be filled.

Both anchor scenarios are verified on-device (single C270 → `RECORD-ONLY`; C270 + IMX296 →
`RECORD + LANES`, lane engine loaded, both files finalised).

## Config, models, storage

- **Config** `dashcam.xml` (`<DashcamConfig>`): sections `<Encoder> <Overlay> <Cameras> <System>
  <Pipeline> <Recording> <Detection> <DriverScore> <Log>`. Every field is a self-describing
  `ConfigVar<T>` (value + default + min/max/step + description written as XML attributes; numeric
  writes clamp). Model-locked params (input dims, class counts, mean/std) stay in each detector's
  own struct so the XML can't desync them from the engine. `loadOrCreate()` self-seeds a default
  file on first run. `DASHCAM_LOG_LEVEL` env overrides `<Log><Level>`.
- **Models** (`models/`): `culane_res18_fp16.engine` (lane), `drowsiness_resnet18_fp16.engine`
  (driver — retrained on akahana DDD, val 99.99%; the original Teen-Different weights were
  unusable), `face_detection_yunet_2023mar.onnx` (YuNet, vendored from OpenCV Zoo).
- **TRT engines are version+GPU locked** — build them with `trtexec` **inside this l4t-ml-gpio
  container on this Orin Nano**; a host-built or other-device engine will fail or misbehave.
- **Storage** resolves to the `/user/output/{footage,logs,configs}` mounts, falling back to
  `<exe_dir>/{footage,logs,config}` when a mount is missing (removed SD card keeps working).

## Build & run

- **Build:** plain `make` (no `BUILD_DIR` override, no ad-hoc `g++`) → timestamped
  `bin/build_<ts>/dashcam_v0_3` plus every test target. The `dashcam_v0_3` target links
  `liblog + libcamera + libconfig + librecord + liblanedetector + libdriverstate` and the two CUDA
  preprocess `.cu` objects; it needs TRT + OpenCV but **no VPI**. Build inside the container:
  ```bash
  docker run --rm --runtime nvidia -w /user/dashcam -v /home/$USER/dashcam:/user/dashcam \
    l4t-ml-gpio:latest bash -lc make
  ```
- **Run** (`docker_dev/launchcode_v0_3.sh`): `--runtime nvidia --privileged --network=host
  --ipc=host`, `-v /dev:/dev`, `-v /tmp/argus_socket:/tmp/argus_socket`, and the three
  `/user/output/*` mounts. **`--privileged` is required** — without it `/dev/videoN` opens fail with
  EPERM (the device-cgroup denies access even when `/dev` is bind-mounted), and discovery silently
  finds nothing. `--runtime nvidia` + the argus socket are what make in-container `nvarguscamerasrc`
  reach the host daemon.
- The dev image `l4t-ml-gpio:latest` is built from `dustynv/l4t-ml:r36.4.0` via
  `docker_dev/build.sh` (archives the previous `:latest` first).

## Resource cost & known quirks (this rig)

- **Two-model load** (lane + driver) measures ≈ **+8 pts CPU / +1.3 GB RAM / GPU 34 % avg** over
  recording alone — comfortable on the 8 GB unified board at MAXN_SUPER, but watch RAM if sign
  detection / stereo are added later.
- **`nvarguscamerasrc` teardown** (IMX296): the PAUSED→READY transition can block ~5 s internally on
  `stop()` (daemon-state dependent, not fixable app-side) — expect a few-second gap between "stopping
  pipeline" and "camera closing" at shutdown. `appsink` also defers EOS until drained (handled in
  teardown). Neither is a bug.
- **Thread model:** main loop (200 ms tick: telemetry clock, health, lane→scorer bridge, alert
  hysteresis) + one inference thread per active detector + the recorder's subtitle/bus thread + the
  camera streaming threads. Shared detector state is behind `poll()`/mutex; never block in a probe
  or Argus callback.
