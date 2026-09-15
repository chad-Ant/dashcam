# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Headless C++17 dashcam software for an NVIDIA **Jetson Orin Nano** (JetPack 6.2.x / L4T R36.5), built and run inside an L4T container. The rig: 1× IMX296 CSI camera (road-facing, inference), 1× USB UVC camera (road-facing, the evidentiary recording), 1× USB UVC camera (driver-facing, drowsiness), and an ESP32-C3 over USB supplying GPS/IMU/ECU telemetry.

Two skills in `.claude/skills/` carry the platform knowledge that is easy to get wrong; consult them before writing pipeline, inference, or container code:
- `jetson-dashcam-dev` — capture stacks, Orin Nano constraints, TensorRT, containers, and the severity-ranked code-review contract used here.
- `jetson-spe-firmware` — Cortex-R5 / AON firmware (not used by anything in `src/` or `lib/` today).

## Build, run, test

Plain `make` — no CMake, no test runner, no CI. Build inside the container.

```bash
docker_dev/build.sh            # build/refresh the l4t-ml-gpio:latest dev image
docker_dev/launchcode_dev.sh   # interactive shell in the container, cwd /user/dashcam

make -j6                       # every target
make -j6 dashcam_v0_4          # just the current app (+ seeds its logs/ and config/)
make clean                     # rm -rf bin/
```

**Every `make` creates a fresh timestamped `bin/build_<YYYYMMDD_HHMMSS>/` and rebuilds everything.** For an incremental rebuild, reuse a directory explicitly:

```bash
make -j6 BUILD_DIR=bin/build_20250101_120000
```

Only `dashcam_v0_3`, `dashcam_v0_4` and `commlink_test` have bare-name aliases; anything else is named by its full path under `BUILD_DIR`:

```bash
B=bin/build_20250101_120000 && make BUILD_DIR=$B $B/usb_test
```

Tests are standalone binaries under the build dir. **Most need real hardware** — there are no mocks. The exceptions worth knowing, because they run anywhere:

```bash
$B/dashcam_v0_4 --self-test-camera-configuration   # camera role assignment, no hardware
$B/config_test                                     # XML round-trip
$B/liblog_test                                     # re-execs itself with --child-level <lvl>
```

| Binary | What it does |
| --- | --- |
| `dashcam_v0_4` | Current app. v0.3 wired for unattended boot: recording-only startup, segmented recording, by-id camera pinning |
| `dashcam_v0_3` | UVC recording + IMX296 lanes + driver drowsiness |
| `dashcam_v0_2` | UVC recording + IMX296 lane detection |
| `dashcam` | Older production app (CSI + 3 USB + stereo) — needs VPI |
| `scan_cameras` | Enumerate V4L2 devices, formats, controls |
| `record_test` | UVC passthrough recording + ASS sidecar |
| `usb_test`, `csi_test [sensor-id]` | Camera lifecycle / raw Argus pipeline |
| `lane_test`, `driverstate_test` | Live inference smoke tests |
| `commlink_test`, `network_test`, `csi_rtp_test` | ESP32-C3 link, sockets/SNTP, CSI→RTP |
| `can_test`, `gpio_test`, `midi_test` | Peripheral interfaces |

`DASHCAM_LOG_LEVEL=debug|info|warn|error|off` overrides the configured log level at runtime.

**Retired, still in-tree, will not compile:** `dashcam_v0_1`, `test_single_record`, `test_dual_recording`, `recording_and_safe_shutdown`, `demo_graphical`, `demo_terminal`. They target the pre-passthrough librecord API. Don't add them back to `TARGETS` without porting them.

### VPI guard

The `dashcam` target and `libstereocam` need VPI 3.x headers (`vpi3-dev`). The Makefile probes `/usr/include/vpi/Image.h` and silently drops `dashcam` from `TARGETS` when absent. That is why `make` "isn't building dashcam".

### Deploying to boot

```bash
sudo deploy/install_dashcam_service.sh --now   # systemd unit -> docker_dev/launchcode_v0_4.sh
journalctl -u dashcam -f
sudo systemctl stop dashcam                    # SIGTERM -> finalises the last segment
```

The unit runs on the **host**; the app stays containerised. The launcher deliberately **fails fast** rather than building at boot, and warns loudly when the backup drive isn't mounted (see "Storage" below for why that matters).

## Architecture

### Layering

```
src/dashcam_v0_{2,3,4}.cpp, src/main.cpp, src/tests/*   ← wiring, role assignment, signals
  ├─ liblanedetector, libdriverstate, libsigndetector, libstereocam  ← inference/depth
  ├─ librecord     ← UVC passthrough recorder + ASS telemetry sidecar (owns its pipeline)
  ├─ libnetwork    ← WiFi/NTP/RTP/stream/control servers + HMAC auth
  ├─ libcommlink   ← ESP32-C3 telemetry bridge (sits on libuart)
  ├─ libcamera     ← iCamera / Camera_GST / Camera_CSI / Camera_USB
  ├─ libconfig     ← AppConfig, ConfigVar<T>, XML load/save
  ├─ libcan, libgpio, libi2c, libspi, libuart, libmidi   ← peripherals
  └─ liblog        ← async spdlog wrapper; bottom of the stack, depends on nothing
```

Two dependency edges are load-bearing and deliberately worked around rather than "fixed" by adding an include:
- **liblog cannot include libconfig** (libconfig already includes liblog). The app copies `config::LogConfig` → `log::LogParams` and passes it to `log::init()`.
- **libcamera cannot include libconfig** (libconfig.cpp includes libcamera.h). The app copies `config::PipelineConfig` → `camera::PipelineParams` and calls `setPipelineParams()`.

Every library takes a `dashcam::log::LogCallback` by injection rather than calling the logger directly.

### Recording — passthrough, not encoding

**`librecord` does not encode.** It records the camera's *already-compressed* MJPEG/H.264 stream straight into Matroska, which removes the software x264 stage entirely — the single most important design fact on a board with no NVENC. `startRecording()` **rejects raw pixel formats** outright.

Consequences that trip people up:
- The Recorder **owns its own pipeline** (`v4l2src ! … ! splitmuxsink`) and opens the V4L2 device **exclusively**. It does *not* attach to a `Camera_GST` tee, so you cannot record and run inference on the same physical camera — that is why recording and driver monitoring need separate UVC devices.
- CSI/Argus is deliberately unsupported for recording: it only produces raw frames.
- Telemetry is **not burned into the video**. It goes to an ASS subtitle sidecar next to the clip (`clip.mkv` → `clip.ass`), auto-loaded by mpv/VLC; `ffmpeg -i clip.mkv -vf ass=clip.ass` burns it in for export.
- With `RecordingConfig::segmentSeconds > 0` the tail is a `splitmuxsink` and each segment gets its own sidecar, rotated by the `format-location-full` signal with timestamps rebased per segment.

When touching the splitmuxsink path: **`muxer=` and `sink=` must be set in the parse string, not assigned afterwards.** splitmuxsink requests its sink pad from whatever muxer is current at link time; set them later and the queue links against the default `mp4mux`, which rejects the caps and fails the pipeline with "Internal data stream error" before a frame moves.

Telemetry validity is per-source and deliberately fine-grained (`speedValid`, `accelValid`, `positionValid`, `headingValid`, `laneValid`, `driverValid`). An invalid or stale domain renders as a dash, **never** as a held-over reading — a frozen-but-plausible speed burned into evidence is wrong by an amount too small to look wrong. Preserve that property when editing the sidecar writers.

### Camera model

`iCamera` is the contract: `open → setCameraVideoFormat → setCameraAttribute → start → captureFrame → stop → close`. Mutating methods return `void`; **errors surface through `getCameraStatus().currentError`**, not return codes.

`Camera_GST` implements the lifecycle with a template-method pattern; subclasses supply only `buildPipelineString()`, `cameraTypeTag()` and `pipelineError()`. The pipeline is always a `tee`:

```
<source + caps> ! tee name=srctee
  srctee. ! queue ! <convert> ! BGR ! appsink        ← captureFrame()
  srctee. ! queue ! valve ! <branch>                 ← addBranch(), one per consumer
```

- Branches register **before `start()`**; inference branches are leaky so old frames drop rather than backpressure capture.
- `setBranchEnabled()` toggles the valve at runtime with no rebuild.
- **A `stop()`/`start()` cycle clears all branches** — detectors must re-create their bin and re-register before the next `start()`.

### Camera role assignment

`resolveCameraConfiguration()` in the app inspects what is present and assigns each camera a role (record / lane / driver), then logs the recognised profile. Assignment order is load-bearing: a pinned cabin camera is reserved **before** recording picks, so the recorder never steals the device the operator set aside. Any role that cannot be filled stays OFF; the app keeps running.

Pinning, both via `<Camera>` entries in `dashcam.xml`:
- `name="cabin"` reserves the driver-monitoring camera.
- `name="record"` pins the road camera (v0.4+), and outranks any other USB entry.
- `<Device>` accepts a **`/dev/v4l/by-id/...` symlink**, resolved to its real node at startup. Prefer it: `/dev/videoN` is probe-order and not stable across reboots, which matters most when booting unattended.

v0.4 adds a second stage: roles are assigned as usual, then the `<Startup>` flags decide which actually start. That split is deliberate — the log still reports what the *rig* could run alongside what was switched on, so a camera left idle by config is distinguishable from one that was never detected.

### Config

`libconfig` is self-describing: every tunable is a `ConfigVar<T>` (`int`/`float`/`bool`/`std::string`) carrying value, default, min/max/step and description. It converts implicitly to `const T&`, so `cfg.encoder.bitrate` reads like a plain field; assignment clamps. `save()` writes the metadata as XML attributes, so the file documents itself.

Sections: `Encoder`, `Overlay`, `Cameras`, `System`, `Pipeline`, `Recording`, `Detection`, `DriverScore`, `Log`, `Network`, `Startup` → `AppConfig` under `<DashcamConfig>`.

Adding a tunable = add a `ConfigVar` with a PascalCase name matching the XML tag, **and** add `readVar`/`writeVar` lines to that section's parse/write function in `libconfig.cpp`. Missing elements fall back to the built-in default, so old config files keep working.

Two gotchas:
- `ConfigVar<std::string>` has no `.empty()` — copy into a local `std::string` first.
- **The build seeds `bin/build_<ts>/config/dashcam.xml` from `write_default_config`, i.e. from the struct defaults.** The repo's `config/dashcam.xml` is reference documentation and is *not* copied (only `camera_attributes.xml` is). So a default that matters must be right in the struct; shipping an XML will not do it.

`config/camera_attributes.xml` maps capability aliases to GStreamer source properties per camera type. **Adding a camera control needs no recompile — edit that XML.**

Model-locked parameters (input dims, class counts, normalisation) stay in each detector's own config struct, *not* in the XML, so they cannot be desynced from the TRT engine.

### Inference libraries

`liblanedetector` (UFLD v2), `libdriverstate` (YuNet face crop + ResNet18 drowsiness + long-horizon fatigue scoring) and `libsigndetector` share one shape: `createBin()` → register as a **leaky** branch → internal thread pulls frames from the branch appsink → CUDA preprocess (`*_preprocess.cu`, nvcc at `-arch=sm_87`) → TRT inference → `poll()` returns the latest result. `poll()` is thread-safe; nothing else is. Engines are built **on-target, inside the runtime container** (`trtexec --onnx=... --saveEngine=... --fp16`) and are not portable across TRT version or module.

### Runtime directories

Binaries resolve paths relative to the **executable**, not the cwd, so each build tree is self-contained. Deployment writes to bind mounts `/user/output/{footage,logs,configs}`; `resolveStorageDir()` probes writability and falls back to `<exe_dir>/{footage,logs,config}`.

**Storage gotcha worth knowing:** Docker *creates* a missing bind-mount source as root. If the backup drive isn't mounted, the directories are created on the internal disk, they are writable, the fallback never triggers, and footage silently lands on the NVMe. `launchcode_v0_4.sh` prints a banner for exactly this case — it is the only thing that distinguishes the two afterwards.

## Hardware constraints that shape the code

- **No NVENC on Orin Nano, and no DLA.** Recording is passthrough precisely to avoid encoding. Where encoding is unavoidable (RTP streaming of the inference cameras) it is software `x264enc` and costs real CPU. `nvv4l2h264enc`/`omxh264enc`/`h264_nvenc` do not exist here.
- **Headless.** No display sinks — `appsink`, `filesink`, `splitmuxsink`, `fakesink` only.
- IMX296 is natively 1456×1088; 1080p is upscaled. CSI-side downscale via `setOutputResolution()` is VIC-backed and ~free; `Camera_USB` ignores it.
- CSI needs `nvargus-daemon` up and `/tmp/argus_socket` bind-mounted. v0.4 only mounts it when it already exists, since mounting a missing path makes Docker create a *directory* that then breaks real CSI runs.
- Hot-path rule: no INFO logging inside the capture loop; WARN/ERROR on events only.
- `/dev/videoN` numbering is not stable across reboots — resolve cameras by config pin (preferably by-id), sysfs sensor name, or role assignment, never by index.

## Repo conventions

- Namespaces mirror libraries: `dashcam::camera`, `dashcam::config`, `dashcam::log`, `dashcam::record`, `dashcam::lane`, `dashcam::driver`, `dashcam::network`, `dashcam::commlink`, `dashcam::stereo`, `dashcam::gpio`, `dashcam::bus`, `dashcam::can`, `dashcam::midi`.
- Headers carry Doxygen file blocks with a `@code` usage example and explicit threading/ordering contracts. Comments here explain *why*, often at length, and frequently record a bug that the current shape exists to prevent — read them before changing the shape, and keep that standard in new code.
- A new app version is a new `src/dashcam_v0_N.cpp` rather than an edit in place; older versions stay buildable.
- Adding a target: define a `*_OBJS` list from the existing `LIB*_SRCS` groups, add it to `BASE_OBJS`, add the binary to `TARGETS`, and add a link rule with the right `LD_*` groups (`LD_BASE` always; `LD_CV` OpenCV, `LD_TRT` TensorRT, `LD_VPI`, `LD_GPIO`, `LD_ALSA`).
- Recordings (`*.mkv`), `bin/build*`, `logs/` and `archive/` are gitignored.
