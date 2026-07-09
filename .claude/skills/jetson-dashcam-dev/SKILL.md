---
name: jetson-dashcam-dev
description: Use whenever the user builds, reviews, debugs, or fixes HEADLESS multi-camera apps on an NVIDIA Jetson — especially Orin Nano on JetPack 6.x. Covers CSI (nvarguscamerasrc/libargus), USB UVC (v4l2src), the V4L2+libargus Multimedia API (C++), multi-camera capture/composition and cross-process IPC; AI dashcam features (inference, event/smart recording, GPS/speed overlays, lane/sign/driver detection); DeepStream, GStreamer+TensorRT, software encode (Orin Nano has NO NVENC), NVDEC decode, and the l4t-ml/deepstream containers from jetson-containers. Trigger on fragments too ("pipeline for my IMX296", "trtexec OOM", "review this capture loop", "appsink hangs over SSH"). Prefer this over answering Jetson questions from memory — version pairings and Orin Nano specifics are easy to get wrong. Default to a headless target (no display sink), check thread safety, minimize non-determinism, apply the severity-ranked review + Power-of-Ten output contract, and flag Orin Nano limits (no NVENC/DLA, unified RAM, thermal).
---

# Jetson Dashcam / Headless Multi-Camera Development

A skill for **building, reviewing, and fixing** headless multi-camera applications on NVIDIA Jetson Orin Nano (JetPack 6.2.2 / L4T 36.5.0 on the target rig) — the concrete target being an AI dashcam. It covers three capture stacks (Argus/`nvarguscamerasrc`, V4L2/`v4l2src`, and the C++ libargus + V4L2 Jetson Multimedia API), DeepStream, GStreamer + TensorRT, software encode, and containers. Orin NX is mentioned where relevant as a drop-in upgrade path.

The three things this skill produces, per the user's request:
1. **Working code** — pipelines, capture/encode/inference apps, container invocations, device-tree/udev glue.
2. **Code reviews** — severity-ranked findings against the Jetson-specific and general-safety checklist in `references/code-review.md`.
3. **Bug + improvement notices** — concrete defects (wrong element for this HW, race conditions, buffer leaks, headless-incompatible sinks) and prioritized improvements.

## Default assumptions (override if the user contradicts)

| | Value |
|---|---|
| Hardware | Jetson Orin Nano 8GB |
| Host OS | **L4T R36.5.0 = JetPack 6.2.2** (Ubuntu 22.04, kernel 5.15). Kernel, Tegra drivers, `nvargus-daemon`, `/dev` nodes, `nvpmodel`, thermal all live host-side |
| Container | **`dustynv/l4t-ml:r36.4.0` (L4T 36.4.0)** — where this app builds and runs. `--runtime nvidia` mounts the host's 36.5.0 Tegra libs over the 36.4.0 base; the host↔container L4T skew is intentional and JP-6.2.x-compatible |
| CUDA | 12.6 |
| TensorRT | 10.3 |
| cuDNN | **9.3** |
| VPI | 3.2 |
| DeepStream | **7.1** (the 6.2.x-line pairing; requires JP 6.1+/L4T ≥ R36.4). Confirm with `deepstream-app --version` |
| Runtime | Containers via `jetson-containers` (`l4t-ml` for ML, separate `deepstream` image for DeepStream) |
| Cameras | Mix of CSI (Argus or V4L2) and USB UVC |
| Mode | **Headless** — SSH / autostart, no attached display |

If the user is on a different Jetson, different JetPack, or bare metal, surface that and adjust — version pairings change.

> **Version note:** the **host** is confirmed on **L4T R36.5.0 = JetPack 6.2.2**, but this app **runs in an L4T 36.4.0 container** (`dustynv/l4t-ml:r36.4.0`). Both are on the 6.2.x line and ship the identical compute stack — **CUDA 12.6 / TensorRT 10.3 / cuDNN 9.3 / VPI 3.2 / DLA 3.1** — with DeepStream **7.1**, so the table is accurate for either. **How the skew works:** the NVIDIA container runtime mounts the *host's* 36.5.0 Tegra userspace (the `nvarguscamerasrc`/`nvvidconv`/`NvBufSurface` plugins, the driver, and — via CSV — the multimedia libs) over the 36.4.0 base, while CUDA/cuDNN/TensorRT come from the container image. Net: the app links against the 36.4.0 container userspace but talks to host-side Argus (via the mounted `/tmp/argus_socket`) and host Tegra libs at runtime. **Build-vs-run:** build the app and any custom GStreamer plugins *inside* the r36.4.0 container, and build TensorRT engines inside that same container on this Orin Nano (engine ⇄ TRT-version ⇄ GPU must all match at runtime). Confirm on-device with `dpkg -l | grep -E 'cuda|tensorrt|cudnn|deepstream'` and `deepstream-app --version`; for any *other* L4T/JetPack version, web-search that version's release notes rather than trusting memory. Stable across the 36.4↔36.5 skew: the major SONAMEs (`libnvinfer.so.10`, `libcudart.so.12`, `libcudnn.so.9`) and the TRT-10 `--memPoolSize` flag. JP 6.2+ **cut Argus CPU ~40%** and added `nvunixfdsink`/`nvunixfdsrc`.

Verify on the device when in doubt:
```bash
cat /etc/nv_tegra_release           # L4T version (R36.4.3 = JP 6.2, R36.4.4 = JP 6.2.1, R36.5.0 = JP 6.2.2 — this device)
dpkg -l | grep nvidia-l4t-core      # exact L4T core package
dpkg -l | grep -E 'cuda|tensorrt|deepstream'
```

## The single most important fact for this hardware

**Orin Nano has NO NVENC (no hardware video encoder).** It has NVDEC for decode, but every encoded frame must be produced in software on the CPU. Confirmed in NVIDIA's "Software Encode in Orin Nano" application note (`references/multimedia-api.md` has the C++ path and the full libx264 API list).

Architectural consequences for a dashcam:

- Continuous H.264 encoding of multiple streams saturates the 6 Cortex-A78AE cores fast.
- Realistic budget for sustained software encode (`x264enc tune=zerolatency speed-preset=ultrafast`): ~1–2 1080p30-class streams while leaving CPU for inference/IO. Validate with `tegrastats` / `jtop`.
- Strategies to suggest, roughly in order:
  1. **Smart / event-triggered recording** instead of 24/7 record (`references/event-recording.md`).
  2. Lower resolution / framerate for non-primary cameras.
  3. Drop framerate during inference-heavy windows.
  4. Last resort: Orin NX (drop-in module, has NVENC + 1 DLA).
- **Decode is fine** — NVDEC accelerates H.264/H.265/VP9/AV1 input via `nvv4l2decoder`.

Suggesting `nvv4l2h264enc`/`nvv4l2h265enc`/`nvv4l2av1enc` or `omxh264enc` (dead on JP6) or `h264_nvenc` on Orin Nano is a **wrong-answer red flag**. Software options: `x264enc`, `x265enc` (2–3× cost), `nvjpegenc`/`jpegenc`, or the libav/libx264 C++ path.

## Headless-first (this changes almost every pipeline)

The app runs over SSH / as a service with **no display**. Nearly every example in the NVIDIA docs ends in a display sink — `nv3dsink`, `nveglglessink`, `nvdrmvideosink`, `xvimagesink`, `nvoverlaysink` — plus `export DISPLAY=:0`. **None of those belong in this app.** When generating or reviewing pipelines:

- Terminate branches with `appsink` (frames into your code), `filesink`/`splitmuxsink` (recording), `udpsink`/`rtph264pay` (streaming out), or `fakesink` (sanity checks / metadata-only via pad probes).
- For DeepStream headless, use `fakesink sync=0` + a metadata probe rather than `nvdsosd → nv3dsink`.
- Keep `nvargus-daemon` running on the host and (in containers) bind-mount `/tmp/argus_socket` — Argus does not need a display, but it does need the daemon + socket.
- A display sink appearing in a headless target is itself a bug to flag (see `references/code-review.md`).

## Other Orin Nano-specific limits (corrected for Super Mode)

The skill recommends **MAXN_SUPER** (`nvpmodel -m 2`), so quote the Super-mode figures, not the original ones:

- **NVENC:** none (above).
- **DLA:** **0 cores** (Orin NX has 1). `--useDLACore=N`, `enable-dla=1`, `precisionMode …--useDLACore` all fail / silently fall back. Never suggest DLA on Orin Nano.
- **GPU:** Ampere, 1024 CUDA cores across 8 SMs, 32 tensor cores. Under **MAXN_SUPER the GPU clocks up to ~1020 MHz** (vs ~635 MHz original) for **~67 sparse-INT8 TOPS** (vs ~40 original). Fine for one or two TensorRT models at moderate input size; will struggle with large transformer vision models in realtime.
- **CPU:** 6× Cortex-A78AE, **1.7 GHz under Super Mode** (vs 1.5 GHz original).
- **RAM:** 8 GB unified (CPU + GPU share). Be intentional about model size. 16 GB swap sits on a **dedicated NVMe** (`nvme1n1p1`, separate from the ~1 TB system NVMe) as a build-time safety net for TRT engine builds; with low `vm.swappiness` it isn't reached during normal inference (swap *priority* is moot with a single area). Any swap-in during inference means RAM is genuinely exhausted — a hard signal, not background noise.
- **Power (confirmed from this device's `nvpmodel.conf`):** the Orin Nano **8GB Super** profile has exactly three modes — **`0`=15 W, `1`=25 W (DEFAULT after boot), `2`=MAXN_SUPER (uncapped)**. There is **no 7 W** mode on the 8GB (that's the 4GB, which has 10 W). GPU cap per mode: 15 W→612 MHz, 25 W→918 MHz, MAXN_SUPER→uncapped (~1020 MHz). Quirk: 25 W caps the CPU *lower* than 15 W (1.34 vs 1.50 GHz), trading CPU for GPU/EMC. Set MAXN_SUPER once after boot:
  ```bash
  sudo nvpmodel -m 2      # MAXN_SUPER — index 2 confirmed on this rig
  sudo nvpmodel -q        # confirm the active mode name
  sudo jetson_clocks      # lock clocks high (does NOT persist across reboot)
  ```
- **Thermal (confirmed trips on this device):** `trip_point_0` reads **99 °C** on the cpu/gpu/cv/soc zones (throttle trips; Tj max ~100 °C) and **35 °C** on `tj-thermal` (a low fan/active-cooling trip). So hardware throttling is near ~99 °C — but sustained operation in the 90s °C means you're already thermally limited. In a sealed automotive enclosure, active cooling is mandatory. See `references/orin-nano-constraints.md`.

## Output contract (how to produce and review code here)

When **writing** code:
- Assume headless (above). No display sinks unless the user explicitly has a monitor.
- **Thread safety:** capture, inference, encode, IMU, and GPS run on separate threads. Guard shared state (GPS/speed snapshot, event flags, ring-buffer indices) with a mutex or a lock-free single-producer/single-consumer queue. Never block inside a GStreamer pad probe or an Argus consumer callback — copy out and return fast.
- **Determinism / robustness (Power-of-Ten spirit):** bound every loop; check every return (Argus `Status`, V4L2 ioctl, CUDA, `avcodec_*`, `NvBufSurface*`); no unbounded dynamic allocation on the hot path (pre-allocate buffer pools); keep functions small; validate every pointer/handle before use; prefer fixed-size buffers. Full checklist in `references/code-review.md`.
- Every GStreamer caps that must keep buffers on the GPU across `nvargus*`/`nvvidconv`/`nvinfer` uses `(memory:NVMM)`.

When **reviewing** code, read `references/code-review.md` and return findings **severity-ranked**:
- **BLOCKER** — will not work on this HW / crashes / data loss (e.g. `nvv4l2h264enc` on Orin Nano; display sink in a headless service; unreleased `NvBufSurface` exhausting the pool; missing `argus_socket` mount; DLA flag).
- **HIGH** — race condition, buffer leak, blocking in a probe, TRT engine shipped for the wrong module, `--workspace` on TRT 10.
- **MEDIUM** — perf/thermal footguns (encoding 60 fps when 30 suffices; no `queue` between tee branches; oversized model).
- **LOW / NIT** — style, naming, missing bounds asserts, magic numbers.

Each finding: file/line (or code region) → what's wrong → why it bites on *this* platform → concrete fix. End with a short prioritized improvements list.

## The routine (follow this loop when triggered)

1. **Confirm versions.** Have the user run the verify-on-device commands above. **When a specific L4T/JetPack version is in play, web-search that version's release notes** (docs.nvidia.com/jetson archives, or the JetPack release-notes page) to pin the CUDA/TensorRT/cuDNN/DeepStream pairing — don't infer it from memory, and don't infer a patch (e.g. 6.2.2) from the base release. Pairings and available BSPs change every release; a 30-second search beats a stale assumption. (This device is JP 6.2.2 / L4T 36.5.0 — stack per the table.)
2. **Pick the matching reference file(s)** (see map) and read them BEFORE answering. Don't answer a DeepStream question without reading `deepstream-pipelines.md`; a container question without `containers.md`; a C++ libargus/V4L2/encode question without `multimedia-api.md`; or perform a review without `code-review.md`.
3. **Give concrete answers** — actual `gst-launch-1.0` lines, actual config snippets, actual `jetson-containers run` invocations, actual C++. Failure mode #1 of LLM Jetson advice is generality.
4. **Proactively flag Orin Nano constraints** (NVENC, DLA, unified RAM, thermal) when the plan brushes against them, and **flag headless-incompatible sinks**.
5. **Prefer a live source over memory** for anything version-sensitive — NVIDIA docs/release notes, the jetson-containers repo, NVIDIA forums, the Jetson Linux Multimedia API reference. Web-search when a fact could have changed since training (versions, tags, package names, container images, known-issue status); assert from memory only for stable fundamentals.

## References map

| Trigger words / topics | Read this |
|---|---|
| recording, encoding, multi-stream, throughput, `nvpmodel`, MAXN_SUPER, thermal, swap, DLA, CPU saturation, TOPS, clocks | `references/orin-nano-constraints.md` |
| CSI, IMX219/IMX477/IMX296 (global shutter), `nvarguscamerasrc`, `v4l2src`, `nvv4l2camerasrc`, USB/UVC, `v4l2-ctl`, multi-cam, stereo, sensor-mode, exposure/gain | `references/cameras.md` |
| libargus C++, V4L2 C++ API, Jetson Multimedia API, `NvVideoEncoder`/`NvVideoDecoder`, `NvBufSurface`, EGLStream, software H.264 (libx264), `19_argus_camera_sw_encode`, `13_argus_multi_camera`, `nvunixfdsink`/`nvunixfdsrc` IPC, appsink | `references/multimedia-api.md` |
| Docker, podman, `jetson-containers`, `l4t-ml`, `l4t-jetpack`, `deepstream` container, `--runtime nvidia`, device passthrough, `argus_socket` | `references/containers.md` |
| DeepStream, `nvinfer`, `nvtracker`, `nvstreammux`, `nvdsosd`, PGIE/SGIE, `deepstream-app`, pyds | `references/deepstream-pipelines.md` |
| smart/event recording, pre/post buffer, ring buffer, G-sensor, accelerometer, GPS/speed overlay, hard-braking, `NvDsSRContext` | `references/event-recording.md` |
| ONNX, TensorRT, `trtexec`, `.engine`, model conversion, YOLO, FP16, INT8, calibration, `--memPoolSize` | `references/models-and-tensorrt.md` |
| review this code, is this correct, find bugs, Power of Ten, thread safety, buffer leak, race | `references/code-review.md` |
| broken, error message, "no element X", crash, segfault, OOM, ImportError, `libnvinfer.so.X: cannot open` | `references/debugging.md` |

Multiple may apply; read all that match.

## Quick recipes (memorize — don't make the user wait)

### List all cameras
```bash
ls /dev/video*
sudo systemctl restart nvargus-daemon    # if Argus is wedged after a misbehaving client
v4l2-ctl --list-devices                  # USB UVC + V4L2-mode CSI
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### Sanity-check the IMX296 CSI camera (Argus path) — headless
The IMX296 is a **1456×1088** global-shutter sensor (1.58 MP, ~60 fps max). It has **no native 1920×1080 mode** — capture native and only scale where needed.
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=120 ! \
  'video/x-raw(memory:NVMM),width=1456,height=1088,framerate=60/1,format=NV12' ! \
  nvvidconv ! 'video/x-raw,format=I420' ! fakesink   # no display sink
```
If `nvarguscamerasrc` gives "no element": you're missing the NV GStreamer plugins — almost always a container issue (`references/containers.md`).

### Sanity-check a USB UVC camera — headless
```bash
gst-launch-1.0 v4l2src device=/dev/video2 ! \
  'image/jpeg,width=1280,height=720,framerate=30/1' ! jpegdec ! videoconvert ! fakesink
```

### Software-encode to MP4 (no NVENC), capture native, encode at 30
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=900 ! \
  'video/x-raw(memory:NVMM),width=1456,height=1088,framerate=60/1,format=NV12' ! \
  nvvidconv ! 'video/x-raw,format=I420,framerate=30/1' ! videorate ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 ! \
  h264parse ! mp4mux ! filesink location=/data/clip.mp4
```

### Run l4t-ml with all cameras visible (1 CSI + 3 USB)
```bash
jetson-containers run \
  --device /dev/video0 \
  --device /dev/video1 --device /dev/video2 --device /dev/video3 \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --volume /etc/enctune.conf:/etc/enctune.conf \
  --volume /data:/data \
  $(autotag l4t-ml)
```
The `argus_socket` bind-mount is what makes in-container `nvarguscamerasrc` work — forgetting it is the #1 silent CSI failure. `l4t-ml` does **not** include DeepStream; use NGC `deepstream:7.1-samples-multiarch` or build via jetson-containers (known PYDS build issue on JP 6.2 — `references/containers.md`).

### Max performance once after boot
```bash
sudo nvpmodel -m 2      # MAXN_SUPER (~25 W) on Orin Nano 8GB — verify with -q
sudo nvpmodel -q
sudo jetson_clocks
```

## Common failure modes to guard against

1. **HW encoder on Orin Nano** (`nvv4l2h264enc`/`nvv4l2h265enc`/`nvv4l2av1enc`/`omxh264enc`/`h264_nvenc`) — no NVENC. Use `x264enc` or the libx264 C++ path.
2. **Display sink in a headless service** (`nv3dsink`/`nveglglessink`/`nvdrmvideosink`/`xvimagesink`/`nvoverlaysink`, `export DISPLAY=:0`) — use `appsink`/`filesink`/`udpsink`/`fakesink`.
3. **DLA inference on Orin Nano** (`useDLACore=0/1`, `enable-dla=1`) — no DLA cores.
4. **Forgetting `argus_socket`** in the container run — CSI works bare-metal, fails in-container.
5. **TRT engines assumed portable** — they aren't, even Orin-to-Orin (Nano vs NX vs AGX). Build `.engine` on the actual target, or let DeepStream auto-build from ONNX on first launch.
6. **`trtexec --workspace=N`** — deprecated/removed in TRT 10; use `--memPoolSize=workspace:N`.
7. **Caps without `(memory:NVMM)`** when buffers must stay on GPU between `nvargus*`/`nvvidconv`/`nvinfer`.
8. **Claiming an IMX296 does 1920×1080** — it's 1456×1088; any 1080p is upscaled (ISP/VIC cost, no new detail).
9. **Stock apt OpenCV** (`python3-opencv`) — lacks CUDA + full GStreamer. Use the jetson-containers `opencv` image or build `WITH_CUDA=ON WITH_GSTREAMER=ON`.
10. **`apt upgrade` on a working JetPack** — can clobber pinned `nvidia-l4t-*`. `apt-mark hold 'nvidia-l4t-*'` first.
11. **Blocking inside a pad probe / Argus consumer callback** — backpressures the whole pipeline; copy out and return.
12. **Asserting JP 6.2 specifics from memory** — verify against a reference; the stack moves fast.

## When something is broken

Triage order (full version in `references/debugging.md`):
1. `tegrastats` / `jtop` — thermal, RAM, swap, or compute bound?
2. `dmesg | tail -50` — kernel camera/USB/OOM complaints.
3. `sudo systemctl status nvargus-daemon` and restart if wedged.
4. `GST_DEBUG=3 gst-launch-1.0 …` for negotiation failures ("could not link X to Y").
5. `ldd` the failing binary; on JP 6.2, TRT-linked code expects `libnvinfer.so.10`, `libnvonnxparser.so.10`, `libcudart.so.12`, `libcudnn.so.9`.

## Closing reminder — the concrete build

Mission-critical AI dashcam, fully offline, in the `l4t-ml` container, headless:

| Camera | Sensor | Capture | Workloads |
|---|---|---|---|
| Front CSI | IMX296 global shutter | **1456×1088 @ 60 fps** (device-confirmed: single RG10 mode → ISP → NV12; no 1080p mode) | Lane detection + recording (GPS+speed overlay) + event save ±30 s + sign reading throttled to 5 fps |
| USB cam A + B | Reused laptop webcams | 360p 10 fps each | Stereo rangefinder |
| USB cam C | Reused laptop webcam | 360p 10 fps | Driver-behavior analysis |

Total: 1 CSI + 3 USB. Architecture notes that follow:
- The IMX296 stream is **multi-tasked** — one capture, multiple inference branches at different framerates via `tee` + `videorate` (or DeepStream `nvinfer interval=N`). See `references/deepstream-pipelines.md`.
- Capture at 60 fps for motion-rich inference, but `videorate` **down to 30 fps before the encoder** — 60 fps software encode is ~2× the CPU of 30 fps. The extra frames help lane/optical-flow AI, not recorded evidence.
- Global shutter is ideal for sign reading (no rolling-shutter skew) and optical-flow / geometric features.
- Recommendations should fit *this* headless architecture, not generic NVIDIA cloud demos.
