---
name: jetson-dashcam-dev
description: Use whenever the user is developing on an NVIDIA Jetson — especially Jetson Orin Nano on JetPack 6.2 — and the work touches AI dashcam features (multi-camera capture, on-device inference, event-triggered recording, GPS/speed overlays, lane/sign/driver detection), DeepStream SDK pipelines, GStreamer + TensorRT, the l4t-ml or deepstream containers from jetson-containers, CSI cameras via nvarguscamerasrc, USB cameras via v4l2src, smart recording, NVDEC/NVENC questions, or any combination of these. Use even when the user only mentions a fragment ("nvarguscamerasrc isn't working in my container", "what pipeline should I use for my IMX296", "trtexec engine build is OOMing"), and use proactively before answering Jetson questions from memory because version pairings and Orin Nano hardware specifics are version-sensitive and easy to get wrong. Also check for thread safety when generating code, use non-deterministic code or language features as low as reasonably possible, and proactively flag Orin Nano constraints (no NVENC, no DLA, RAM limits) when relevant to the user's question or plan.
---

# Jetson Dashcam Development

A skill for developing AI dashcam applications on NVIDIA Jetson Orin Nano (JetPack 6.2) with DeepStream SDK. Orin NX is mentioned where relevant as a drop-in upgrade path.

## Default assumptions (override if the user contradicts)

| | Value |
|---|---|
| Hardware | Jetson Orin Nano 8GB |
| OS | JetPack 6.2 = **L4T R36.4.3** = Ubuntu 22.04 + kernel 5.15 |
| CUDA | 12.6 |
| TensorRT | 10.3 |
| cuDNN | 9.x |
| DeepStream | **7.1** (the version paired with JP 6.2) |
| Runtime | Containers via `jetson-containers` (`l4t-ml` for ML, separate `deepstream` image for DeepStream) |
| Cameras | Mix of CSI (Argus or V4L2) and USB UVC |

If the user is on a different Jetson, different JetPack, or running on bare metal, surface that and adjust — version pairings change.

Verify on the device when in doubt:
```bash
cat /etc/nv_tegra_release           # L4T version (R36.4.3 = JP 6.2, R36.4.4 = JP 6.2.1)
dpkg -l | grep nvidia-l4t-core      # exact L4T core package
dpkg -l | grep -E 'cuda|tensorrt|deepstream'
```

## The single most important fact for this hardware

**Orin Nano has NO NVENC (no hardware video encoder).** It has NVDEC for decode, but every encoded frame must be produced in software on the CPU.

Architectural consequences for a dashcam:

- Continuous H.264 encoding of multiple 1080p streams will saturate the 6 Cortex-A78AE cores fast.
- Realistic budget for sustained 1080p30 software encoding (`x264enc tune=zerolatency speed-preset=ultrafast`): ~1–2 streams while still leaving CPU for inference/IO. Validate with `tegrastats` / `jtop`.
- Strategies the skill should suggest, in roughly this order:
  1. **Smart / event-triggered recording** instead of 24/7 record (see `references/event-recording.md`).
  2. Lower resolution / framerate for non-primary cameras.
  3. Drop framerate during inference-heavy windows.
  4. Last resort: upgrade hardware to Orin NX (drop-in module replacement, has NVENC + 1 DLA).
- **Decode is fine** — NVDEC accelerates H.264/H.265 input.

Always surface this constraint when the user asks about recording multiple streams. Suggesting `nvv4l2h264enc` or `omxh264enc` on Orin Nano is a wrong-answer red flag.

## Other Orin Nano-specific limits to keep in mind

- **No DLA cores.** Orin Nano has 0 DLA (Orin NX has 1, if upgrade is on the table). `--useDLACore=N` will fail.
- **GPU**: 1024 CUDA cores @ ~625 MHz, 1 SM count = 8 (Ampere). Modest by datacenter standards but fine for one or two TensorRT models.
- **RAM**: 8 GB unified (CPU + GPU share). Be intentional about model size; the 16 GB swap the user has helps for builds but you don't want to swap during inference.
- **Power**: defaults to 7W mode after boot. For dashcam workloads run `sudo nvpmodel -m 2` (MAXN_SUPER, ~15W) and `sudo jetson_clocks` once.

## How to use this skill (the routine)

When triggered, follow this loop:

1. **Confirm versions if there's any ambiguity.** Don't guess — ask the user to run the verify-on-device commands above.
2. **Pick the matching reference file(s)** (see "References map") and read them BEFORE answering. Do not answer a DeepStream question without reading `deepstream-pipelines.md`. Do not answer a container question without reading `containers.md`.
3. **Give concrete answers**, not generic Linux advice. Include actual `gst-launch-1.0` lines, actual config snippets, actual `jetson-containers run` invocations. Failure mode #1 of LLM Jetson advice is generality.
4. **Proactively flag Orin Nano constraints** (NVENC, DLA, RAM) when the user's plan brushes against them.
5. **Cite the source of facts** when uncertain — link to NVIDIA docs, the jetson-containers repo, NVIDIA developer forums, or the Jetson Linux Multimedia API reference rather than asserting from memory.

## References map

| Trigger words / topics | Read this |
|---|---|
| recording, encoding, multi-stream, throughput, `nvpmodel`, MAXN_SUPER, thermal, swap, DLA, CPU saturation | `references/orin-nano-constraints.md` |
| CSI, IMX219, IMX477, IMX296 (global shutter), `nvarguscamerasrc`, `v4l2src`, USB camera, UVC, `v4l2-ctl`, multi-cam | `references/cameras.md` |
| Docker, podman, `jetson-containers`, `l4t-ml`, `l4t-jetpack`, `deepstream` container, `--runtime nvidia`, device passthrough, argus_socket | `references/containers.md` |
| DeepStream, `nvinfer`, `nvtracker`, `nvstreammux`, `nvdsosd`, primary/secondary inference, `deepstream-app`, pyds, Python bindings | `references/deepstream-pipelines.md` |
| smart recording, event recording, pre/post buffer, ring buffer, G-sensor, accelerometer, GPS overlay, speed overlay, hard-braking, NvDsSRContext | `references/event-recording.md` |
| ONNX, TensorRT, `trtexec`, `.engine`, model conversion, YOLO, FP16, INT8, calibration | `references/models-and-tensorrt.md` |
| something is broken, error message, "no element X", crash, segfault, OOM, ImportError, `libnvinfer.so.X: cannot open` | `references/debugging.md` |

Multiple may apply; read all that match.

## Quick recipes (memorize these — don't make the user wait)

### List all cameras
```bash
# CSI (Argus-visible) sensors:
ls /dev/video*
sudo systemctl restart nvargus-daemon  # if Argus is wedged after a misbehaving client
# Everything V4L2 sees (USB UVC + V4L2-mode CSI):
v4l2-ctl --list-devices
# Per-device formats / framerates:
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### Sanity-check a CSI camera (Argus path)
```bash
gst-launch-1.0 nvarguscamerasrc num-buffers=120 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! 'video/x-raw,format=I420' ! fakesink
# With display:
gst-launch-1.0 nvarguscamerasrc ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! nv3dsink
```

If `nvarguscamerasrc` gives "no element": you're missing the GStreamer bad/nv plugins — almost always a container issue. See `references/containers.md`.

### Sanity-check a USB UVC camera
```bash
# Most laptop webcams: MJPEG at higher res, YUYV at lower
gst-launch-1.0 v4l2src device=/dev/video2 ! \
  'image/jpeg,width=1280,height=720,framerate=30/1' ! \
  jpegdec ! videoconvert ! fakesink
```

### Software-encode to MP4 (since no NVENC)
```bash
gst-launch-1.0 nvarguscamerasrc num-buffers=900 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! 'video/x-raw,format=I420' ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 ! \
  h264parse ! mp4mux ! filesink location=/data/clip.mp4
```

### Run l4t-ml with all cameras visible
```bash
# 1 CSI (typically /dev/video0) + 3 USB (numbering depends on plug order; check v4l2-ctl --list-devices)
jetson-containers run \
  --device /dev/video0 \
  --device /dev/video1 --device /dev/video2 --device /dev/video3 \
  --volume /tmp/argus_socket:/tmp/argus_socket \
  --volume /etc/enctune.conf:/etc/enctune.conf \
  $(autotag l4t-ml)
```

The `argus_socket` bind-mount is what makes `nvarguscamerasrc` work *inside* the container. Forgetting it is the #1 silent failure for in-container CSI capture.

**Note:** `l4t-ml` does NOT include DeepStream. For DeepStream pipelines, either use the official `nvcr.io/nvidia/deepstream:7.1-samples-multiarch` image or build the `deepstream` package from `dusty-nv/jetson-containers` (which has a known PYDS build issue on JetPack 6.2 — see `references/containers.md`).

### Set max performance once after boot
```bash
sudo nvpmodel -m 2     # MAXN_SUPER on Orin Nano 8GB (~15W boosted)
sudo nvpmodel -q       # confirm
sudo jetson_clocks     # lock all clocks high
```

## Common failure modes to guard against

1. **Suggesting `nvv4l2h264enc` / `omxh264enc` on Orin Nano** — no NVENC, won't work. Use `x264enc` for software encode.
2. **Suggesting DLA inference on Orin Nano** (`useDLACore=0/1`) — no DLA cores; will silently fall back or fail.
3. **Forgetting `argus_socket` in container run** — leads to `nvarguscamerasrc` working bare-metal but failing in-container.
4. **Forgetting that TensorRT engines aren't portable across Jetson modules** — even Orin-family to Orin-family (Nano vs NX vs AGX). Always build `.engine` files on the actual target device with `trtexec`, or let DeepStream auto-build from ONNX on first launch.
5. **Quoting GStreamer caps without `(memory:NVMM)`** when buffers need to stay on GPU between `nvargus*` / `nvvidconv` / `nvinfer`.
6. **Recommending stock apt OpenCV** — the apt-installed `python3-opencv` lacks CUDA and full GStreamer support. Use the `opencv` image from jetson-containers, or build from source with `WITH_CUDA=ON WITH_GSTREAMER=ON`.
7. **Suggesting `apt upgrade` on a working JetPack** — can clobber pinned `nvidia-l4t-*` packages and break the system. Use `apt-mark hold` on `nvidia-l4t-*` first.
8. **Asserting facts about JetPack 6.2 specifics from training memory** instead of consulting a reference. The Jetson stack moves fast; verify before claiming.

## When something is broken

Triage order (full version in `references/debugging.md`):
1. `tegrastats` or `jtop` — is it thermal, RAM, swap, or compute bound?
2. `dmesg | tail -50` — kernel-level camera or driver complaints
3. `sudo systemctl status nvargus-daemon` and restart if wedged
4. `GST_DEBUG=3 gst-launch-1.0 ...` for pipeline negotiation failures
5. `ldd` the failing binary for missing libraries; on JetPack 6.2, TensorRT-linked code expects `libnvinfer.so.10`, `libnvonnxparser.so.10`, etc.

## Closing reminder

The user is building a mission-critical AI dashcam with this specific camera architecture:

| Camera | Sensor | Capture | Workloads |
|---|---|---|---|
| Front CSI | IMX296 global shutter | 1080p **60 fps** | Lane detection, recording (with GPS+speed overlay), event-triggered save ±30 s, **plus** speed-limit sign reading throttled to 5 fps |
| USB cam A + B | Reused laptop webcams | 360p 10 fps each | Stereo rangefinder |
| USB cam C | Reused laptop webcam | 360p 10 fps | Driver behavior analysis |

Total: 1 CSI + 3 USB. Fully offline. Runs in the `l4t-ml` container.

Architecture notes that follow from this:
- The IMX296 stream is **multi-tasked** — one capture, multiple inference branches at different framerates. See `references/deepstream-pipelines.md` for the tee+`videorate`+`nvinfer interval` pattern.
- 1080p60 software encoding (`x264enc`, since no NVENC) is roughly **2× the CPU cost** of 1080p30. Recommend recording at 30 fps even though the sensor runs at 60 fps; the extra 60 fps frames are valuable for lane-detection AI but not for recorded evidence.
- Global shutter is ideal for sign reading (no rolling-shutter skew on fast-moving signs) and for any AI that uses optical flow or motion-derived features.

Keep this context in mind — recommendations should fit this architecture, not generic NVIDIA cloud demos.
