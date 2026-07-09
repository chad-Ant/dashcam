# Cameras on Jetson Orin Nano

## CSI vs USB: the mental model

CSI cameras attach to one of the two MIPI CSI connectors on the Orin Nano carrier board. USB cameras attach to USB 3.0 / 2.0 ports as standard UVC devices. They are accessed via *different* GStreamer elements and *different* underlying APIs.

| | CSI Argus path | CSI V4L2 path | USB UVC |
|---|---|---|---|
| GStreamer src | `nvarguscamerasrc` | `v4l2src` (with `nvv4l2camerasrc` on some setups) | `v4l2src` |
| Underlying API | libargus (NVIDIA ISP pipeline) | V4L2 directly | V4L2 directly |
| Sensor support | Sensors with NVIDIA ISP driver (IMX219, IMX477, IMX296, OV5693…) | Any V4L2-registered sensor | Any UVC-class device |
| Auto-exposure / ISP | Yes, by Argus | No (raw bayer out) | Up to the camera firmware |
| Output | NVMM buffer (NV12) | DMA buffer (raw bayer or YUV) | YUYV / MJPEG / etc. |
| Container requirement | Bind-mount `/tmp/argus_socket` | Just `--device /dev/videoN` | Just `--device /dev/videoN` |

Rule of thumb: **if it's a CSI camera with an Argus driver, use `nvarguscamerasrc`** — you get the ISP for free (auto-exposure, white balance, demosaic). Fall back to `v4l2src` only if the sensor lacks an Argus driver.

## Identifying what's connected

```bash
# Everything V4L2 sees:
v4l2-ctl --list-devices

# Per-device detail:
v4l2-ctl -d /dev/video0 --all
v4l2-ctl -d /dev/video0 --list-formats-ext   # supported formats + framerates

# What does Argus see? Tegra-multimedia-api gives the cleanest answer:
# (run from device, may need to install the multimedia samples)
/usr/src/jetson_multimedia_api/argus/build/samples/utils/argus_camera   # if built

# Quick "is the sensor present on I2C" check:
sudo i2cdetect -r -y 9    # IMX296 confirmed at bus 9, addr 0x1a (v4l2 name: imx296 9-001a)
```

> **Status on this rig (device-confirmed):** with the camera attached, `/dev/video0` enumerates as **`vi-output, imx296 9-001a`** (Sony IMX296 on I2C bus 9, address 0x1a) under `platform:tegra-capture-vi`. `v4l2-ctl -d /dev/video0 --list-formats-ext` reports **exactly one mode**: **`RG10` (10-bit Bayer RGRG/GBGB), discrete 1456×1088 @ 60.000 fps**. There is **no 1920×1080 mode** — the earlier 1456×1088 correction is now verified against the sensor itself. Two consequences:
> - **V4L2 path** (`v4l2src`/`nvv4l2camerasrc`) gives you **raw RG10 Bayer at 1456×1088** — you'd debayer yourself (hard).
> - **Argus path** (`nvarguscamerasrc`) runs this RG10 through the ISP and outputs **NV12 at 1456×1088@60** — use this. Cap your pipeline at `width=1456,height=1088,format=NV12,framerate=60/1`; any "1080p" downstream is ISP/VIC upscaling.
>
> (`gst-inspect-1.0 nvarguscamerasrc` still shows only the plugin's generic template caps `[1, 2147483647]` and `total-sensor-modes=0` — those populate at *runtime*, not from `gst-inspect`, so they don't contradict the single V4L2 mode above.)

## CSI camera (Argus path)

Standard pipeline:
```
nvarguscamerasrc sensor-id=0 ! 
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12' ! 
  ...
```

Key properties:
- `sensor-id=0` / `sensor-id=1` — selects the CSI port. Verify mapping with the Argus camera sample; do not assume CSI0 is sensor-id 0.
- `sensor-mode=N` — picks one of the sensor's published modes (resolution + framerate combos). Use `-1` for auto. Inspect modes:
  ```bash
  gst-inspect-1.0 nvarguscamerasrc
  # Or for richer info, look at /var/log/Xorg.0.log (no, this is Argus daemon log):
  sudo journalctl -u nvargus-daemon -f
  ```
- `wbmode=N`, `tnr-mode=N`, `ee-mode=N` — white balance, temporal noise reduction, edge enhancement. ISP knobs.
- `exposurecompensation=`, `gainrange=`, `exposuretimerange=`, `ispdigitalgainrange=` — manual exposure controls.

The output is always in NVMM memory. To use buffers on CPU, convert with `nvvidconv`:
```
nvarguscamerasrc ! 'video/x-raw(memory:NVMM),...' ! nvvidconv ! 'video/x-raw,format=I420' ! ...
```

### Multi-camera Argus

Multiple CSI cameras can run concurrently if your carrier has more than one CSI port populated. Use one `nvarguscamerasrc` element per port with `sensor-id=0`, `sensor-id=1`, etc. For DeepStream, feed each into a different `nvstreammux` sink pad.

If Argus daemon gets confused after a crash, all cameras may go dark. Recover with:
```bash
sudo systemctl restart nvargus-daemon
```

### IMX296 (global shutter) at 1456×1088 @ 60 fps — the dashcam's primary sensor

**Resolution reality check:** the Sony IMX296 is a **1456×1088** (1.58 MP) global-shutter sensor with a ~60 fps max at full resolution. It has **no native 1920×1080 mode** — you physically cannot capture true 1080p from it. Anything labelled "1080p" from an IMX296 is the ISP/VIC **upscaling** 1456×1088, which costs cycles and adds zero real detail. Capture native 1456×1088 and only scale at the point of need (e.g. a model that wants a specific input size). If the user insists their module does 1080p, confirm the actual sensor-mode list (`gst-inspect-1.0 nvarguscamerasrc`, or the Argus sample) before believing it.

The user's front camera is the IMX296 running **1456×1088 @ 60 fps**. Notes specific to this configuration:

- **Argus support**: The IMX296 has Argus drivers on most modern carrier-board BSPs (Connect Tech, Leopard Imaging). Verify with `gst-inspect-1.0 nvarguscamerasrc` and a sanity capture; if Argus doesn't see it, you're on the V4L2 path with manual debayer (much harder).
- **Why global shutter matters for this build**: it eliminates rolling-shutter skew. That's important for (a) reading road signs at speed without text being warped, and (b) any AI feature that relies on optical flow or geometric correspondence between frames (e.g., visual odometry, stereo derived from sequential frames).
- **Sample pipeline** (Argus path, native 1456×1088 @ 60, headless):
  ```bash
  gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
    'video/x-raw(memory:NVMM),width=1456,height=1088,framerate=60/1,format=NV12' ! \
    nvvidconv ! 'video/x-raw,format=I420' ! fakesink   # fakesink, not a display sink
  ```
- **Exposure control**: a global shutter at 60 fps has at most ~16 ms of exposure per frame. In low light (tunnels, dusk), you'll want to push gain up rather than extending exposure (which 60 fps caps anyway). Use `gainrange="1 16" exposuretimerange="13000 16000000"` (nanoseconds) and let Argus auto-balance, or fix gain manually if you see auto-exposure thrash.
- **Multi-tasking the stream**: one capture at 60 fps feeds multiple inference workloads at different rates (lane detection at 30/60 fps, sign reading at 5 fps, recording at 30 fps). Don't open the camera multiple times — use a `tee` + `videorate` per branch, or DeepStream's `nvinfer interval=N` to skip frames per inference. See `deepstream-pipelines.md`.
- **60 fps + software encoding is tight**: `x264enc` at 1456×1088@60 ultrafast is roughly 2× the CPU cost of the same frame at 30 fps. Recommend recording at 30 fps (downsample after the tee with `videorate ! video/x-raw,framerate=30/1`) while keeping the inference branches at 60 fps for motion-rich features. There is no NVENC to fall back on. See `orin-nano-constraints.md` and `event-recording.md`.

## CSI camera (V4L2 path)

For sensors without Argus drivers (e.g., bare bayer sensors with custom kernel modules), you'll get raw frames out. You'll need to debayer in software or with VPI. This path is significantly more painful — use only if Argus isn't an option.

```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=1920,height=1080,pixelformat=BG10
gst-launch-1.0 v4l2src device=/dev/video0 ! ... ! bayer2rgb ! ...
```

The user has an IMX296 global-shutter CSI camera. Path notes:
- **IMX296** — has Argus support on newer L4T BSPs; check `gst-inspect-1.0 nvarguscamerasrc`. Argus path strongly preferred (you get auto-exposure / WB for free).
- If Argus doesn't recognize it, the carrier board's BSP doesn't include the Argus driver — you'd have to write or source one, or fall back to V4L2 + manual debayer. Ask the user the carrier board (Seeed reComputer, Connect Tech Boson, Leopard, custom?) before going down this road.

## USB UVC cameras

Standard pipeline:
```
v4l2src device=/dev/video2 ! 
  'image/jpeg,width=1280,height=720,framerate=30/1' ! 
  jpegdec ! videoconvert ! ...
```

Most laptop webcams output:
- **MJPEG** at higher resolutions (720p, 1080p) — needs `jpegdec`
- **YUYV** at lower resolutions / framerates — direct via `videoconvert`

Check what the camera supports:
```bash
v4l2-ctl -d /dev/video2 --list-formats-ext
```

### USB camera notes

- USB 2.0 bandwidth (~480 Mbps shared) limits how many cameras you can simultaneously stream. Three 720p30 MJPEG cameras fit; three 1080p30 raw YUYV cameras do not.
- `/dev/videoN` numbering is not stable across reboots if multiple USB cameras are plugged in. For reproducibility, use udev rules to assign symlinks like `/dev/dashcam_driver_facing` based on the camera's USB serial or path.
  ```
  # /etc/udev/rules.d/99-dashcam.rules
  SUBSYSTEM=="video4linux", KERNEL=="video*", \
    ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0825", \
    ATTRS{serial}=="ABCDEF", \
    SYMLINK+="dashcam_driver_facing"
  ```
- USB 3.0 ports give a full 5 Gbps lane each — much better for higher-bandwidth USB cameras. Plug demanding cameras into USB 3 first.

## Stereo USB cameras (rangefinder) — 360p @ 10 fps each

Two reused laptop webcams operating at 360p / 10 fps for stereo depth. Notes:

1. **Hardware sync** — UVC doesn't standardize external triggers, and reused webcams definitely don't sync. Frames from the two cameras drift in time; on a moving vehicle this means depth gets noisy whenever the scene or vehicle moves. Mitigations:
   - Capture timestamps with `v4l2_buffer.timestamp` per frame and pair the closest matches; expect ±50 ms misalignment at 10 fps.
   - Accept the result is rough — fine for "is there an obstacle in the next 5 m" coarse rangefinding, **not** fine for safety-critical AEB-style decisions.
   - If accuracy matters more later, swap to a synced pair (Stereolabs ZED, Arducam Stereo HAT, or two Argus-driven CSI cameras hardware-triggered).
2. **Calibration** — intrinsic + extrinsic via OpenCV's `cv2.stereoCalibrate` once, save to `/data/calib/stereo.yaml`, reuse forever. Recalibrate if you remount the cameras.
3. **Disparity computation** — at 360p / 10 fps the compute cost is small, so plenty of options:
   - `cv2.StereoBM` (fastest, blockmatching) — fine here.
   - `cv2.StereoSGBM` (slower, better at textureless surfaces) — also fine at this resolution.
   - VPI's CUDA stereo (`vpi.stereodisp`) — overkill for 360p10 but available if you want GPU offload:
     ```python
     import vpi
     with vpi.Backend.CUDA:
         disparity = vpi.stereodisp(left, right, maxdisp=64)
     ```
4. **Bandwidth check**: 360p MJPEG at 10 fps per camera is ~5–10 Mbps each — comfortably fits USB 2.0 even with the driver-monitoring camera on the same root hub. If you went to 720p or 30 fps you'd need to think harder about USB topology.
5. **Be honest with the user** about the limits of two unsynced webcams as a rangefinder on a moving vehicle. Useful for prototyping; not a production safety sensor.

## Driver-monitoring camera (single USB, 360p @ 10 fps)

A reused laptop webcam at 360p / 10 fps facing the driver. Notes:

- **Resolution is fine for the task**: face/eye landmark detection works at 360p (mediapipe's face mesh is happy down to ~240p). 10 fps is enough for blink rate and head-pose tracking; you'd want 30 fps for accurate microsleep detection but it's a reasonable v0.1.
- **Lighting is the real problem.** A reused laptop webcam has no IR illumination and a small fixed-aperture lens. Day driving: fine. Night driving: face will be near-black except where dashboard light spills onto it. Production DMS uses 850nm or 940nm IR LEDs with an IR-pass filter on the camera so the system works in the dark *and* isn't fooled by sunglasses.
- Be honest with the user: this camera will work for daytime prototyping, but expect to swap it for an IR-illuminated camera (e.g., a cheap ELP IR USB module, ~$30) before night driving is reliable.
- **Models that work at 360p / 10 fps on Orin Nano**:
  - mediapipe face mesh — runs on CPU, free of GPU contention.
  - Eye Aspect Ratio (EAR) for drowsiness — trivial compute.
  - Head pose via PnP from mediapipe landmarks — trivial compute.
  - Lightweight CNN classifier for distraction (phone-to-ear, looking-down) — small TRT engine.
- **Latency tolerance is loose** (this is a behavioral monitor, not a safety-critical loop), so don't sacrifice front-camera resources for DMS responsiveness.
