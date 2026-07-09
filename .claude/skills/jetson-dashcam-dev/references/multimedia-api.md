# Jetson Multimedia API — libargus / V4L2 C++ path (Orin Nano, headless)

This is the **low-level, in-code** path — used instead of, or alongside, GStreamer/DeepStream when you need fine control (custom buffer handling, tight latency, no framework overhead) or when you're porting NVIDIA's C++ samples. On Orin Nano the encode half of this path is **software** (no NVENC).

Contents:
1. What the Multimedia API is
2. Camera architecture + API matrix (which path for which camera)
3. libargus capture (producer/consumer + EGLStream)
4. NvBufSurface (the buffer type everything shares) + format/memory rules
5. Software H.264 encode in C++ (the `19_argus_camera_sw_encode` sample) — the Orin Nano encode path
6. Hardware decode (V4L2 NVDEC) in C++ and via ffmpeg
7. Multi-camera + cross-process IPC (`nvunixfdsink`/`nvunixfdsrc`, new in JP 6.2)
8. Samples + `common/classes` you'll reuse
9. Headless notes + review pointers

---

## 1. What the Multimedia API is

A set of low-level APIs for flexible app development with direct control of the hardware blocks (an alternative to GStreamer). It comprises:
- **libargus** — camera capture (CSI, through the NVIDIA ISP).
- **V4L2 API** — encode, decode, and other media functions (the encoder/decoder are exposed as V4L2 devices).
- **NVOSD** — on-screen display.
- **Buffer Utility (`NvBufSurface`)** — allocation, management, sharing, transform, composition, blending of hardware buffers.

Docs: the header/reference is the *Jetson Linux API Reference* (download from the L4T release page). Samples install under `/usr/src/jetson_multimedia_api/` (via SDK Manager or the standalone package).

Use this path when a framework is in the way; use GStreamer/DeepStream (`deepstream-pipelines.md`) when you want batching, metadata, trackers, and OSD for free.

---

## 2. Camera architecture + API matrix

The NVIDIA camera stack, top to bottom: libargus / GStreamer / V4L2 applications → `nvarguscamerasrc` / `v4l2src` → libargus + Camera Core (+ tuning) → V4L2 MediaController framework → Tegra drivers (+ device tree) → kernel (UVC V4L2 driver for USB) → hardware (USB cam, CSI sensor, VI, ISP).

| Camera config | API to use | Notes |
|---|---|---|
| CSI **with** NVIDIA ISP driver (uses Jetson ISP) | **libargus** (or GStreamer `nvarguscamerasrc`) | Preferred path — ISP gives auto-exposure/AWB/demosaic for free. |
| CSI **without** ISP (raw Bayer out) | **V4L2** | Records raw Bayer; you debayer yourself. Painful — use only if no Argus driver. |
| USB (UVC) | **V4L2** | Drivers not provided by NVIDIA; UVC-class works out of the box. |

For a Bayer sensor without an integrated ISP (e.g. the reference OV5693), V4L2 records raw Bayer; `v4l2-ctl` can capture a raw frame for bring-up:
```bash
v4l2-ctl --set-fmt-video=width=1920,height=1080,pixelformat=RG10 \
  --stream-mmap --stream-count=1 -d /dev/video0 --stream-to=ov5693.raw
```
**Bring-up gotcha:** if `v4l2-ctl` captures fine but Argus (`argus_camera` / `nvarguscamerasrc`) fails, suspect the sensor driver's CID control functions (e.g. `..._set_exposure()`), because the Argus AE loop calls them continuously while `v4l2-ctl` does not.

For the dashcam: the IMX296 is a CSI camera — use the **libargus/Argus** path (it has ISP drivers on modern carrier BSPs). See `cameras.md` for IMX296 specifics (native **1456×1088**, no 1080p mode).

---

## 3. libargus capture (producer/consumer + EGLStream)

libargus uses a **producer/consumer** model:
- An **Argus producer thread** opens the camera driver, creates a `BufferOutputStream` (or an EGLStream producer), and issues **repeating capture requests** for N seconds, then closes the producer and driver.
- A **consumer thread** acquires frames from the stream and does something with them (encode, inference, render, copy to your app).

The stream between them is typically an **EGLStream**; libargus outputs **NV12 in block-linear NVMM** memory. Key exposure/ISP controls are set on the capture request (analogous to the `nvarguscamerasrc` properties in `cameras.md`: exposure time range, gain range, ISP digital gain, AWB, TNR, EE).

---

## 4. NvBufSurface — the shared buffer type + the rules that bite

`NvBufSurface` is the hardware surface abstraction every block hands around (capture → convert → encode/decode/infer). What matters for correctness on Orin Nano:

- **Memory layout:** Argus/ISP output is **block-linear** NVMM. Hardware blocks like the encoder/decoder/VIC understand this; **CPU code and the software (libx264) encoder do not** — they need **pitch-linear, CPU-accessible** memory.
- **Getting to CPU memory:** convert block-linear NVMM → CPU pitch-linear with **`NvBufSurfaceCopy`** (a hardware→software copy). Skipping this and passing block-linear buffers straight to a CPU consumer produces the classic **garbled/striped frame**.
- **Format conversion:** to change pixel format (e.g. to feed a format the encoder doesn't take), use **`NvBufSurfTransform`**.
- **Lifecycle (leak trap):** buffers come from a **finite pool**. Every acquired surface must be released back; a held ref exhausts the pool and you get `nvbuf_utils: Failed to create NvBufSurface` and a stalled pipeline. Pre-allocate the pool, and in every consumer path release on all exits (including error paths). This is a top review item (`code-review.md`).

---

## 5. Software H.264 encode in C++ — the Orin Nano encode path

Because Orin Nano has **no NVENC**, the in-code encode path uses **libav / libx264**. NVIDIA provides this as the **`19_argus_camera_sw_encode`** sample (a.k.a. `camera_sw_encode`) — shipped as a **separate BSP-sources package** you extract into `samples/` (it's not in the stock `00_`–`18_` set): libargus sets up the camera, an **EGLStream** connects capture to the software encoder, and encoded H.264 is written to file.

**Data flow:** Libargus `EGLStreamProducer` → `NvBufSurfaceCopy` (block-linear NVMM → CPU system memory) → LibAVEncoder (`AVFrame` in → encode YUV → `AVPacket` out) → write to file.

### Build & run
```bash
# 1. Get argus_cam_libavencoder_src.tbz2 from the L4T Driver Package (BSP) Sources.
# 2. Extract into the samples path:
#    /usr/src/jetson_multimedia_api/samples/
cd /usr/src/jetson_multimedia_api/samples/19_argus_camera_sw_encode
make
./camera_sw_encode [OPTIONS]     # writes an .h264 in the current dir
```
Options: `-r WxH` output resolution (default 640×480), `-f` filename (default `output.h264`), `-d` capture duration (default 5 s), `-i` camera index (default 0), `-v` verbose, `-p` input format `1=NV12 2=I420` (default NV12), `-H` help.

**For the IMX296 use `-r 1456x1088`** (native), not 1920×1080.

### Key classes / members (from the sample)
- **Argus Producer Thread** — opens the Argus driver, creates a `BufferOutputStream`, issues repeating capture requests for `CAPTURE_TIME` seconds.
- **ConsumerThread** — acquires from the `BufferOutputStream` and feeds the libav/libx264 encoder, saving the stream. Members: `const AVCodec* codec`, `AVCodecContext* m_enc_ctx`, `AVFrame* m_frame`, `AVPacket* m_pkt`, `NvBufSurface* sysBuffers[NUM_BUFFERS]` (the CPU-accessible copies). Methods: `createVideoEncoder`, `destroyVideoEncoder`, `libavEncode`.

### The libav/libx264 call sequence
`avcodec_find_encoder_by_name("libx264")` → `avcodec_alloc_context3` → `avcodec_open2` → `av_frame_alloc` / `av_packet_alloc` → per frame: `av_frame_make_writable`, `avcodec_send_frame`, `avcodec_receive_packet`, write, `av_packet_unref` → teardown: `avcodec_close`, `avcodec_free_context`, `av_frame_free`, `av_packet_free`. **Check every return** (`< 0` is an error; `avcodec_receive_packet` returns `EAGAIN`/`EOF` as normal control flow) — see `code-review.md`.

### Buffer compatibility (the format trap)
- NVENC (which you don't have) accepts I420/NV12/NV24/P010_10LE. **libx264 accepts only I420 and NV12** among those (other libx264 formats: I422, I444, NV16, NV21, GRAY, and 10-bit variants). To feed anything else, run **`NvBufSurfTransform`** first.
- The software encoder needs **CPU-accessible pitch-linear** memory → block-linear NVMM must go through **`NvBufSurfaceCopy`** first (see §4).

### Mapping HW-encoder settings to libx264
Directly mappable: bitrate control (VBR/CBR via `bitrate`, `vbv-maxrate`, `vbv-bufsize`); entropy mode (CABAC/CAVLC via `no-cabac`); lossless (`qp=0`); AUD insertion at IDR/I (`aud`); B-frames (`bframes`); reference frames (`ref`); profile/level (`profile`, `level`); extended/full-range color (`fullrange`); fps (`fps`); sample aspect ratio (`sar`). Other knobs: libx264 preset docs.

### Realistic preset performance (Orin Nano, 6 cores)
`x264enc` preset trades speed for size. Rough transcode numbers from NVIDIA's note (30 Mbps → 4 Mbps CBR): `ultrafast` ≈ 97 fps @ 47%/core, `veryfast` ≈ 61 fps @ 66%, `medium` ≈ 23 fps @ 85%, `slow` ≈ 15 fps, `veryslow` ≈ 4 fps. **Use `ultrafast` (or `superfast`) for realtime.** Tuning the GOP to mimic the HW encoder (IDR interval 30, I interval 30, `ref=1`, B-frames off, AQ off) recovers extra throughput.

### GStreamer equivalents (if you're not in C++)
```bash
# Camera → software H.264 → MP4 (capture native, encode; add videorate to drop fps):
gst-launch-1.0 nvarguscamerasrc ! \
  'video/x-raw(memory:NVMM),width=1456,height=1088,format=NV12,framerate=60/1' ! \
  nvvidconv ! 'video/x-raw,format=I420' ! x264enc tune=zerolatency speed-preset=ultrafast ! \
  h264parse ! qtmux ! filesink location=/data/clip.mp4 -e

# Transcode a recorded file (HW decode → SW encode):
gst-launch-1.0 filesrc location=in.mp4 ! qtdemux ! h264parse ! nvv4l2decoder ! \
  'video/x-raw(memory:NVMM),format=NV12' ! nvvidconv ! 'video/x-raw,format=I420' ! \
  x264enc tune=zerolatency speed-preset=ultrafast ! h264parse ! filesink location=/data/out.h264 -e
```

---

## 6. Hardware decode (NVDEC) — C++ V4L2 and ffmpeg

Decode **is** hardware-accelerated on Orin Nano (NVDEC). Two ways in code:

**V4L2 decoder (C++)** — the decoder is a V4L2 device with an output plane (compressed in) and a capture plane (frames out). Functional flow: `nvv4l2dec_create_decoder` (creates the decoder on `/dev/v4l2-nvdec`) → `subscribe_event` (resolution-change) → `set_output_plane_format` → start `capture_thread` → `set_capture_plane_format` → feed buffers (`q_buffer`/`dq_buffer`, `req_buffers_on_*_plane`) → `dq_event` on resolution change → get decoded frames → close. Key structs: `nvPacket`, `nvFrame`, `nvCodingType`, `BufferPlane`, `Buffer`, `context_t`.

**ffmpeg (NVIDIA build)** — supports HW decode of H.264/H.265/VP8/VP9/MPEG2/MPEG4 (note: the ffmpeg package doesn't support MPEG4 *container* files):
```bash
sudo apt update && sudo apt install -y ffmpeg
apt source ffmpeg        # if you need the sources
```

Useful for replaying recorded clips into an inference pipeline without a live camera, and for testing.

---

## 7. Multi-camera + cross-process IPC

**Multiple Argus cameras:** one producer per CSI port (`sensor-id=0`, `sensor-id=1`, …); verify port↔id mapping (don't assume). Compose feeds with **`nvcompositor`** (set `sink_N::xpos/ypos/width/height`). See `cameras.md` (multi-Argus) and `deepstream-pipelines.md` (per-source `nvstreammux` pad).

**Splitting a pipeline across processes** (e.g. one process owns the decoder/capture, others do inference and encode/stream independently): use the **`nvunixfdsink` / `nvunixfdsrc`** plugins — **new in JetPack 6.2** — which pass DMABUF fds over a Unix socket. This is exactly the pattern for "the front camera feeds several consumers that must not block each other."
```bash
# Server (owns capture/decode):
gst-launch-1.0 uridecodebin uri=<rtsp_or_file> ! queue ! \
  nvunixfdsink socket-path=/tmp/test1 sync=false -v
# Client (independent process — inference, or encode/stream):
gst-launch-1.0 nvunixfdsrc socket-path=/tmp/test1 ! \
  'video/x-raw(memory:NVMM),format=NV12' ! queue ! <your branch> -v
```
The server sends the decoded buffer's dmabuf fd to each client and reuses the buffer once all clients acknowledge. `nvgstipctestapp` is the reference sample (`nvgstipctestapp server <URL> <socket>` / `client <socket>`).
**Deprecated — do not use:** `nvipcpipelinesink`/`nvipcpipelinesrc`/`nvipcslavepipeline`. Prefer `nvunixfdsink`/`nvunixfdsrc`.

---

## 8. Samples and `common/classes` you'll reuse

Under `/usr/src/jetson_multimedia_api/`:
- **`samples/`** — the stock install is **`00_`–`18_`** (confirmed on this device). Most relevant here: `00_video_decode`, `01_video_encode`, `04_video_dec_trt` (decode + TensorRT), `07_video_convert`, `09_argus_camera_jpeg`, `10_argus_camera_recording`, `12_v4l2_camera_cuda` and `18_v4l2_camera_cuda_rgb` (USB/V4L2 + CUDA), **`13_argus_multi_camera`** (multi-CSI capture + composition — the multi-camera reference), and `14/15/16_multivideo_{decode,encode,transcode}`. **`19_argus_camera_sw_encode` is NOT in the stock set** — it appears only after you extract the `argus_cam_libavencoder_src.tbz2` BSP-sources package into `samples/` (see §5).
- **`common/classes/`** — reusable C++ wrappers (confirmed present): `NvVideoEncoder`, `NvVideoDecoder`, `NvJpegEncoder`, `NvJpegDecoder`, **`NvBufSurface`** *and* **`NvBuffer`** (both ship — `NvBufSurface` is the current JP6 buffer API; `NvBuffer` is the legacy layer kept for compatibility, so prefer `NvBufSurface` in new code), `NvEglRenderer` and `NvDrmRenderer` (**both display-only — skip on headless**), `NvApplicationProfiler`/`NvElementProfiler`, the V4L2 plumbing (`NvElement`, `NvV4l2Element`, `NvV4l2ElementPlane`), and `NvUtils`/`NvLogging`. Reuse these rather than re-implementing V4L2 plumbing.
- **`argus/`** — libargus samples and the `argus_camera` utility (enumerate/verify cameras).

When reviewing or extending a sample, cross-check the actual class against the installed headers — NVIDIA's demo code has had bugs, so verify against `common/classes/` and the API reference rather than trusting the snippet.

---

## 9. Headless notes + review pointers

- **No renderer.** `NvEglRenderer`, `nveglglessink`, `nv3dsink`, `nvdrmvideosink`, `nvoverlaysink` all need a display. Headless C++ apps end at your consumer (encode-to-file, feed inference, `appsink`, or write raw). Argus/V4L2 encode/decode themselves are display-free (Argus still needs `nvargus-daemon` running; in a container also bind-mount `/tmp/argus_socket` — see `containers.md`).
- **Threading.** The producer/consumer split is inherently multi-threaded; guard shared state and never block in the consumer callback. See `code-review.md`.
- **Buffer discipline.** Release every `NvBufSurface`; pre-allocate pools. The #1 crash in this path is pool exhaustion from a held ref.
- **Format/memory.** Block-linear NVMM → CPU needs `NvBufSurfaceCopy`; format changes need `NvBufSurfTransform`. Passing block-linear to a CPU/libx264 consumer without copy = garbled output.
