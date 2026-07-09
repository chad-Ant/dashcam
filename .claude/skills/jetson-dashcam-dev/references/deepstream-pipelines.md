# DeepStream Pipelines

DeepStream is NVIDIA's high-level framework over GStreamer for AI video pipelines. On JetPack 6.2 the matched version is **DeepStream 7.1**.

Two ways to use it:
1. **Config-driven (`deepstream-app`)** — write `.txt` config files, run `deepstream-app -c yourapp.txt`. Fastest start, hardest to extend.
2. **Programmatic (Python `pyds` or C/C++)** — full GStreamer pipeline in code, with DeepStream's `nvinfer`, `nvtracker`, `nvstreammux`, `nvdsosd` elements. More flexible; required once you do anything custom (GPS overlay probes, custom recording triggers, IPC).

For a dashcam, you'll outgrow `deepstream-app` quickly — plan on a Python or C++ pipeline.

## The DeepStream pipeline shape

```
[source 0] ─┐
            ├─→ nvstreammux ─→ nvinfer (PGIE) ─→ nvtracker ─→ nvinfer (SGIE) ─→ nvdsosd ─→ tee ─→ [display sink]
[source 1] ─┘                                                                                  └→ [encoder + filesink]
```

Components:
- **`nvstreammux`** — batches frames from N sources. `batch-size` should equal source count (or a multiple). Output is batched NVMM frames.
- **`nvinfer`** (PGIE = primary inference) — runs a TensorRT model on each frame. Produces detection metadata attached to the buffer.
- **`nvtracker`** — assigns persistent IDs to detections across frames. Backends: IOU, NvDCF, NvSORT.
- **`nvinfer`** (SGIE = secondary inference) — runs a second model **on each detected ROI** from PGIE. This is how you do "detect signs, then OCR each sign" in one pipeline.
- **`nvdsosd`** — draws boxes / text on frames using the detection metadata.
- **`tee`** — duplicate the buffer for multiple sinks (display + record).

## Multi-source mux (general)

For multi-camera DeepStream pipelines, batch sources via `nvstreammux`:
```
nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=1456,height=1088,framerate=60/1' ! mux.sink_0   # IMX296 native
v4l2src device=/dev/video1 ! 'image/jpeg,width=640,height=360,framerate=10/1' ! jpegdec ! videoconvert ! nvvideoconvert ! 'video/x-raw(memory:NVMM)' ! mux.sink_1
nvstreammux name=mux batch-size=2 width=1920 height=1080 batched-push-timeout=40000 live-source=1 ! 
  ...
```

`nvstreammux` properties for live cameras:
- `live-source=1` — required for live capture; sets sync semantics correctly.
- `batched-push-timeout=40000` (microseconds) — wait this long for a batch to fill before pushing partial. ~40 ms is fine for 30 fps; lower (~16000) for 60 fps.
- `width`/`height` — frames are scaled to this for the muxer's batched buffer; pick the largest source size, smaller streams are padded.

For mixing sources at radically different framerates (CSI@60 + USB@10), you generally want **separate pipelines or a tee** rather than one shared `nvstreammux` — the muxer can't usefully batch streams that produce frames 6× apart in time.

## Single-camera multi-task pattern (the dashcam's primary pipeline)

The user's IMX296 runs at 1080p60 and feeds **three workloads at three different framerates**:
- Lane detection — full 60 fps (or 30 fps if compute is tight)
- Sign reading — 5 fps (signs don't move much frame-to-frame, no need for 60)
- Recording — 30 fps to disk (encode budget)

Don't open the camera multiple times. One `nvarguscamerasrc`, then `tee`, then per-branch rate control. Two ways to throttle:

**(a) `nvinfer interval=N`** — DeepStream's built-in frame skipping for inference. `interval=0` runs every frame; `interval=N` skips N frames between inferences. For 5 fps from 60 fps source: `interval=11` (process 1 of every 12 frames).

**(b) `videorate` element + caps** — actually drops frames in the GStreamer pipeline. Use this when you want a downstream element (encoder, OSD) to see fewer frames, not just the inference engine.

### Skeleton

```
nvarguscamerasrc sensor-id=0
  → caps NVMM 1456x1088@60   // IMX296 native; not 1080p
  → nvstreammux (batch-size=1, width=1920, height=1080, live-source=1, batched-push-timeout=16000)
  → pgie_lane_obstacle (nvinfer, interval=0)   // every frame, 60 fps
  → nvtracker (NvSORT)
  → tee
       ├→ branch_signs:   queue ! nvinfer interval=11 (config_sign_detector.txt) ! ... // ≈5 fps effective
       ├→ branch_record:  queue ! videorate ! 'video/x-raw(memory:NVMM),framerate=30/1' ! nvvidconv ! x264enc ... ! splitmuxsink
       └→ branch_display: queue ! nvdsosd ! nv3dsink   // optional, for HDMI dev work; remove in prod
```

Notes on this layout:
- The PGIE runs lane + obstacle detection at 60 fps because lane geometry changes fast at highway speed.
- The sign-detector is a *second* `nvinfer` on the same buffer, with `interval=11` to throttle. Its `gie-unique-id` must differ from the PGIE's.
- The recording branch uses `videorate` to drop to 30 fps **before** the encoder, so `x264enc` is only encoding 30 fps worth of work.
- Each branch starts with a `queue` so they run on independent threads — without this, a slow branch (the encoder) backpressures the others.
- `nvtracker` after the PGIE keeps object IDs persistent — useful for "did this car appear, get tracked, and then trigger the recording event?" workflows.

### Two `nvinfer` elements on the same source

Yes, you can chain two PGIEs on one source — they're not mutually exclusive. The pattern:
```
... → pgie_lane_obstacle (gie-unique-id=1) → pgie_signs (gie-unique-id=2, interval=11) → ...
```

Each `nvinfer` adds its detections to the buffer's metadata under its `gie-unique-id`. Downstream consumers (OSD, your custom probe) iterate detections per GIE.

### Tracker choice for the dashcam

| Tracker | Use when |
|---|---|
| `IOUTracker` | Fastest, OK for stationary cameras with sparse objects. **Not great for a moving dashcam** — IDs swap during ego-motion. |
| `NvSORT` | Lightweight Kalman-based. Good default for moving platforms. |
| `NvDCF` | Discriminative correlation filter; best ID consistency, more compute. Default in DeepStream samples. |
| `NvDeepSORT` | Adds re-ID embeddings. Best when cars frequently occlude each other. Heaviest; may not fit Orin Nano budget alongside other inference. |

For dashcam v0.1 on Orin Nano: **NvSORT** as a balance of cost and quality. Upgrade to NvDCF if you see ID swaps in real-world testing.

The stock low-level tracker configs live in `/opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/` (all confirmed present on this device): `config_tracker_IOU.yml`, `config_tracker_NvSORT.yml`, `config_tracker_NvDCF_perf.yml`, `config_tracker_NvDCF_accuracy.yml`, `config_tracker_NvDCF_max_perf.yml`, `config_tracker_NvDeepSORT.yml`. They all share one low-level lib, `/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so`. For the dashcam, point `ll-config-file` at **`config_tracker_NvSORT.yml`**.

## What about the USB cameras?

The 3 USB cameras (stereo pair @ 360p10 + driver cam @ 360p10) don't need DeepStream. They're low-rate, single-task, and DeepStream's value (batched inference, metadata pipeline, OSD, Smart Record) doesn't pay off at 10 fps.

Recommended split:
- **DeepStream** for the IMX296 (lane + obstacle + sign + record).
- **Plain Python + OpenCV + raw TensorRT** (or even mediapipe on CPU) for the stereo and driver cameras. See `models-and-tensorrt.md` for the raw-TRT-in-Python skeleton.
- **IPC between them** if you need fusion (e.g., "driver was distracted *and* hard braking event") — Unix domain socket, ZeroMQ, or shared memory ring.

## Python (`pyds`) skeleton for a dashcam

```python
import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import pyds

Gst.init(None)
pipeline = Gst.Pipeline.new("dashcam")

# Source 0: front CSI camera (IMX296, 1080p60)
src0 = Gst.ElementFactory.make("nvarguscamerasrc", "src0")
src0.set_property("sensor-id", 0)
caps0 = Gst.ElementFactory.make("capsfilter", "caps0")
caps0.set_property("caps", Gst.Caps.from_string(
    "video/x-raw(memory:NVMM),width=1456,height=1088,framerate=60/1,format=NV12"))  # IMX296 native

# nvstreammux
mux = Gst.ElementFactory.make("nvstreammux", "mux")
mux.set_property("batch-size", 1)
mux.set_property("width", 1456)    # match the IMX296 native frame; setting 1920 here would upscale
mux.set_property("height", 1088)
mux.set_property("live-source", 1)
mux.set_property("batched-push-timeout", 16000)   # 16 ms ≈ 1 frame at 60 fps

# Inference
pgie = Gst.ElementFactory.make("nvinfer", "pgie")
pgie.set_property("config-file-path", "/configs/pgie_dashcam.txt")

# Tracker
tracker = Gst.ElementFactory.make("nvtracker", "tracker")
tracker.set_property("ll-config-file", "/opt/nvidia/deepstream/deepstream/samples/configs/deepstream-app/config_tracker_NvDCF_perf.yml")  # swap to config_tracker_NvSORT.yml for the dashcam
tracker.set_property("ll-lib-file", "/opt/nvidia/deepstream/deepstream/lib/libnvds_nvmultiobjecttracker.so")

# OSD for overlay
nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "nvvidconv")
osd = Gst.ElementFactory.make("nvdsosd", "osd")

# Sink — for headless, use fakesink + a probe to extract metadata
sink = Gst.ElementFactory.make("fakesink", "sink")
sink.set_property("sync", 0)

for el in [src0, caps0, mux, pgie, tracker, nvvidconv, osd, sink]:
    pipeline.add(el)

src0.link(caps0)
srcpad = caps0.get_static_pad("src")
sinkpad = mux.get_request_pad("sink_0")   # correct for nvstreammux (see note below)
srcpad.link(sinkpad)
mux.link(pgie)
pgie.link(tracker)
tracker.link(nvvidconv)
nvvidconv.link(osd)
osd.link(sink)

# Probe on osd's src pad to read detection metadata frame-by-frame
def osd_sink_pad_buffer_probe(pad, info, u_data):
    gst_buffer = info.get_buffer()
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    while l_frame is not None:
        frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        # iterate frame_meta.obj_meta_list for each detection
        # ... attach GPS / speed text as display_meta here
        l_frame = l_frame.next
    return Gst.PadProbeReturn.OK

osd.get_static_pad("sink").add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)

pipeline.set_state(Gst.State.PLAYING)
loop = GLib.MainLoop()
loop.run()
```

This is the skeleton; real apps add a bus message handler, error recovery, and clean shutdown.

> **`get_request_pad` vs `request_pad_simple`:** GStreamer deprecated `get_request_pad()` in 1.20 in favour of `request_pad_simple()`, so a linter may flag it. **Keep `get_request_pad()` for `nvstreammux`** — NVIDIA's current DeepStream docs still use it, and on some builds the `GstNvStreamMux` object has no `request_pad_simple` attribute (`AttributeError`), so "modernizing" the call breaks the pipeline. It emits a deprecation warning at most.

## Adding GPS / speed overlay

`nvdsosd` reads display metadata you attach to each frame in a probe. To overlay GPS coordinates and current speed:

```python
def add_gps_overlay(frame_meta, gps_state):
    display_meta = pyds.nvds_acquire_display_meta_from_pool(frame_meta.batch_meta)
    display_meta.num_labels = 1
    txt = display_meta.text_params[0]
    txt.display_text = f"{gps_state.lat:.6f}, {gps_state.lon:.6f}  {gps_state.speed_kmh:.0f} km/h"
    txt.x_offset, txt.y_offset = 20, 20
    txt.font_params.font_name = "Serif"
    txt.font_params.font_size = 18
    txt.font_params.font_color.set(1.0, 1.0, 1.0, 1.0)
    txt.set_bg_clr = 1
    txt.text_bg_clr.set(0.0, 0.0, 0.0, 0.7)
    pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)
```

The `gps_state` is updated by a separate thread reading from `gpsd` (or the host's gpsd via TCP). Don't block in the probe — it runs on the streaming thread.

## Performance notes specific to Orin Nano

- One PGIE (e.g., YOLOv8s @ 640×640 FP16) + tracker + OSD on a single 1080p30 source typically lands at ~25–30 fps realtime on Orin Nano MAXN_SUPER. The user's primary stream is 1080p**60** — at full rate this is borderline; expect to either drop the inference rate (via `nvinfer interval=1` for ~30 fps effective) or use a smaller model (YOLOv8n).
- A second `nvinfer` for sign reading at `interval=11` (≈5 fps) costs very little — sign-detector-on-cropped-frame is cheap. Don't fear adding it.
- Software encoding 1080p30 H.264 (after `videorate` drops the recording branch from 60 to 30) costs ~2–3 CPU cores under `x264enc ultrafast`. Validate with `tegrastats` while the full pipeline runs.
- INT8 quantization (`network-mode=1` + INT8 calibration table) roughly doubles inference throughput; worth pursuing for the lane/obstacle PGIE once FP16 baseline is solid.
- Don't forget `model-engine-file` — DeepStream rebuilds the TensorRT engine on every launch if it can't find a cached `.engine` matching the config. First-launch builds take many minutes; pre-build once on the device and ship the `.engine` with your container/image.

See `models-and-tensorrt.md` for engine building details.
