# Event-Triggered Recording

The dashcam recording requirement is: continuously record the front camera, but **only persist clips around interesting events** (hard accel/decel detected from accelerometer, optionally collision, optionally manual button). Specifically, save 30 s before + 30 s after each event.

This is exactly the "smart recording" problem. Two implementations:

## Option 1: DeepStream's built-in Smart Record (`NvDsSRContext`)

DeepStream ships a Smart Record API specifically for this. It maintains a circular cache of encoded video and dumps a clip on demand.

C API: `NvDsSRCreate`, `NvDsSRStart`, `NvDsSRStop`, `NvDsSRDestroy`. Python bindings: `pyds.NvDsSRContext`.

Conceptually:
```
[source] → encoder → smart_record_bin (caches last N seconds)
                            ↓
                    on trigger: dump cached + record N more → file
```

The catch on Orin Nano: Smart Record assumes you have a working encoder feeding it. With no NVENC, your "encoder" is `x264enc`, which has to keep up at line rate. Smart Record itself doesn't care which encoder, but make sure `x264enc tune=zerolatency speed-preset=ultrafast` keeps up before relying on this.

Sketch (Python):
```python
import pyds
sr_config = pyds.NvDsSRInitParams()
sr_config.containerType = pyds.NvDsSRContainerType.CONTAINER_MP4
sr_config.cacheSize = 60      # seconds of pre-event cache (we want 30 + headroom)
sr_config.dirpath = "/data/events"
sr_config.fileNamePrefix = "event"
sr_ctx = pyds.NvDsSRContext()
pyds.NvDsSRCreate(sr_ctx, sr_config)
# attach sr_ctx.recordbin into the pipeline as a sink branch
# ... when the IMU thread detects a hard event:
sr_params = pyds.NvDsSRStart(sr_ctx, 30, 30, None)   # 30s pre, 30s post
```

Reference: NVIDIA's `deepstream-testsr` Python sample is the closest example.

## Option 2: Pure GStreamer ring buffer

If you're not already on DeepStream, or you want to record a camera that isn't going through DeepStream (e.g., the driver-monitoring USB cam), use `splitmuxsink` with a circular file scheme + a memory ring.

Approach: continuously encode and write a sliding window of small files (e.g., 5-second segments). On event, copy the relevant segments out of the rotating directory.

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1' ! \
  nvvidconv ! 'video/x-raw,format=I420,framerate=30/1' ! \
  videorate ! 'video/x-raw,format=I420,framerate=30/1' ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000 ! \
  h264parse ! \
  splitmuxsink location=/data/ring/seg_%06d.mp4 \
    max-size-time=5000000000   # 5 seconds in nanoseconds
```

Note the `videorate` element dropping 60 → 30 fps before the encoder — capture at 60 (so inference branches see motion-rich frames) but only software-encode at 30 to keep `x264enc` under budget. In a real DeepStream pipeline this `videorate` lives on its own `tee` branch; see `deepstream-pipelines.md` for the full layout.

A small daemon trims `/data/ring/` to keep only the most recent ~12 segments (=60 s). On event, atomically move the relevant segments into `/data/events/<id>/` and continue recording into the ring.

Tradeoff vs Smart Record:
- ✅ Works without DeepStream.
- ✅ Easier to debug (just files on disk).
- ✅ Multiple cameras can each have their own ring.
- ❌ Cuts on segment boundaries, not at event timestamp — your 30 s pre might be 25–35 s.
- ❌ More moving parts (ring trimmer daemon, event mover).

## IMU / accelerometer integration

The dashcam needs an accelerometer to detect hard braking / acceleration. Options:

| Source | How |
|---|---|
| OBD-II (vehicle bus) | ELM327 USB or CAN dongle → speed deltas. Detects hard braking via dv/dt. Most accurate for vehicle dynamics. |
| External IMU (MPU-6050, BMI088) | I²C or SPI to Jetson 40-pin header. ~$5–20. Gives 3-axis accel + gyro at 100+ Hz. |
| GPS speed (already needed for overlay) | Lower rate (1–10 Hz), noisy at low speeds, but good enough for braking detection. |

Recommended: external IMU on I²C (e.g., BMI088 at ~50 Hz) for fast trigger + GPS for ground truth.

Threshold heuristic for "hard event" (tune empirically):
```python
HARD_DECEL_THRESHOLD_G = 0.5   # tunable
HARD_ACCEL_THRESHOLD_G = 0.4
WINDOW_MS = 300                 # sustained, not single-sample spikes

# Filter accelerometer with low-pass (e.g., 5 Hz cutoff) to avoid road bump triggers
# Project onto vehicle longitudinal axis (calibrate orientation once)
# Trigger when |longitudinal_accel| > threshold sustained for WINDOW_MS
```

Don't trigger on raw single samples — vibration spikes will give constant false events.

## GPS integration

Use `gpsd` on the host, expose via TCP, connect from inside the container.

Host setup:
```bash
sudo apt install gpsd gpsd-clients
sudo systemctl edit gpsd.socket
# Override ListenStream=/var/run/gpsd.sock with a TCP socket:
# ListenStream=0.0.0.0:2947
sudo systemctl restart gpsd.socket gpsd.service
```

In container, Python:
```python
from gps import gps, WATCH_ENABLE, WATCH_NEWSTYLE
session = gps(host="host.docker.internal", port="2947", mode=WATCH_ENABLE | WATCH_NEWSTYLE)
for report in session:
    if report['class'] == 'TPV':
        lat = getattr(report, 'lat', None)
        lon = getattr(report, 'lon', None)
        speed_mps = getattr(report, 'speed', None)
        speed_kmh = speed_mps * 3.6 if speed_mps is not None else None
        # publish to shared state read by overlay probe
```

`host.docker.internal` requires `--add-host=host.docker.internal:host-gateway` on `docker run`, or use the host's LAN IP directly.

## Event clip metadata

When persisting an event clip, also persist a metadata JSON:
```json
{
  "event_id": "20260430-153012-Z",
  "trigger": "hard_decel",
  "trigger_value_g": 0.74,
  "video_files": ["video_front.mp4", "video_driver.mp4"],
  "duration_s": 60,
  "pre_event_s": 30,
  "post_event_s": 30,
  "gps_track": "gps.csv",
  "imu_track": "imu.csv",
  "detections": [
    {"t": -1.2, "class": "car", "bbox": [820, 410, 1120, 660], "track_id": 17},
    ...
  ],
  "speed_at_event_kmh": 73.4,
  "speed_limit_at_event_kmh": 50
}
```

This makes events queryable later without re-running inference — important since the user is fully offline and may want to surface "events where I exceeded the speed limit" or similar.

## Storage budgeting (Orin Nano + 1.2 TB)

Architecture: 1 IMX296 CSI recording at 1080p30 (after the 60→30 videorate drop). The 3 USB cameras (stereo @ 360p10 + driver cam @ 360p10) are *not* continuously recorded — they're inputs to inference; only their inference outputs are logged unless an event triggers.

At 4 Mbps continuous record of the 1080p30 front stream:
- ~1.8 GB/hour
- ~14 GB / 8-hour driving day
- Fills 1.2 TB in ~85 days of continuous recording

Smart-record-only (no continuous record) is much cheaper:
- ~30 MB per 60 s event clip (front camera only)
- 1.2 TB = ~40,000 events stored

If you want to also persist USB-camera footage during events: a 60 s event clip from each USB cam at 360p10 MJPEG-recoded is tiny (~5 MB). Bundle them into the event directory.

Recommend: smart record for the front camera, **and** keep a low-bitrate continuous "context loop" (e.g., 720p15 @ 1.5 Mbps, ~675 MB/hour, ~5.4 GB / 8h) so you have evidence even for non-trigger events. Rotate the context loop to overwrite anything older than ~7 days.

## Honest caveats

1. Software encoding 1080p30 + running inference + reading IMU + writing files: this is close to the Orin Nano budget. **Validate end-to-end performance with `tegrastats` before committing to a recording strategy.** If `x264enc` falls behind, you'll get growing latency and eventually frame drops — Smart Record's pre-event buffer becomes lies.
2. NVMe storage is required for sustained writes. eMMC / SD card will throttle or wear out quickly with continuous video.
3. For evidentiary use, consider also writing a hash chain or signed metadata so clips are tamper-evident.
