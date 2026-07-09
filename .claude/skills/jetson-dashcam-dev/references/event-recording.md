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
pyds.NvDsSRCreate(sr_ctx, sr_config)      # fills sr_ctx; check the returned NvDsSRStatus
# attach sr_ctx.recordbin into the pipeline as a sink branch
# ... when the IMU thread detects a hard event:
session_id = pyds.NvDsSRSessionId()
status = pyds.NvDsSRStart(sr_ctx, session_id, 30, 30, None)  # see signature below
```

**Verified C API** (`gst-nvdssr.h` — the pyds bindings mirror this arg order):
```c
NvDsSRStatus NvDsSRCreate(NvDsSRContext **ctx, NvDsSRInitParams *params);
NvDsSRStatus NvDsSRStart (NvDsSRContext *ctx, NvDsSRSessionId *sessionId,
                          guint startTime, guint duration, gpointer userData);
NvDsSRStatus NvDsSRStop  (NvDsSRContext *ctx, NvDsSRSessionId sessionId);
```
`NvDsSRStart` takes **five** args — note the `sessionId` (returned, later passed to `NvDsSRStop`). `startTime` = seconds *before* now, `duration` = seconds *after* start; so 30/30 saves `t-30 … t+30` (≈60 s total). If `duration=0`, recording stops after `defaultDuration` from `NvDsSRCreate`. Confirm the exact pyds wrapper (especially how `sessionId` is passed) against the DS 7.1 **`deepstream-testsr`** Python sample — the key point is the 5-arg shape, not the 3-arg call an earlier draft used.

## Option 2: Pure GStreamer ring buffer

If you're not already on DeepStream, or you want to record a camera that isn't going through DeepStream (e.g., the driver-monitoring USB cam), use `splitmuxsink` with a circular file scheme + a memory ring.

Approach: continuously encode and write a sliding window of small files (e.g., 5-second segments). On event, copy the relevant segments out of the rotating directory.

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1456,height=1088,framerate=60/1' ! \
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

## Storage budgeting

**Storage decision:** footage goes on the **SD card** (`mmcblk0`, currently mounted at `/media/jetson/backup`, ~167 GB usable; a dedicated footage directory there is TBD). The ~1 TB system NVMe (`nvme0n1p1`, rootfs `/`) stays for OS/software; the second NVMe (`nvme1n1`) holds swap + ~103 GB free. Footage-on-SD is exactly what commercial dashcams do — the card choice and write pattern are what make it robust:

- **Throughput is a non-issue at these bitrates.** A single 4 Mbps H.264 stream is ~0.5 MB/s; even a modest Class-10/U1 card (~10 MB/s sustained) has 20× headroom. SD write-throttling only matters at 4K/high-Mbps, not here.
- **Endurance is the real constraint.** Continuous record writes ~14 GB/8 h-day (~43 GB/day 24/7); a consumer card's TBW is spent fast. Use a **high-endurance / surveillance-rated microSD** (SanDisk High Endurance, Samsung PRO Endurance) or an industrial card, and plan to monitor + replace it.
- **Power-loss safety.** A dashcam loses power abruptly at ignition-off, mid-write. Use a **crash-tolerant, flash-friendly filesystem** (f2fs, or ext4 with journaling), mount `noatime`, and let `splitmuxsink` finalize each segment atomically so only the in-progress segment can be lost — never the whole recording. Consider an ignition-sense GPIO → graceful-stop, or a supercap-backed clean shutdown.
- **Wear management.** Prefer larger segment files (fewer metadata writes), let the ring/aging job overwrite oldest segments, avoid tiny frequent `fsync`s. Watch `dmesg` for `mmc`/`I/O error` — the first sign of a dying card.
- **Isolation is good:** footage on the SD card (not rootfs) means a full or failed card can't wedge the OS on the NVMe.

Architecture: 1 IMX296 CSI recording at 1456×1088@30 (native res, after the 60→30 videorate drop). The 3 USB cameras (stereo @ 360p10 + driver cam @ 360p10) are *not* continuously recorded — they're inputs to inference; only their inference outputs are logged unless an event triggers.

At 4 Mbps continuous record of the 1456×1088@30 front stream (size is set by bitrate, not resolution):
- ~1.8 GB/hour
- ~14 GB / 8-hour driving day
- ~167 GB SD card ≈ ~93 h of recording ≈ ~11 days at 8 h/day (~4 days 24/7) — the card is far smaller than the NVMe, so aging/rotation or smart-record matters *more* here

Smart-record-only (no continuous record) is much cheaper:
- ~30 MB per 60 s event clip (front camera only)
- ~167 GB ≈ ~5,000+ events stored

If you want to also persist USB-camera footage during events: a 60 s event clip from each USB cam at 360p10 MJPEG-recoded is tiny (~5 MB). Bundle them into the event directory.

Recommend: smart record for the front camera, **and** keep a low-bitrate continuous "context loop" (e.g., 720p15 @ 1.5 Mbps, ~675 MB/hour, ~5.4 GB / 8h) so you have evidence even for non-trigger events. Rotate the context loop to overwrite anything older than ~7 days.

## Honest caveats

1. Software encoding 1080p30 + running inference + reading IMU + writing files: this is close to the Orin Nano budget. **Validate end-to-end performance with `tegrastats` before committing to a recording strategy.** If `x264enc` falls behind, you'll get growing latency and eventually frame drops — Smart Record's pre-event buffer becomes lies.
2. **SD-card endurance is the thing to watch** — footage lives on the SD card by design (see Storage budgeting above). The write *rate* is trivial (~0.5 MB/s at 4 Mbps); the write *volume* over time is what wears the card (~14 GB / 8 h-day). Use a high-endurance / surveillance-rated card, pair it with power-loss-safe segmenting (journaling FS or f2fs, short `splitmuxsink` segments, flush on boundaries), and watch `dmesg` for `mmc`/I-O errors as the first sign of a dying card. The system rootfs stays on NVMe, so a worn or corrupted footage card never affects boot.
3. For evidentiary use, consider also writing a hash chain or signed metadata so clips are tamper-evident.
