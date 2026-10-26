# Lane test drive — operator sheet (P5a + P5b)

Harness: `lane_drive` (one container per camera) + MKR raw-CAN logger + `can_live.py` (live CAN state and
clip triggers) + `lane_supervisor.py` (restarts, degrade ladder, `status.json`) + `lane_status` (CYD / ES3C28P
pages and alert beeps) + `sysmon.py` + tegrastats, orchestrated by `tools/lane_eval/lane_run.sh`.
Output: `/media/jetson/backup/lane_tests/run_NNN/`. Never copy run data into the repo (it is public).

## Kit

Laptop on the phone hotspot (ssh to the Jetson), the status display where the driver can see it, a phone with
an accurate clock, a phone torch (sync check at night).

## Before the first drive (once per mount change): calibration, P0b

1. **Build** in the container with plain `make` (the newest `bin/build_<timestamp>/` must contain `lane_drive`
   and `lane_status`).
2. **Engine:** `models/culane_res18_fp16.engine` must be the native build (413 MB, sha256 `d048fbde…`).
3. **Tests:** `lane_test --unit --strict --allow-xfail B5,B6,B8` and the `--gpu` tier pass on the build you drive.
4. **Calibrate** parked on a straight, marked road, engine running:

   ```bash
   sudo systemctl stop dashcam-v04                       # root step: frees the UGREEN and the C3
   tools/lane_eval/lane_run.sh calibrate                 # 40 s, both cameras; writes calib/CAL_NNN/
   ```

   `roi_calib.py` prints, per camera: hood row, roll, car-body fraction of the lane band, the model's vanishing
   point, a verdict (CSI: PASS / WARN / FAIL per plan 12.1; UGREEN: report) and a crop proposal (`--crop T,B`).
   Look at `CAL_NNN/<cam>_overlay.jpg` (green = lane band, yellow = reference row, red = hood edge, magenta = model
   lanes, cyan = VP). **FAIL: re-aim the mount** and calibrate again. If the hood edge or the VP was not found
   (night, no lanes in view), give them: `python3 tools/lane_eval/roi_calib.py CAL_DIR --hood-row csi=ROW --vp
   csi=X,Y` (in the container).
5. **Mount template:** `CAL_NNN/mount/` holds each camera's parked reference and, when a hood edge was found, its
   car-fixed region. Pass `--mount-ref CAL_NNN/mount` at start. A camera without a car-fixed region (the UGREEN's
   view is all road) gets a report-only mount monitor: no FAIL alert from it.

## Pre-flight (parked, engine running)

```bash
sudo systemctl stop dashcam-v04          # root step
cd ~/Documents/github_repos/dashcam
tools/lane_eval/lane_run.sh start --plan day --expect-can --mount-ref /media/jetson/backup/lane_tests/calib/CAL_NNN/mount \
    --csi-crop T,B --usb-crop T,B        # crops from the calibration; --plan night for the night drive
tools/lane_eval/lane_run.sh status       # after ~2.5 min: GREEN (ready), both cameras RUN 20Hz, CAN with a speed
```

`start` refuses (and says why) when: dashcam-v04 or any lane_/drive_ container is running, a camera, the MKR or
the display is missing or held by another process, there is no build with `lane_drive`, the engine is missing,
or the disk has less than 20 GB per planned hour free.

- The first 120 s run with lanes OFF (resource baseline, mount reference if none was given). Do not drive before
  `status` (or the display) shows **ALL GREEN** / `ready`.
- **Sync check (P13):** pass a hand (night: the torch) in front of both lenses 3 times, 2 s apart, after lanes
  are on. After the drive `python3 tools/lane_eval/sync_check.py RUN_DIR` reports the UGREEN − CSI offset; PASS
  = SD ≤ 33 ms.
- `--plan night`: CSI review at 3 fps q70. `--display none` runs without the status display; `--display DEV`
  names another board (default `auto` = the CYD's CH340 port).

## During the drive

Nothing to do; never touch the system while moving. A passenger may run `status` or mark a moment:

```bash
tools/lane_eval/lane_run.sh mark "lorry cut in"          # event + a kept clip on both cameras
```

The display shows GREEN / AMBER / RED with four lines (per camera: state, rate, lanes, ego, mount; CAN; the
first alert or memory / tj / GB/h / degrade level), and beeps on entering RED and every 60 s while RED.

| Alert (RED) | Rule |
|---|---|
| `<CAM> mount FAIL` | Pull over safely when convenient; check the mount. If it moved: re-calibrate or end the run |
| `<CAM> LOST` / `SILENT` | The camera failed 10 container restarts (or stopped reporting): end the run when parked |
| `CAN log not growing` | Continue (lanes still useful); when parked, decide on the MKR power-cycle recovery |
| `degrade N` (N ≥ 3), `tj > 92 C` | Continue; end early if it persists |

Automatic handling: a camera that errors or stalls for 10 s restarts in-process (UGREEN 5 tries, CSI 2), then
`lane_drive` exits 3 and the supervisor restarts its container (up to 10 times, ≥ 20 s apart, appending to the
same CSVs, lanes on at once, the run's mount reference kept); a dead or
silent CAN logger restarts into `can/partN`; the degrade ladder sheds load (1 clips off, 2 dumps off, 3 CSI review
off, 4 UGREEN lanes off, 5 UGREEN review off, 6 stop the run) on MemAvailable < 1.0 GB, tj > 92 °C or > 22 GB/h,
and stops at once on free disk < 5 GB or MemAvailable < 600 MB; a camera below 80 % of its rate for 30 s halves
the UGREEN lane rate.

Clips (full rate, about 20 s around the event) are kept on: stall, camera error, latency spike, error counter,
mount shift (faults); turn-stalk episodes at ≥ 30 km/h (lane changes) and operator marks; stop → go; flicker
bursts and a random 10 s every 10 min — within 1.5 GB/h per camera, faults first. Tensor dumps (outputs + input
band + the camera frame with the same PTS) every 5 s and on events.

## Stop (parked)

```bash
tools/lane_eval/lane_run.sh stop                      # SIGTERM → each lane_drive finalises, exit 0
python3 tools/lane_eval/check_run.py /media/jetson/backup/lane_tests/run_NNN
python3 tools/lane_eval/sync_check.py /media/jetson/backup/lane_tests/run_NNN
sudo systemctl start dashcam-v04                      # root step, if wanted
```

Exit codes in `run_end.json`: 0 = finalised; 3 = camera lost (finalised); 4 = finalised after a stop hang (data
intact, `stop_hang` in `summary.json`). Copy `run_NNN/{*.json,*.csv,*/*.csv,*/clips,*/dumps,can,sys,post}` off
the Jetson after each drive (`review/` too if space allows).

## Run directory

| Path | Content |
|---|---|
| `run_config.json`, `run_end.json`, `events_host.csv`, `status.json`, `triggers.csv` | run parameters (git sha, build, engine sha256), exit codes, host + supervisor events, live status, clip triggers |
| `<cam>/frames.csv` | every lane result: timestamps (CLOCK_MONOTONIC), stage timings, lanes, ego, offset, per-slot geometry, band luma, slot order, dump id, live CAN |
| `<cam>/counters.csv` | 1 Hz: source fps, lane Hz, gated, errors, RSS / threads / fds, clip ring, clips, dumps, degrade |
| `<cam>/camera_ctrl.csv`, `image_stats.csv`, `mount.csv` | UGREEN exposure / gain / focus / luma / night mode; 2 Hz luma / noise / motion; 1 Hz mount shift |
| `<cam>/clock.csv`, `<cam>/events.csv` | clock triples (CAN join); lifecycle, faults, triggers, mount, degrade |
| `<cam>/review/*.mkv`, `*.ass` | review footage, 60 s segments (CSI JPEG 5 fps, UGREEN MJPEG 10 fps); lane banner (burn in: `ffmpeg -i seg.mkv -vf ass=<cam>_p0.ass out.mp4`) |
| `<cam>/clips/`, `clips_index.csv` | full-rate event clips |
| `<cam>/dumps/`, `dumps_index.csv` | `dNNNNNN.bin` (LTD1: header + four output tensors), `_band.jpg`, `_frame.jpg` (exact PTS) |
| `<cam>/run.json`, `summary.json`, `status.json`, `mount_ref/` | config + geometry; totals at the end; live status; the mount reference used |
| `can/` | `can_raw.log` (candump), `can_stats.csv`, `can_sync.csv`, `mkr_console.txt`, `live_state.json`, `live_events.csv`, `partN/` after a logger restart — decode offline with `can_decode` |
| `sys/` | `sysmon.csv` (incl. VDD_IN and soctherm OC event counters), `proc.csv`, `tegrastats.txt`, supervisor / can_live / container logs |

## Bench tools

- `tools/lane_eval/balloon.py --stages 1000:180,600:120` — B3m memory pressure beside a run.
- `lane_drive --mount-replay --out DIR --files A.mkv,B.mkv --ref-file A.mkv --ref-at S --mount-roi y0,y1,x0,x1` —
  B9: the mount monitor over recorded footage (10-03: the CSI slip at 22:20:13 flagged 5 s later).
- `lane_run.sh start --idle-lead 60 --csi-args "--lanes-schedule off:120,on:120,off:120,on:300" --usb-args
  "--lanes-schedule off:240,on:420"` — B0 staged resource phases.

## Known limits (2026-10-26)

- UGREEN lanes use software JPEG decode; `--usb-decode hw` (nvv4l2decoder) is broken (13 Hz, growing lag).
- CSI gain / exposure are not logged (not readable via V4L2 under Argus).
- The IMX296 intermittently stops with an Argus sensor timeout (2026-10-26, 39 s – 11 min into runs, also with a
  bare nvarguscamerasrc under load); the restart path recovers it in ~30 s. Check the CSI flex and restart
  nvargus-daemon before a drive.
- The CYD's current firmware does not answer the display link (no HELLO): flash the navigation firmware
  (`peripherals/cyd`) or use the ES3C28P for the status display; the run carries on without it.
- No GNSS logger, VP-based mount check, or B1b neutrality test yet.
- The production rate gate default is still `kLegacy`; lane_drive uses `--rate-gate pts` (exactly 20 Hz).
