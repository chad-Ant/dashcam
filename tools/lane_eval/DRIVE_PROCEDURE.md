# Lane test drive — operator sheet (lean cut, P5a)

Harness: `lane_drive` (one container per camera) + MKR raw-CAN logger + `sysmon.py` + tegrastats,
orchestrated by `tools/lane_eval/lane_run.sh`. Output: `/media/jetson/backup/lane_tests/run_NNN/`.
Never copy run data into the repo (it is public).

## Before the first drive (once per mount change)

1. **Build** in the container with plain `make` (the newest `bin/build_<timestamp>/` must contain `lane_drive`).
2. **Engine:** `models/culane_res18_fp16.engine` must be the native build (413 MB, sha256 `d048fbde…`).
3. **CSI crops (P0b):** park on a straight, marked road; take a still from each camera and check that the lane
   band (CSI default rows 544–864 of 1088 = `--crop 0.5,0.2059`) shows road, not the dashboard or hood. If it does
   not, choose crops that do and pass them per camera (see the start command below).
4. **Tests:** `lane_test --unit --strict --allow-xfail B5,B6,B8` and the `--gpu` tier pass on the build you drive.

## Pre-flight (parked, engine running)

```bash
sudo systemctl stop dashcam-v04          # root step: frees the UGREEN and the C3
cd ~/Documents/github_repos/dashcam
tools/lane_eval/lane_run.sh start --plan day          # or night; add --minutes N to bound the run
tools/lane_eval/lane_run.sh status                    # after ~2.5 min: both cameras RUNNING, lanes 1
```

`start` refuses (and says why) when: dashcam-v04 or any lane_/drive_ container is running, a camera or the MKR is
missing or held by another process, there is no build with `lane_drive`, the engine is missing, or the disk has
less than 20 GB per planned hour free.

Crops from the calibration: `--csi-crop T,B` / `--usb-crop T,B` (defaults CSI 0.5,0.2059 and UGREEN 0.5,0).
Other `lane_drive` options go after `--` and apply to both cameras.

The first 120 s run with lanes OFF (resource baseline). Do not start driving before `status` shows `lanes 1`.

## During the drive

- Nothing to do. `status` (from a laptop over ssh) shows per camera: state, processed results, effective Hz,
  source fps, restarts; system MemAvailable, GPU, storage rate.
- A camera that errors or stalls for 10 s restarts itself (5 tries, then `cam_lost`); a dead CAN logger is
  restarted by the supervisor into `can/partN`.

## Stop (parked)

```bash
tools/lane_eval/lane_run.sh stop                      # SIGTERM → each lane_drive finalises, exit 0
python3 tools/lane_eval/check_run.py /media/jetson/backup/lane_tests/run_NNN
sudo systemctl start dashcam-v04                      # root step, if wanted
```

Exit codes in `run_end.json`: 0 = finalised; 4 = finalised after a stop hang (data intact, `stop_hang` in
`summary.json`). Copy `run_NNN/{*.json,*/*.csv,can,sys}` off the Jetson after each drive.

## Run directory

| Path | Content |
|---|---|
| `run_config.json`, `run_end.json`, `events_host.csv` | run parameters (git sha, build, engine sha256), exit codes, host events |
| `<cam>/frames.csv` | every lane result: timestamps (CLOCK_MONOTONIC), stage timings, lanes, ego, offset, per-slot geometry |
| `<cam>/counters.csv` | 1 Hz: source fps, lane Hz, gated, errors, RSS / threads / fds |
| `<cam>/camera_ctrl.csv` | 1 Hz: UGREEN exposure / gain / focus / luma / night mode (CSI: unavailable) |
| `<cam>/clock.csv`, `<cam>/events.csv` | clock triples (CAN join); lifecycle and faults |
| `<cam>/review/*.mkv` | review footage, 60 s segments (CSI JPEG 5 fps, UGREEN MJPEG 10 fps) |
| `<cam>/run.json`, `<cam>/summary.json` | config + geometry; totals at the end |
| `can/` | `can_raw.log` (candump), `can_stats.csv`, `can_sync.csv`, `mkr_console.txt` — decode offline with `can_decode` |
| `sys/` | `sysmon.csv`, `proc.csv`, `tegrastats.txt`, container logs |

## Known limits (2026-10-25)

- UGREEN lanes use software JPEG decode; `--usb-decode hw` (nvv4l2decoder) is broken (13 Hz, growing lag).
- CSI gain / exposure are not logged (not readable via V4L2 under Argus).
- No live CAN gating, clip rings, tensor dumps, mount monitor or status display yet (P5b).
- The production rate gate default is still `kLegacy`; lane_drive uses `--rate-gate pts` (exactly 20 Hz).
