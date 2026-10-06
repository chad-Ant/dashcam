# Turning lab — passenger-operated, read-only CAN demo

Python 3 standard library only. Runs on the Orin host or in the existing dev
container. It reads the existing logger's **candump file**, never opens a serial
port, transmits CAN, changes a map, or changes firmware. Leave v0.4 running.
This is an experimental calibration tool, **not vehicle control or navigation**.

## What it estimates

This decoder is explicitly for the current Brio map: `0x1D0` wheel fields at
Motorola starts 7/8/25/42, length 15; `0x191` gear at 44|5. Do not use it with
another vehicle/map without updating and verifying the decoder. EPS assist
torque is unsigned effort, **not steering angle**, and is not used.

With `u=(RR+RL)/2`, `d=RR-RL-epsilon*u`:

- rear-centre speed = `u * wheel_kmh_per_count / 3.6`;
- yaw = `d * yaw_dps_per_count`, positive left;
- curvature = `yaw_radians_per_second / speed_m_per_second`;
- radius = `1/abs(curvature)`, at the **rear-axle centre**;
- equivalent front ROAD-wheel angle = `atan(wheelbase * curvature)`.

This is the no-slip, planar rear-axle/bicycle model, not either individual
Ackermann wheel angle and not steering-wheel rotation. See MathWorks'
[kinematic equations](https://www.mathworks.com/help/robotics/ug/mobile-robot-kinematics-equations.html)
and [rear-axle bicycle model](https://www.mathworks.com/help/robotics/ref/bicyclekinematics.html).
Independent speed/yaw references are needed: these three outputs are derived
from the same wheel signals and cannot validate each other.

Defaults follow the existing map (2.405 m wheelbase, 0.01 km/h/count,
0.1083 deg/s/count); these are **provisional**, not newly measured dimensions.
Verify wheelbase on this car. The demo requires both rear wheels >=300 counts,
fresh forward gear, speed <=25 km/h and |yaw| <=60 deg/s. Below the sensor
cutoff, reverse, missing gear or a corrupt/short frame means unavailable, not
zero. Near straight (|radius| >=500 m), radius is unavailable rather than an
unstable enormous number. No-slip cannot be verified from these inputs alone.

Time-based smoothing (default 0.15 s) applies to the display; calibration uses
raw wheel counts and time integrals, not smoothed curves. There is no automatic
straight-line learning from the estimator's own yaw (that would be circular).

## Try it without hardware

From this directory, choose output paths that do not yet exist:

```bash
python3 demo.py synthetic --out /home/jetson/drive_logs/turn_synthetic
python3 demo.py run --input /home/jetson/drive_logs/turn_synthetic/can_raw.log \
  --out /home/jetson/drive_logs/turn_replay --speed 1 --serve 8765
```

Open `http://127.0.0.1:8765` on the Orin. From a passenger laptop, forward it:
`ssh -L 8765:127.0.0.1:8765 jetson@JETSON_ADDRESS`, then open localhost there.
The server binds only to loopback and has read-only endpoints. No internet,
external assets or JavaScript packages are needed. Ctrl-C stops the demo.
`--speed 0` without `--serve` analyses a log as fast as possible. Sessions are
bounded to two hours unless `--seconds` is supplied (maximum 24 h).

## Live

Use the `can_raw.log` already produced by `mkr_stream_log.py` or the drive
session. **Never start a second reader of the MKR USB port.** If no logger is
running, start the existing logger per its parent README, with a new output
directory. Its exclusive-port refusal protects an existing reader. The C3
connection used by v0.4 is separate; do not stop the recorder for this demo.

```bash
python3 demo.py run --input /absolute/session/can_raw.log --follow \
  --out /home/jetson/drive_logs/turn_live_01 --serve 8765
```

`--follow` starts at EOF, so old frames are never presented as live. It refuses
log replacement/truncation; start a new session after rotation. The logger holds
frames 2 s and flushes about once per second: **typically 2–3 s display latency**.
After 1 s with no new wheel sample, or epoch age >5 s, displayed estimates are
hidden. Source time and lag are shown. **Do not time a physical gate crossing
from the delayed dashboard.** Keep raw logs and logger stats/sync files.

Outputs: `samples.csv`, `session.json` (profile/provenance), `result.json`.
Failed writes stop the demo visibly; the independent raw logger continues.
Invalid samples are retained with a reason. History is bounded to 300 points.
The tool does not rotate output; reserve disk space for the planned run.

## Reference drive

See [DRIVE_PROCEDURE.md](DRIVE_PROCEDURE.md). Use a flat, dry, empty private
area with permission, a passenger operating the equipment, and measured paths.
The driver watches the course, not the display. All manoeuvres are forward at
6–9 km/h; no sudden steering, braking, full-lock demand, or public-road test.

Afterwards annotate complete manoeuvres in JSON (`references.example.json`).
Use recorded gate video / independent lap timing, then align it to source time.
Snap start/end to actual wheel-sample times (nearest sample), not arrival time.
`segment` is the demo's segment column, not the raw logger's timeline number.
Check `can_sync.csv` for resets/clock steps and `can_stats.csv` for gaps/loss;
exclude affected runs. Set `clock_and_loss_checked=true` only after that check.
A source timestamp gap >1 s/backwards resets decoding; smaller clock steps
cannot reliably be distinguished from real time using candump alone.

```bash
python3 demo.py fit --samples /absolute/demo/samples.csv \
  --references /absolute/references.json --out /home/jetson/drive_logs/turn_fit_01
python3 demo.py run --input /absolute/session/can_raw.log \
  --profile /home/jetson/drive_logs/turn_fit_01/candidate.json \
  --out /home/jetson/drive_logs/turn_candidate_replay --speed 1 --serve 8765
```

Fit uses time-weighted straight runs for tyre mismatch, measured path length
for wheel speed scale, and whole left/right laps (±360°/lap) for yaw scale.
It requires separate straight/left/right runs in **both** fit and validation
sets and rejects overlapping windows, invalid samples, gaps >0.2 s and wrong
turn signs. It writes a **candidate only** plus per-run errors and baseline
comparisons. Inspect disagreement between individual fitted scales; median
aggregation is not proof against bad course measurements. Optional `--profile`
sets the fit's initial geometry/envelope; JSON keys are `Profile` fields in
`model.py`. Do not tune wheelbase to hide path errors.

## Verification

Optional local x/y/heading: build `lib/libdeadreckoning` from the repo root and
pass `--dr-library /absolute/path/to/libdeadreckoning.so` to `run`. This calls the
real C++ library, not another Python implementation; see its README for validity,
monotonic-time requirements, drift budgets and MKR code reuse. The display and
CSV label discontinuous trajectories explicitly. There is no production deployment.
For a known pose origin at a measured gate, replay just that labelled window
with `--start-time START_EPOCH --end-time END_EPOCH` (not with `--follow`).

```bash
python3 -m unittest discover -s ../tests -p test_turning.py -v
```

The synthetic generator supplies known mismatch/scales and independent
straight/circle geometry; it is labelled synthetic and is not road evidence.
The October 6 short hardware capture received FS/console data from production
DISCOVER firmware, with **zero CAN frames** on the quiet bench bus. That proves
USB/logging connectivity and the no-data path only, not live turning accuracy.
