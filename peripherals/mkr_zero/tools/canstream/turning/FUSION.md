# Turning EKF — experimental Orin-side sensor fusion

Opt-in extension to `demo.py`; Python standard library only, CPU only. The old
wheel-only display/calibration and optional C++ dead-reckoning path remain
unchanged. This filter estimates **equivalent bicycle front road-wheel angle**,
not steering-wheel rotation or each individual Ackermann tyre angle. It does
not control anything, change firmware, or deploy into v0.4.

## Rig configuration

The user confirmed the BNO055 **+Z points up**, and the GNSS antenna is centred
at the bottom of the windscreen. `fusion.brio_rs.example.json` uses `[0,0,1]`
and zero lateral antenna offset. With a level installation, positive gyro Z
should agree with a left turn; verify that on the measured course.

[Honda Vietnam's Brio specifications](https://www.honda.com.vn/o-to/san-pham/honda-brio/index.html)
list **RS front/rear track = 1475/1459 mm**, wheelbase **2405 mm**, with
185/55R15 tyres (the G's 1481/1465 mm track is different). These are nominal
stock-car geometry, not a measurement of this car or its effective tyre radii.

**Fill in `antenna_x_m` before using the example:** it is deliberately `null`
so loading an unmeasured template fails. Measure the horizontal distance from
the rear axle centre line forward to the antenna's ground projection, in metres.
The windscreen description does not determine that distance accurately. Do not
substitute the wheelbase. `antenna_y_m` is positive left; adjust if not centred.
The synthetic example's 2 m lever arm is test geometry, not a rig measurement.

## Run

From this directory, with new output paths:

```bash
python3 demo.py synthetic-fusion --out /home/jetson/drive_logs/ekf_synthetic
python3 demo.py run \
  --input /home/jetson/drive_logs/ekf_synthetic/can_raw.log \
  --telemetry /home/jetson/drive_logs/ekf_synthetic/telemetry.csv \
  --fusion-config /home/jetson/drive_logs/ekf_synthetic/fusion.json \
  --out /home/jetson/drive_logs/ekf_replay --speed 1 --serve 8765
```

For live use, supply the existing logger's `can_raw.log`, v0.4's growing
`telemetry_<start>.csv` **from the same session**, a copy of the completed rig
config, and `--follow`. Leave both original logging processes running; this
demo opens neither serial port. The dashboard remains localhost-only; use the
SSH tunnel described in `README.md`. Session timeouts and bounded line/history
sizes still apply. No automatic log-rotation switching: restart the demo with
the new file pair. Both sources start at EOF in follow mode.

No matching telemetry log means no GNSS/gyro fusion. The files must overlap;
catch-up is bounded at 2000 telemetry rows per wheel sample and buffered sensor
events at 4096. For a late replay window, trim/align the telemetry log first
while preserving its header. Raw logs are never overwritten.

The wheel-only cards stay visible for comparison; the second set displays EKF
angle, yaw and radius with model **1-sigma** uncertainty, accepted wheel count,
gyro bias, active sources, rejected observations and late-event count. CSV adds
`ekf_*` columns, including last gyro/course normalised innovation squared (NIS)
and the current wheel batch's maximum NIS for noise/gating diagnosis. Sensor NIS
values are held diagnostics, not new observations; consult source freshness.
`session.json` records the selected config and telemetry path.
The C++ trajectory, if enabled, still belongs to the original wheel-only DR
library; this change does **not** claim GNSS-corrected position.

## Model and safeguards

Four states: heading `psi` (east-zero, CCW), forward speed `v`, yaw rate `r`, and
gyro bias `b`. Constant-rate prediction advances `psi += r*dt`; speed, yaw and
bias have independent random-walk process noise, with the integrated heading/yaw
cross-covariance retained. Fixed 4×4 matrices, scalar innovation gates, analytic
Jacobians and Joseph-form covariance updates. Python allocates small matrices;
this is a bounded-memory demo, not an allocation-free production estimator.

For each tyre at vehicle coordinates `(a,y)` (forward, left):

- rear wheel: `h = v - r*y`;
- front wheel: `h = sqrt((v-r*y)^2 + (r*wheelbase)^2)`;
- gyro: `h = r+b`;
- GNSS antenna course: `h = psi + atan2(r*antenna_x, v-r*antenna_y)`.

GNSS north-zero clockwise course is converted to the filter's convention; angle
innovations wrap across 0/360°. The lever-arm term matters on small circles.
GNSS speed only gates course validity; it is not currently fused as another
measurement. The baseline's `yaw_dps_per_count` is **not** also fused: that would
double-count the same wheel information. EKF yaw uses physical track widths,
individual wheel speed scale and the profile's fixed rear-tyre mismatch
correction. No online tyre-scale learning or tuning to the filter's own output.

Each wheel is range/cutoff checked and innovation-gated independently; all four
contribute when consistent. Initialisation requires both rear wheels; subsequent
updates require at least two accepted wheels. This can reject an isolated slip,
but not detect every coherent/common-mode slip. High-rate wheel noise is inflated
relative to a 10 Hz effective rate to limit overconfidence from correlated samples.
No-slip planar motion is still an assumption: slope, bank, tyre compliance and
sideslip are not states, and gyro Z is not tilt-compensated on a banked road.

Gyro observations require finite axes, IMU-present, gyro calibration >=2, and no
fallback/data-gap/low-power/saturation flag. GNSS requires valid 2D/3D fix, >=5
satellites, valid UTC, finite course, and both reported/filter speed >=1.5 m/s.
Only the first telemetry row for a new UTC second can contribute a GNSS fix;
held 1 Hz fixes are not fused again at the 10 Hz bridge rate. Duplicate master
frames are ignored. Bad/missing observations never become zero readings.

Clock resets/large clock-vs-master discrepancies and CAN segment changes stop
fusion with an error so sessions can be split/re-aligned. Gaps >0.5 s reset the
filter; it cannot extrapolate across unknown travel. Late sensor events are
counted and dropped, never silently restamped. Wheel freshness is 0.25 s, gyro
0.35 s, GNSS 2.5 s; status becomes **degraded** when one source ages out. Without
accepted wheels, angle/yaw/radius outputs are unavailable. Reverse is not enabled
in this demo. Radius is hidden if near straight or yaw's 2-sigma interval includes
zero; the local delta-method radius uncertainty is otherwise displayed.

## Timing and accuracy limits

CAN is mapped by the raw logger; telemetry CSV provides **host arrival time**,
master frame time, and only whole-second GNSS UTC. It does not provide IMU sample
time, GNSS iTOW/fractional time, per-channel age, heading accuracy or velocity
accuracy. Thus this is an **approximate timestamp join**, not tightly synchronised
navigation. Config offsets are acquisition time minus arrival time (−2..0 s).
The join reorders pending telemetry events by adjusted time before each CAN wheel
sample. It does not rewind a completed update; late arrivals are rejected.

Default assumed timing uncertainty is 0.1 s gyro / 0.5 s GNSS. It inflates sensor
noise (gyro uses a nominal yaw-acceleration scale; GNSS uses current yaw rate).
These defaults, course noise and process noise need independent tuning. They
cannot correct systematic latency or model mismatch. Live view also retains the
raw logger's ~2–3 s delay. Do not time gate crossings from this display.

Noise matrices and reported uncertainties are **not calibrated error bounds**.
In particular, lack of independent GNSS direction leaves gyro bias partly
confounded with tyre/track errors. There is no covariance for uncertain geometry,
no slip state, no magnetometer, no GNSS position fusion, and no safety guarantee.

## Validation / next steps

1. Measure the antenna lever arm, check stock wheels/tyre pressures, confirm gyro
   turn sign on the existing private-area course with a passenger. Do not change
   firmware or mode just to run this tool.
2. Preserve CAN stats/sync and bridge-status logs; exclude loss, clock changes,
   IMU faults and weak fixes. Use independent gate video/lap timing and measured
   radius to score all three estimates, not their agreement with each other.
3. Fit wheel scale/mismatch on training runs using the existing calibration
   command; freeze them before tuning noise/latency on separate training runs.
   Evaluate untouched left/right/straight runs and report bias/RMSE and uncertainty
   coverage, including GNSS dropouts. The old `fit` command still fits wheels only.
4. Before production integration, extend logging with per-sensor acquisition
   timestamps, GNSS accuracy and fix sequence/iTOW; implement validated time
   mapping/latency compensation and mounted-frame calibration. Port the verified
   filter into C++ if production timing/allocation constraints require it.

Tests run with `python3 -m unittest discover -s ../tests -p test_fusion.py -v`.
Synthetic truth includes gyro bias/noise, independent front/rear geometry, GNSS
outage, one-wheel slip and a gyro spike. Passing it is software verification,
**not evidence of live-drive accuracy**.

Algorithm references: [MathWorks EKF formulation](https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html),
[NovAtel's distinction between antenna course and body heading](https://docs.novatel.com/OEM7/Content/Logs/BESTVEL.htm).

## Verification on 2026-10-07

- 27 fusion tests pass on the Orin host and in `l4t-ml-gpio:latest` (read-only
  repository mount, no devices or network). Analytic Jacobians checked against
  finite differences; long noisy-run covariance checked by Cholesky; duplicate,
  stale, out-of-order, corrupt, low-speed and slip cases exercised. HTTP page and
  JSON endpoints checked; no automated visual browser-layout test.
- Original turning suite: 29 tests pass. C++ DR remains unchanged: 55 checks pass.
- Full canstream suite: 134 tests, no failures, one intentional abstract-fixture
  skip (189 s). That run preceded the final HTTP and regressing-UTC additions;
  both are included in the final 27-test focused reruns above.
- Deterministic 60 s synthetic replay, scored after the first 10 s: yaw RMSE
  **1.3721 deg/s raw wheels, 0.3777 deg/s existing smoothed wheels, 0.4651 deg/s
  EKF**. Final gyro bias **1.7661 deg/s** versus injected 1.8; 52 rejected
  observations, zero late events. The EKF does **not** outperform the existing
  smoother in this clean-wheel, slow-changing scenario. Do not equate complexity
  with improved accuracy; noise/latency tuning and independent held-out driving
  evidence remain necessary. Sensor outages correctly change source/status.
- Evidence: `/home/jetson/drive_logs/turning_ekf_check.NAQNaN/` (`source/`,
  `replay/`, timed `http_replay/`). Synthetic outputs only; not committed.
- Existing October 3 CSV field parser check: first 2000 rows expose 2000 usable
  gyro observations and no usable course in that initial window. Strict timing
  rejects the first boundary: host delta 0.078 s, master delta 5.211 s. This
  demonstrates a timing inconsistency, not its root cause; no real-data fusion
  accuracy result is claimed, and the original file is untouched.
- No live drive, flash, service control or production deployment was performed
  for this extension. The measured antenna fore/aft offset is still needed
  before running the rig configuration. Implementation and verification are
  also recorded in the October 7 entry of `HANDOVER.md`.
