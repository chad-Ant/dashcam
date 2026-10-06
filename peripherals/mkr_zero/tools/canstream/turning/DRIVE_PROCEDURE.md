# Repeatable reference drive — Brio turning / local odometry

## Safety and setup

Use a flat, dry, closed private area with permission, clear of traffic, people
and obstacles. Driver controls the car normally; a belted passenger handles the
logger. Stop if visibility, grip, clearance or equipment is uncertain. No one
stands in the vehicle's possible path. Place cameras/cones before moving and
keep wide clearance for the front overhang and outside front wheel.

Use normal manufacturer tyre pressures, equal loading and unchanged tyres for
all runs. Record pressures, tyre sizes, load, surface, weather and wheelbase.
Secure the CSI/IMU mounts first. Do not adjust a mount or use a laptop while driving.

1. Mark a straight 30 m timing section, with ample run-in and stopping room.
2. Mark a 10 m **rear-axle-centre** circle and, if space allows, a 15 m circle.
   Measure the path, do not assume a cone circle equals the vehicle path.
   A practical reference is the inner REAR tyre contact-centre path: measure its
   radius and add half the measured rear track to obtain rear-centre radius.
   Measure track between contact centres, not body edges. Do not substitute the
   manufacturer's kerb-to-kerb turning radius (outside FRONT wheel).
3. Mark one gate on each circle. A fixed camera safely outside the course films
   the rear axle crossing it; a passenger also times laps independently.
   Record the timing camera's alignment to the logger clock before and after.
   If alignment cannot be established, lap duration remains a check but a
   sample-level calibration is **not** ready. Dashboard clicks lag the car.
4. Start the raw logger, v0.4 telemetry/video if desired, and the demo. Wait for
   clock synchronisation to settle; inspect `can_sync.csv`. Do not alter system
   time mid-test. Keep `can_raw.log`, stats, sync, video and handwritten notes.
5. Stand still 20 s. Confirm “unavailable” at zero wheel speed (correct), no
   malformed input, and no increasing overflow/drop/host-loss counters.

## Runs (record each separately)

All targets below are gentle walking/jogging speed; do not chase an exact
speed at the expense of maintaining the marked path. Avoid acceleration/braking
inside timed sections. Build up gradually on the untimed run-in.

| Run | Role | Manoeuvre | Target | Measured reference |
|---|---|---|---|---|
| S1, S2 | Fit | 30 m straight, opposite travel directions on separate passes | 6 km/h | 30 m, heading change ~0° |
| L1, R1 | Fit | Same 10 m circle, left then right; 1 settling lap + 2 timed laps each | 6 km/h | ±720°, measured rear-centre path |
| S3, S4 | Validation | Repeat straight passes, **not reused from fit** | 9 km/h | 30 m, heading change ~0° |
| L2, R2 | Validation | Same circle, 1 settling lap + 2 timed laps each | 9 km/h | ±720°, same measured radius |
| L3, R3 | Extra validation | 15 m circle, same procedure, if space permits | 9 km/h | ±720°, second measured radius |
| C1 | Local-pose check | Two complete laps ending at the same gate and direction | 6 km/h | Near-zero displacement, net heading wraps to start |

Allow room to leave each circle and stop normally before changing direction.
Do not include approach, settling lap, departure, reverse, U-turn repositioning,
or braking in reference windows. Record one window per uninterrupted timed run.
If an inner wheel falls below the sensor cutoff, discard that run rather than
increasing speed on a course that is not safe for it.

## Predictable reference numbers

For measured rear-centre radius R, wheelbase L and independently timed whole
laps n taking T seconds:

- distance = 2πRn;
- mean speed = 2πRn/T;
- mean yaw = ±360n/T deg/s;
- equivalent road-wheel angle = ±atan(L/R).

For L=2.405 m, R=10 m: angle ≈13.52°. At exactly 6 km/h: one lap ≈37.70 s,
yaw ≈9.55°/s. At 9 km/h: ≈25.13 s and 14.32°/s. At R=15 m, angle ≈9.11°.
These are planning numbers, **not labels to force onto an imperfect drive**.
Use actual measured radius and complete-lap timing. The vehicle's speedometer
or CAN speed is not an independent reference for calibrating CAN speed.

## Annotate, fit, then validate

Copy `references.example.json` outside the repository. Match gate crossings to
source timestamps, round each boundary to the nearest actual wheel sample and
record timing uncertainty (aim <=0.1 s) and radius uncertainty (aim <=0.2 m).
Rows require exact sample timestamps to prevent silent trimming. Fit only S1/S2,
L1/R1. Freeze the resulting candidate before looking at validation errors.

Suggested **experimental** acceptance targets (not a safety certification):

- straight residual mean |yaw| <=0.3°/s in both directions;
- measured distance error <=3%; whole-lap yaw error <=3%; radius error <=5%;
- inferred angle error <=1° on held-out circles;
- left/right yaw-scale estimates within 5%; no wrong turn signs;
- repeated laps agree within the reference measurement uncertainty;
- no timestamps/reset/loss events inside the accepted windows.

For dead reckoning, additionally report closure position error / travelled
distance and heading closure error per lap. Compare raw, filtered and calibrated
runs from the **same** input. Do not call a visually closed track proof of
accuracy: yaw and distance errors can cancel. An external measured endpoint is
required. Any sensor gap makes pose continuity incomplete; do not bridge it by
assuming zero motion. Re-anchor explicitly at a known pose after a gap.

If validation fails, keep the raw data. Check path measurement, clock alignment,
tyre mismatch, low-speed cutoff, wheel slip and sensor map before adding model
complexity. Leave production firmware and defaults unchanged until repeat runs
support the result. GNSS course at these low speeds is only an auxiliary check;
an independently aligned IMU gyro can help later, but is not fused in this demo.
