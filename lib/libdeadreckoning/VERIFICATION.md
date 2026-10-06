# Verification — 2026-10-06, Orin Nano

No production application deployment or firmware flash was performed. Existing
user edits in `HANDOVER.md` were preserved; a separate implementation/test summary
was added there when the user requested committing and pushing the work.

## Executed

- Native C++ build with `-Wall -Wextra -Wpedantic -Werror`: **55 checks, 0 failures**.
- Same source built/tested inside `l4t-ml-gpio:latest`, with no hardware devices:
  **55 checks, 0 failures**.
- AddressSanitizer + UndefinedBehaviorSanitizer: **55 checks, 0 failures**.
- GCC `-fanalyzer` syntax/static-analysis pass: no diagnostics.
- Turning demo: **29 tests**, including actual C++ shared-library binding.
- Full canstream regression suite: **109 tests, 0 failures, 1 intentional skip**
  (abstract fixture), about 186 s. Includes the new turning tests.
- MKR host suites: all pass (switch 61, IMU 540, CAN probe 98, BNO init 602,
  ring 2673, drain 178, telemetry 146, stream 1257, self-test 264 and 258,
  sketch policy 231). They do not by themselves exercise the shared maths header.
- Actual MKR compile-only in the pinned Arduino container: succeeded, 123236 B
  flash / 23924 B global RAM, unchanged from the handover. Compiler emits vendor
  core/library warnings (endian macro, RTCZero, Servo, GNSS). No upload command
  or device access was supplied to that build.
- Real October 3 baseline replay: 333679 raw frames. **13103 wheel frames** match
  the existing firmware-based C++ decoder exactly, all four wheels; 312 samples
  pass the demo's forward/motion gate. This is decoder parity, not reference
  turning-angle validation.
- Known synthetic calibration recovered the injected scales approximately
  (integer wheel quantisation remains). The candidate replay called the actual
  shared library, including a timed 10× browser-server replay and GET `/state`.
  A labelled single synthetic lap closed within 0.314 m / 1.81° with the fitted
  profile; this is **synthetic software evidence**, not an accuracy claim.
- HTTP page/state endpoints served successfully; no automated browser-rendering
  or visual layout test was available.

## Hardware check (limited)

A 20 s capture used the original `mkr_stream_log.py` in the existing image,
with only `/dev/ttyACM0` passed through. The demo followed its growing log on the
host. Production firmware reported DISCOVER, IMU/GNSS/C3 up, zero I2C errors.
Logger received 20 FS lines, 21 console lines, **0 CAN frames**, 0 malformed.
The bench CAN bus was quiet. This confirms connection and no-data handling only;
**live motion, wheel angle, turning radius and position accuracy remain untested**.

Local evidence (never publish raw vehicle data):
`/home/jetson/drive_logs/turning_demo_check.thjJHt/`, containing `live/`,
`live_demo/`, `synthetic/`, `replay/`, `calibrated/`, `baseline_demo/`,
`dr_replay/`, `lap_replay/`, container build and compile-only firmware outputs.
Some compiler outputs are container-root-owned; no existing files were removed.

## Current limits / next step

- No production fusion, GNSS anchoring, covariance model or calibrated error
  bounds. Tyre/slip effects require independent measured validation.
- Candump live view inherits 2–3 s logger delay and epoch-clock limitations.
  Production consumers should use mapped monotonic acquisition times directly.
- Run the measured private-area straight/left/right course with a passenger,
  freeze the fit, then evaluate held-out manoeuvres and pose closure.
- No profile or firmware is automatically installed after calibration.
- `dashcam-v04` was active initially, then a systemd stop completed cleanly at
  11:39:52 local time. This task issued no stop/restart; service was left inactive.
