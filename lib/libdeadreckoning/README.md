# libdeadreckoning — local rear-axle odometry on the Orin

Experimental C++17 CPU library, no ROS, CUDA, Arduino runtime, device access,
background threads or vehicle commands. Input: timestamped rear wheel counts,
fresh gear and validity. Output: local x/y/heading, signed speed/yaw, travelled
distance, equivalent front road-wheel angle and rear-centre radius.

**Not a global position source, steering sensor, or navigation/control system.**
No GNSS/IMU fusion is claimed. Tyre slip, effective tyre diameter, camber and
wheel calibration cause drift. Wheel speeds alone cannot establish an error
bound or a statistically calibrated covariance.

## Build

```bash
make -C lib/libdeadreckoning check
make -C lib/libdeadreckoning sanitize
# root aliases: make deadreckoning / make deadreckoning_test
```

Products: `build/libdeadreckoning.a`, `build/libdeadreckoning.so`,
`build/test_deadreckoning`. No production application is relinked, deployed or
restarted, and no timestamped dashcam build is created. Build inside the normal
Orin dev container for consumers running there; it also builds on the Orin host.

## API

```cpp
#include "libdeadreckoning.h"
using namespace dashcam::deadreckoning;
Config config;
// Set measured scales / mismatch from a reviewed calibration candidate.
Estimator estimator(config);
estimator.reset(0, 0, 0); // pose at the first accepted acquisition sample
WheelSample sample;
sample.monotonicNs = acquisitionNs; // common monotonic clock, NOT wall clock
sample.rearLeft = rl;
sample.rearRight = rr;
sample.gear = Gear::Forward;        // independently decoded, freshness checked
sample.valid = true;               // same wheel frame, transport/decode checked
State state = estimator.update(sample, monotonicNowNs);
// Publish status + continuous + calibrated + timestamp alongside pose.
// During silence call snapshot(monotonicNowNs), not a cached State.
```

Add this directory and `peripherals/mkr_zero/lib` to include paths; link the
static library. Serialize access: the estimator is single-owner. Copies of
`State` can be handed between threads. No allocation or IO in `update()`.
Invalid configuration/anchors throw `std::invalid_argument`; updates return
status. All angles in the C++/C API are radians unless named otherwise.

`c_api.h` is an optional C/ctypes bridge for the Python demo. Constructor errors
are contained; handles have explicit destroy/reset operations. This restricted
binding never asserts standstill or marks a calibration as trusted. The demo's
forward-only gate is stricter than the typed library's reverse-gear support.

## Reused MKR signal processing

The library compiles **the original** `SimpleMovingAverage.cpp` and includes
`SignalProcessingFunctions.h` / `MathFunctions.h`, not copied implementations.
The only changes to those headers select `<stdint.h>` for non-Arduino builds;
`ARDUINO` builds retain their original `<Arduino.h>` include.

- Original `rateLimit()` with delta scaled by acquisition dt.
- Original `saturate()` on already validated filter values. Impossible raw
  samples are **rejected first**, never clamped into plausible travel.
- Original SIZE_8 moving averages for diagnostic speed/yaw means. These are
  **not integrated**: sample-count smoothing adds cadence-dependent lag and
  loses the beginning of a manoeuvre during warmup.
- A new dt-aware exponential smoother is used for integration, initialised
  from the first accepted sample. Set `filterTauSec`, `speedSlewMps2` and
  `yawSlewRadps2` to zero for an unfiltered baseline. Filtering can bias motion
  transients; log `limited` and compare held-out trajectories.
- Pose integration/heading wrap use doubles. MKR float angular helpers are
  regression-tested but are not used to quantise accumulated pose.

Tyre mismatch uses the existing MKR ratio model:
`RR-RL - epsilon*(RR+RL)/2`. No self-calibration from predicted straightness.
Wheelbase/scales default to the existing Brio map, not new measurements.

## Geometry and failure behaviour

Rear-axle-centre local frame: +x initial-forward, +y initial-left, heading CCW in
[-π,π). No north/latitude/longitude or steering-wheel ratio is assumed. Reverse
gear reverses signed speed and yaw; equivalent steering retains the correct sign.
Each interval uses mean endpoint speed/yaw and an exact constant-curvature arc
(`sinc(dHeading/2)` form). This is exact for constant motion and approximate
for varying motion. Radius is not the outside front tyre's turning radius.

Use **acquisition timestamps**, not USB arrival times. `nowMonotonicNs` must
share the sample clock. Caller unwraps/maps the MKR clock, checks gear freshness
and resets on a new timeline. Do not feed stale bridge snapshots as new samples.
In DISCOVER, the C3's CAN fields are unavailable: decode the raw USB stream.

- Reject range violations, bad gear, implausible speeds/yaw, old/future/
  duplicate/backwards timestamps. Timestamps never rewind internally.
- Either rear wheel below 300 counts means unavailable, not standstill. Only
  independent `stationaryConfirmed` evidence plus both zeros allows a stop.
- A gap or direction change breaks integration; the next good sample primes
  the filter without extrapolating missing travel.
- `continuous=false` stays latched after unknown travel. Tracking can resume,
  but x/y then describe only known pieces, **not current position relative to
  the original anchor**. Only explicit reset at a known pose restores continuity.
- Default anchor budgets: 120 s / 1000 m. Exceeding either freezes integration
  with `DriftLimit` until reset. These are operational guardrails, **not accuracy
  guarantees**. Configure them to a measured validation envelope.
- Straight/near-straight radius is NaN; at confirmed standstill angle/radius
  are NaN. Never turn unavailable into zero for a downstream consumer.

## Live/replay demo integration

After building, from the repo root:

```bash
python3 peripherals/mkr_zero/tools/canstream/turning/demo.py run \
  --input /absolute/session/can_raw.log --follow \
  --out /home/jetson/drive_logs/turn_dr_live_01 --serve 8765 \
  --dr-library /home/jetson/Documents/github_repos/dashcam/lib/libdeadreckoning/build/libdeadreckoning.so
```

This calls the actual shared library and records `dr_*` columns. The C++ chain
has slew limits in addition to profile smoothing; the Python turning display
does not, so transient results may differ. Session metadata identifies the
library. Each new decoder segment explicitly starts a new (0,0,0) origin;
never join poses across segments. For a fresh anchor after a cutoff/drift limit,
restart/replay a selected manoeuvre from a known pose.

For an exactly labelled manoeuvre, omit `--follow` and add
`--start-time START_EPOCH --end-time END_EPOCH`. Gear/decode are warmed from
earlier frames, but the C++ pose begins at the first wheel sample inside the
window. Use the measured gate's nearest wheel timestamp as START, and keep the
window within the chosen drift budget (default 120 s).

The file adapter evaluates **recorded source time**, not delivery time; live
latency checks are separate. Candump timestamps are epoch-based. Large/backward
jumps are detected but small clock steps cannot be repaired: exclude every
clock-step/reset window from `can_sync.csv`. Production integration should feed
mapped monotonic acquisition times directly, not this file adapter. No logger
or production app is modified here.

Use the [reference drive](../../peripherals/mkr_zero/tools/canstream/turning/DRIVE_PROCEDURE.md)
for measured distance, circles and pose closure. Tests cover software behaviour,
**not road accuracy**. GNSS/IMU anchoring, uncertainty models and production app
integration remain future work. Do not substitute double-integrated noisy
accelerometer data for unknown wheel travel.
