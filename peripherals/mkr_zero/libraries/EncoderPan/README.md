# EncoderPan — MKR Zero single-axis pan prototype

Separate library for the GRF-250 pan mount. The production sketch, telemetry
protocol, Orin tracker and stereo implementation are **not changed**. Nothing is
flashed. `PanBench` builds with motion disabled. Do not use this prototype for
vehicle control or unattended operation.

## Hardware choice and weather protection

Budget requested: under US$100 **each** for servo and encoder. Prices checked
2026-10-11, excluding shipping, tax, enclosure, power and mechanical work:

| Part | Choice | Budget / limitation |
|---|---|---|
| Positional PWM servo | [Hitec D646WP](https://www.hiteccs.com/actuators/product-details/D646WP) | Listed $64.99; IP67. Vendor requires international customers to contact sales. |
| Output-shaft encoder | [AS5048B adapter board](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/AS5048B-ADAPTERBOARD/3188613) and diametric magnet | Listed $19.40; distributor marks this board discontinued with remaining stock. Board is **not weatherproof**; requires housing. Check kit/magnet contents when ordering. |
| Longer encoder cable, if needed | Two [SparkFun QwiicBus EndPoints](https://www.sparkfun.com/sparkfun-qwiicbus-endpoint.html) | Listed $14.95 each; PCA9615 differential I2C. Stock is limited. Not Ethernet and not an RS-422 transceiver. |

The [servo datasheet](https://www.hiteccs.com/public/uploads/data_sheet/HCS_D646WP_Specsheetv2.2_102-1729815645.pdf)
specifies 4.8–7.4 V, up to 2.2 A stall, default ±60° for 900–2100 us,
and maximum 0.5° gear backlash. Start with a separate regulated **6 V supply
with at least 3 A capacity**, fused/current-limited and protected against vehicle
transients. Do not power the servo from the MKR's 3.3 V pin. Actual wind load,
bracket mass and vibration still determine whether this servo is adequate.

Mount the encoder magnet on the actual LiDAR output shaft, **after** gears,
couplers and belts. Keep its board stationary. Use a sealed housing with a
suitable nonmagnetic wall, sealed cable glands and strain relief; protect the
magnet from corrosion and measure the resulting gap/alignment. IP67 on the
servo does not rate the assembled mount, connectors or encoder, and is not a
pressure-wash/salt-spray qualification. A separate bearing should carry the
external bracket load.

The manufacturer's [AS5048A/B datasheet](https://cdn.sparkfun.com/assets/e/d/e/1/d/AS5048_DS000298_4-00.pdf)
defines B as the I2C variant, 14-bit resolution, 3.3 V operation, and approximately
±0.8° optimal-placement nonlinearity at room temperature. Resolution (~0.022°)
is **not accuracy**. At 100 m, 0.5° pointing error is about 0.87 m lateral error;
1° is about 1.75 m. This is a budget bench platform, not a demonstrated 100 m
vehicle-association solution. Measure actual beam placement before trusting it.

## Proposed wiring (standalone bench)

| MKR | Connection |
|---|---|
| A5 / D20 | PWM through a suitable unidirectional logic buffer to servo signal |
| A4 / D19 | Proposed active-HIGH external output-enable, with hardware pulldown |
| SDA / D11, SCL / D12 | AS5048B SDA/SCL; pull-ups to **3.3 V only** |
| 3.3 V, GND | Encoder board configured for 3.3 V, common logic reference |
| Separate 6 V rail | Servo power via fuse and independent cutoff; never MKR supply pin |

Check the adapter board schematic/jumpers: 3.3 V chip operation requires its
VDD3V and VDD5V pins connected appropriately. Strap A1/A2 low for address 0x40;
avoid 0x42 (production GNSS). No 5 V pull-ups on the shared bus. Keep raw I2C
short; for an outside mount use an engineered differential extension or move
the controller near the sensor in a protected enclosure. Differential I2C is
not galvanic isolation, and does not share the LiDAR RS-422 wires.

A4 was a second-servo reservation; this is a **proposed single-axis gate**, not
an installed circuit. A3/A6 remain reserved for GRF UART, Serial1 for C3, SPI
for CAN, TC3 for the CAN drain timer. Arduino Servo 1.3.0 uses **TC4** on SAMD;
do not introduce another TC4 owner. A GPIO software output can use A5 without
selecting its analog/PWM peripheral mux.

The output backend assumes an electrically default-OFF gate and checked signal
levels. Do not directly pull a MKR pin up to 5 V to control a buffer. Verify
servo PWM input compatibility or use a proper 3.3-to-5 V buffer. The bench gate
suppresses PWM; **missing PWM is not guaranteed torque-off** for a digital
servo. Independent servo power cutoff/E-stop and a hardware watchdog that drops
the gate are required for unattended use. A software timeout cannot turn off
an output while the CPU is hung. Test boot, reset, unplug and brownout behavior
with the servo disconnected first, then unloaded and current-limited.

## Library contract

- Integer millidegrees, signed rightward from calibrated forward; local uint32
  `micros()` timestamps, wrap-safe for normal short intervals. Reboot is a new
  clock/command session. A new controller/session must not trust queued commands.
- `Controller`: configure once, feed `observe`, explicitly `arm`, then `request`.
  Call `tick` every pass and immediately apply its output, **including disabled
  outputs**. Do not skip ticking when an encoder/command is absent.
- No runtime heap, recursion or busy waits in the new library. Rate limiting,
  saturation and an integer IIR mirror the existing MKR signal-processing
  patterns without the heap-owning moving average or floating-point PID.
- `calibrated=false` fails closed. All supplied geometry, pulse endpoints, speed,
  error and timing values are placeholders until independently measured.
  Map endpoints may be reversed. Requests outside travel are rejected, not
  silently clamped. Leave margin for encoder uncertainty at both travel limits.
- Limits apply to **commands**, not a guarantee of physical shaft speed.
  `encoderMaxSpeedMdegS` must bound physical movement (including disturbances),
  not merely equal `maxSpeedMdegS`. Optional proportional outer trim defaults
  off: the positional servo already has its own control loop. No integrator.
- Invalid/unreferenced encoder, implausible jump, travel violation, stale sample,
  duplicate/backward timestamp, expired/replayed command, scheduler gap and
  persistent following error latch off. Renewals cannot conceal expiry or a
  stall. `stop` never homes or centres. Clear explicitly, reacquire encoder,
  re-arm and send a newer command; sequence high-water survives stop/clear.
- `AS5048B` reads diagnostics before/after high-low-high angle acquisition
  (up to three bounded attempts across byte carry). Only defined diagnostic
  bits are checked; zero angle is valid. No OTP writes. Error includes calibrated
  encoder error and movement over acquisition time/sensor latency. I2C has no
  data CRC: plausible bus corruption, a slipping magnet and a stuck-but-valid
  encoder cannot all be detected by this prototype.
- `BoundedWireEncoder` requires the repo's patched Wire with bounded waits and
  the single-byte read fix. `maxReadUs` rejects slow results; it does not preempt
  an in-flight Wire call. Wire's per-event timeout can exceed one control period:
  after it returns, tick immediately faults off. Production must additionally
  enforce the hardware watchdog and shared I2C owner's recovery policy.
- `GatedPwm` wraps an injected hardware backend: validates pulses, flushes initial
  Servo centre pulses behind a closed gate, enables only during a low phase,
  and detaches on stop/fault. Check its return and call `outputFailed()` on
  failure. Backend failures latch until deliberate `begin()` reinitialization;
  a controller fault clear alone does not reset the output adapter.
  Arduino Servo itself still contains unbounded timer synchronization waits;
  initialization happens behind the disabled gate. This third-party limitation
  must be bounded or independently watchdog-protected before unattended use.

`Feedback.angleMdeg` is measured, never the target or `Servo.read()`. Settled means
raw samples remained near target for the configured dwell. Its interval ends
at the latest **observed** sample, never at a promised future time. The reported
error also bounds possible inter-sample motion; it can be large. Do not shrink
it just to pass the ranging association gate.

For the existing Orin `ServoPort`, convert millidegrees to radians by
`pi/180000`; map `timeUs`, `stableSinceUs`, `validUntilUs` to its monotonic clock,
shrinking the stable interval by clock uncertainty. Require valid, armed,
fault-free feedback and a covered LiDAR acquisition interval. **No production
wire adapter is supplied yet**: native USB is already owned by raw-CAN capture.
Integrate through a versioned shared transport, with source timestamps, boot
identity, sequence checks and acknowledgements; do not open a second USB reader
or stamp delayed commands as newly issued. The bench key interface is not this
transport. The legacy unused `ServoFunctions` helper remains untouched; do not
use its degree-valued attach limits as pulse-width calibration.

## Bench procedure and validation

1. Leave servo power disconnected; build `PanBench` unchanged. It only monitors
   the encoder. CSV: `pan,time_us,raw_counts,angle_mdeg,error_mdeg,healthy,settled,fault,output_enabled`.
   `Configuration` fault is expected until calibrated. There is no boot wait
   for USB; disconnected/slow USB logs may be dropped.
2. Mount and manually rotate the encoder. Check direction, wrap, zero, diagnostics
   and noise. Align the LiDAR forward independently; record `zeroCounts`. Measure
   angle error across the intended travel and under temperature/vibration.
3. With servo unloaded and an external cutoff ready, characterize safe pulse
   endpoints using a current-limited servo tester. Never find stops by driving
   into them. Measure speed, following error and backlash in both directions.
4. Put measured limits in `panConfig()` / `encoderConfig()`. Only then set both
   interlocks true, verify gate behavior on an oscilloscope and connect servo.
   The sketch accepts `a` to arm; `1/2/3` for -10/0/+10° for **250 ms**; `s` to
   stop; `x` to clear a controller fault. Adjust these test angles if outside
   measured travel. No automatic sweep/renewal. Expiry requires clear/re-arm.
   Start without the LiDAR mounted; do not feed queued automation into this
   manual interface.
5. Disconnect encoder, stop commands, reboot and disconnect USB. Simulate
   following error with a guarded, current-limited fixture, not by restraining
   the shaft by hand. Verify timeout, fault latch, power cutoff and no automatic recovery. Repeat
   under CAN/GNSS/IMU load before considering production integration.
6. Mount LiDAR; calibrate camera-to-pan/beam alignment and time synchronization,
   then measure repeatability on fixed targets before moving-target tests.

Run host tests from repository root:

```sh
bash peripherals/mkr_zero/libraries/EncoderPan/tests/run.sh
```

The runner uses strict warnings, GCC analyzer, ASan and UBSan. Compile only
(never uploads), using the existing container's SAMD 1.8.14 / Servo 1.3.0:

```sh
docker run --rm --runtime=runc --network=none --entrypoint bash \
  -v "$PWD:/repo:ro" l4t-ml-gpio:latest -lc '
  arduino-cli compile --fqbn arduino:samd:mkrzero --warnings all \
    --build-path /tmp/pan-build \
    --library /repo/peripherals/mkr_zero/vendor/Wire \
    --library /repo/peripherals/mkr_zero/libraries/EncoderPan \
    /repo/peripherals/mkr_zero/libraries/EncoderPan/examples/PanBench'
```

Hardware movement, electrical behavior, timing under load and weather resistance
remain unverified. Passing host tests or compiling is not a live hardware test.
