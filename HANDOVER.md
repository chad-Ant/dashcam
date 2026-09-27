# Handover — branch `camera`, 2026-09-27 (evening)

## Where things stand

| Item | Commit | Status | Still needs |
|---|---|---|---|
| High-G settings | `ee952e1` | The threshold is 2 g in every IMU mode (`IMU_HIGHG_THRESHOLD_MG`, converted per accelerometer range). AMG used to write fusion's byte, which is 8 g at ±16 g. Host tests pass, and IMUPLUS is flashed on the Jetson. | a drive: `hg=` / `hgrej=` |
| CAN frame loss | `ee952e1` | Overruns are counted (`ovf=` after `rx=` in SNIFF) so the loss can be sized. Nothing else has changed yet. | a drive: `ovf=` vs `rx=` |
| Autofocus | `0d733b5` | v0.4 `<Recording><FocusMode>fixed</FocusMode>` + `<FocusAbsolute>` holds the lens. The default, `camera`, leaves it untouched. The UGREEN has `focus_automatic_continuous` and `focus_absolute` (0–1023). | the value for the road (below) |
| I2C hang | `8ebb9be` | `vendor/Wire` bounds every bus wait at 25 ms. On expiry it resets the SERCOM, and `i2cBusBegin()` recovers the bus. Proven on the rig: BusFaultInjection 10/10, twice. The control run hung until the watchdog. | — |
| AMG mode | `7882f4e` | AMG came up broken: it restored the IMUPLUS calibration profile at ±16 g, which raises SYS_ERR 0x09. The profile is now restored, and saved, in fusion modes only. Verified on the rig. | — |
| **I2C bus errors** | `1e93aeb` | A glitch that the controller sees as a START or STOP mid-transfer now fails that transfer, resets the controller, counts it (`i2cerr=`) and recovers the bus. Before, it returned a garbage byte as data, or left every later transfer refused while the lines read idle. **Not run on hardware yet.** | **the rig proof below** |
| Build scripts | `c232b6d` | `check_core.sh` holds the arduino:samd 1.8.14 check. `install_map.sh` and the BusFaultInjection script now apply it too. | — |
| liblog | `07a61d4` | `liblog.h` includes `<cstdint>`, so the tree builds with GCC 13 without `-include cstdint`. | — |

## Next build: prove the bus-error fix on the rig

**Why.** The fault on this harness is contact that flickers, not a wedge. The 2026-09-26 corrupt reads came from
breadboard contacts. On the SAMD21, a flicker that lands as a START or STOP mid-transfer is a **bus error**:

- The controller sets BUSERR, ARBLOST and MB, never SB (SB means a byte received cleanly).
- It then gives up the bus.

`8ebb9be` copied the core's read, which accepts MB as a byte. It also kept the core's early "another master holds
the bus" refusal. Two consequences follow from the datasheet's bus-state rules:

- A glitch on a read's last byte returned a garbage byte at the full count.
- A START with no STOP left the controller BUSY. Every later transfer was refused, both lines read idle, and
  nothing was counted, so `i2cBusBegin()` never recovered. The IMU and GNSS would stay down until a power cycle.

`1e93aeb` treats all of these as bus errors (see `vendor/Wire/README.md`, "Bus errors"). It compiles for both
variants with no warnings in project code, but no glitch has been injected yet.

**Run it.** Build the helper instrumented and flash it, from the host:

```bash
docker run --rm --privileged -v /dev:/dev -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
  /user/dashcam/peripherals/mkr_zero/helper_scripts/BusFaultInjection/build_and_upload.sh auto
```

Then send `a` on the console.

- **Expected: 15 of 15.** T0–T6b as on 2026-09-27, plus C7, T7, T8, T9 and T9b.
- **T1b** should now report `endTransmission=4 ... bus errors +1`, not `2`. A slave holding SDA low makes the address
  phase lose arbitration, which is now a bus error. It still passes (the check is only `rc != 0`).
- **C7** (stock path) must show the bus Ready 3/3 and CHIP_ID failing 3/3. That is the dead bus. If it fails, the
  stock driver recovered by itself, and that premise was wrong on this silicon. The fix still holds; it just
  mattered less.
- **T7/T8:** one bus error, fast, then CHIP_ID `0xA0` and the GNSS answer, with no manual recovery.
- **T9/T9b:** SDA is pulled low under a high SCL mid-read. Expect 0 bytes and one bus error, then recovery. If it
  fails with `got 64 bytes, bus errors +0`, the controller did not flag that glitch, which is a hardware limit and
  not something Wire can see.
- **`SKIP` lines:** a test whose premise did not reproduce. For example, T7 did not reach BUSY, or T9 caught no high
  SCL with SDA released. Say which, and rerun once.

After the run, re-flash production (`peripherals/mkr_zero/build_and_upload.sh auto`, plus `amg` if used). At rest
the console should read `i2cto=0 i2cerr=0`.

**Also.** Fix the harness wiring (solder or crimp the I2C lines). The firmware now contains both a wedge and a
glitch; the wiring removes the cause. `i2cerr=` on a drive before and after the rework measures that directly.

## After flashing: one drive answers the rest

- **High-G:** `hg=` counts confirmed events, `hgrej=` rejected evidence. Tune `IMU_HIGHG_THRESHOLD_MG` /
  `IMU_HIGHG_DURATION_LSB` from that data, not before.
- **CAN:** compare how fast `ovf=` climbs with `rx=matched/total`.
  - A few overruns per minute on 50–100 Hz signals is harmless.
  - Steady climbing means the loop drains too late. The fix is to service `tickCANSniff()` between the long steps
    of `loop()` (GNSS poll, SD writes).
  - The MCP2515 INT line is not wired, and RX rollover (BUKT) is already on.
- **I2C:** `i2cto=` counts wedges (a slave holding SCL). `i2cerr=` counts glitches (contacts). Each one used to be
  a watchdog reset, a garbage byte or a dead bus. Now each costs one failed transfer and a recovery.
- **Focus:** while recording, try
  `v4l2-ctl -d /dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0 -c focus_automatic_continuous=0 -c focus_absolute=N`
  with a few N across 0–1023 (road = far). Check plates and signs in the footage, then set `FocusMode` /
  `FocusAbsolute` in `/user/output/configs/dashcam.xml`. The app snaps and clamps N to what the camera offers,
  and warns when it does.
- **Deploying v0.4:** the service runs the newest build it finds.
  - Rebuild (`docker_dev/launchcode_v0_4.sh --rebuild`, or `make dashcam_v0_4` in the container), then restart
    `dashcam-v04`.
  - The deployed `dashcam.xml` still carries `<Encoder>` and `WarmupFrames` (removed in `409c87a`). Expect one
    "obsolete settings ignored" warning at start until those lines are deleted.

**Also open, on the car:** the parked block test for the engine-on IMU faults, and the night plate-exposure test.

## Open, deliberately not done

- **v0.3 overlay:** it keeps its own copy of the bridge→overlay rules. On a fix without altitude it shows the last
  altitude, or the 52.3 m placeholder. v0.4 uses `src/bridge_overlay.h`. Skipped on request.
- **Unused libraries:** `libstereocam` and `libsigndetector` are in the tree, but no target builds them.

## Cloud sessions can compile the MKR firmware now

The sandbox still blocks `downloads.arduino.cc`, but GitHub clones, GitHub release downloads and apt all work.
`1e93aeb` was compile-checked from these parts:

- **Toolchain:** apt `gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib`, i.e. GCC 13.2.
  The Jetson builds with 7.2.1, so image sizes differ (130,676 B here); this is a compile check, not the flashed
  binary.
- **Core:** `arduino/ArduinoCore-samd` at tag `1.8.14`, with `api` symlinked from `arduino/ArduinoCore-API`.
- **CMSIS:** `ARM-software/CMSIS_5` (sparse checkout of `CMSIS/Core/Include`, served as `CMSIS/Include`), plus
  `arduino/ArduinoModule-CMSIS-Atmel`. An empty `libarm_cortexM0l_math.a` satisfies the link.
- **ctags:** `arduino/ctags` built from source, after renaming its `__unused__` / `__printf__` macros, which clash
  with glibc.
- **arduino-cli:** from its GitHub release.
- **Libraries:** the eight from `docker_dev/Dockerfile`, each at its tag.
- **Build:** `arduino-cli compile` with `ARDUINO_DIRECTORIES_USER` pointing at a sketchbook whose
  `hardware/arduino/samd` is the core, and `--build-property runtime.tools.{arm-none-eabi-gcc-7-2017q4,CMSIS-4.5.0,CMSIS-Atmel-1.2.0,ctags}.path=...`.
  A wrapper that adds those flags to `compile` lets `build_and_upload.sh` itself run unmodified.

## Tests

```bash
make -C peripherals/mkr_zero/tests/host check        # switch 61, imu 540, can_probe 98, bno_init 602
make -C peripherals/mkr_zero/tests/host mutations    # 47 mutants, all must be caught
make -C peripherals/esp32-c3/tests/host check        # bridge 79
make                                                 # every target, inside the l4t-ml-gpio container
B=$(ls -td bin/build_* | head -1)                    # the build just made; run from the repo root
$B/dashcam_v0_4 --self-test && $B/config_test && $B/bridge_overlay_test
$B/record_test                                       # Part A needs GStreamer base/good/bad/ugly plugins; Part B a camera
```

The Wire changes have no host test: they are register-level SAMD21 code, proven only on the rig
(BusFaultInjection).
