# Handover — branch `camera`, 2026-09-27

## Where things stand

The last three commits on `camera` fix the three smaller items from the previous handover
("the others are the autofocus setting, the High-G settings, and the CAN frame loss").
Host tests pass for every change. None of it has run on the car yet.

| Item | Commit | What changed | Verified here | Still needs |
|---|---|---|---|---|
| High-G settings | `ee952e1` | The threshold is now 2 g in every IMU mode (`IMU_HIGHG_THRESHOLD_MG`, converted per accelerometer range). AMG used to write the same byte as fusion, which is **8 g** at its ±16 g range. | `bno_init_tests` (586), mutants | SAMD build + flash; one drive (below) |
| CAN frame loss | `ee952e1` | Overruns are **counted** (`canSniffOverrunCount()`, console `ovf=` after `rx=` in SNIFF). Nothing else changed: this makes the loss visible so the fix can be sized. | `can_probe_tests` (98), mutants | a drive's `ovf=` vs `rx=` |
| Autofocus | `0d733b5` | v0.4 `<Recording><FocusMode>fixed</FocusMode>` + `<FocusAbsolute>` holds the lens, and hands focus back as found when recording stops. **Default `camera` = untouched**, so nothing changes until it is set. | `--self-test`, `config_test`, a fake-ioctl run of the real code | whether the UGREEN has focus controls, and which value is the road |

The MKR firmware was **not compiled for SAMD21**. The cloud sandbox these commits came from cannot
download arduino-cli or the core. Build both variants on the Jetson before flashing:
`peripherals/mkr_zero/build_and_upload.sh` and `... amg`.

## Next build: the I2C hang fix

**Why this one.** The SAMD core's I2C driver waits on bus flags with no timeout
(`SERCOM::startTransmissionWIRE`, `SERCOM::readDataWIRE`). See `peripherals/mkr_zero/vendor/Wire/README.md`,
"What this does NOT fix". A device that wedges mid-transfer freezes the MKR until the 8 s watchdog
resets it. The next boot then sees `bootAfterHang()` and **quarantines both the IMU and the GNSS for the rest
of that boot**. That loses High-G, the overlay's position and GPS speed, and the GPS clock fallback, until the
car is power-cycled. The trigger has been seen: the 2026-09-26 corrupt bursts came from breadboard contacts
under vibration, the same glitch that can wedge a transfer.

**What to change.** `lib/I2CBus.h` calls this the definitive fix: bound the waits instead of containing the hang.
Two routes:
1. **Recommended: bounded transfers inside `vendor/Wire`.** It already calls the core's private SERCOM API and is
   pinned to `arduino:samd@1.8.14`. Replace the calls into the blocking SERCOM helpers with equivalents that
   poll the same INTFLAG/STATUS bits against a deadline. On timeout, return an error and run the existing
   `I2CBus` recovery (clock out the stuck slave, re-init the SERCOM). The core stays untouched.
2. Vendor or patch the core's `SERCOM.cpp` itself. This is broader (SERCOM also serves UART/SPI), and duplicate
   symbols against the core make it awkward in an Arduino build.

Either way, the callers (`IMUFunctions`, `GPSFunctions`, `BNO055Transport`) already treat a failed transfer as a
fault, so a bounded failure flows into retire → recover instead of a watchdog reset.

**How to prove it.** `helper_scripts/BusFaultInjection` wedges the bus by stopping the clock mid-read. It still
targets the **retired LSM6DSOX**. Port it to the BNO055: park the read on a register whose first data bit is 0,
e.g. `GYR_ID` (0x03) = 0x0F. Then, unlike today, attempt a real `Wire` transaction while the bus is wedged. It
must return an error within the deadline and recover, with no watchdog reset and no quarantine on the next boot.
Keep the host suites green (`make check`, `make mutations`), and flash IMUPLUS and amg.

**Also.** Fix the harness (solder or crimp the I2C wiring). The firmware can only contain the hang; the wiring
removes what triggers it, and also removes the corrupt reads behind 2026-09-26's false High-G events.

## After flashing: one drive answers the rest

- **High-G:** read `hg=` (confirmed) and `hgrej=` (rejected) on the MKR console. With the corrupt reads gated,
  `hg=` now counts real 2 g / 4 ms events. If potholes fire it, tune `IMU_HIGHG_THRESHOLD_MG` /
  `IMU_HIGHG_DURATION_LSB` from that data, not before.
- **CAN:** compare how fast `ovf=` climbs with `rx=matched/total`. A few overruns per minute on 50–100 Hz
  signals is harmless. Steady climbing means the loop drains too late: service `tickCANSniff()` between the
  long steps of `loop()` (GNSS poll, SD writes). The MCP2515 INT line is not wired on this shield, and RX
  rollover (BUKT) is already on.
- **Focus:** `v4l2-ctl -d /dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0 --list-ctrls | grep -i focus`.
  If `focus_automatic_continuous` and `focus_absolute` are listed, try `-c focus_automatic_continuous=0 -c focus_absolute=N`
  while recording and check the footage. Then set `FocusMode`/`FocusAbsolute` in `/user/output/configs/dashcam.xml`.
  No focus controls means a fixed-focus lens, and the item is closed.
- **Deploying v0.4 changes:** the service runs the newest build it finds. Rebuild
  (`docker_dev/launchcode_v0_4.sh --rebuild`, or `make dashcam_v0_4` in the container), then restart
  `dashcam-v04`. The deployed `dashcam.xml` still carries `<Encoder>`/`WarmupFrames`, which were removed in
  `409c87a`, so expect one "obsolete settings ignored" warning at start until those lines are deleted.

## Open, deliberately not done

- **v0.3 overlay:** it keeps its own copy of the bridge→overlay rules. On a fix without altitude it shows the last
  altitude, or the 52.3 m placeholder. v0.4 uses `src/bridge_overlay.h`. Skipped on request.
- **`lib/liblog/liblog.h`:** uses `uint32_t` without `<cstdint>`. The Jetson's GCC 11 doesn't mind; a newer GCC
  fails on almost every file (built here with `CXX="g++ -include cstdint"`).
- **`install_map.sh`:** does not check the core version the way `build_and_upload.sh` does (harmless in the container).
- **Unused libraries:** `libstereocam` and `libsigndetector` are in the tree but no target builds them.

## Tests

```bash
make -C peripherals/mkr_zero/tests/host check        # switch 61, imu 540, can_probe 98, bno_init 586
make -C peripherals/mkr_zero/tests/host mutations    # 45 mutants, all must be caught
make -C peripherals/esp32-c3/tests/host check        # bridge 79
make                                                 # every target, inside the l4t-ml-gpio container
B=$(ls -td bin/build_* | head -1)                    # the build just made; run from the repo root
$B/dashcam_v0_4 --self-test && $B/config_test && $B/bridge_overlay_test
$B/record_test                                       # Part A needs GStreamer base/good/bad/ugly plugins; Part B a camera
```
