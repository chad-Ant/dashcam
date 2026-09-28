# Handover — branch `camera`, 2026-09-28

## Where things stand

| Item | Commit | Status | Still needs |
|---|---|---|---|
| High-G settings | `ee952e1` | The threshold is 2 g in every IMU mode (`IMU_HIGHG_THRESHOLD_MG`, converted per accelerometer range). AMG used to write fusion's byte, which is 8 g at ±16 g. Host tests pass, and IMUPLUS is flashed on the Jetson. | a drive: `hg=` / `hgrej=` |
| CAN frame loss | `ee952e1` | Overruns are counted (`ovf=` after `rx=` in SNIFF) so the loss can be sized. Nothing else has changed yet. | a drive: `ovf=` vs `rx=` |
| Autofocus | `0d733b5` | v0.4 `<Recording><FocusMode>fixed</FocusMode>` + `<FocusAbsolute>` holds the lens. The default, `camera`, leaves it untouched. The UGREEN has `focus_automatic_continuous` and `focus_absolute` (0–1023). | the value for the road (below) |
| I2C hang | `8ebb9be` | `vendor/Wire` bounds every bus wait at 25 ms. On expiry it resets the SERCOM, and `i2cBusBegin()` recovers the bus. Proven on the rig: BusFaultInjection 10/10, twice. The control run hung until the watchdog. | — |
| AMG mode | `7882f4e` | AMG came up broken: it restored the IMUPLUS calibration profile at ±16 g, which raises SYS_ERR 0x09. The profile is now restored, and saved, in fusion modes only. Verified on the rig. | — |
| I2C bus errors | `1e93aeb` | A glitch that the controller sees as a START or STOP mid-transfer now fails that transfer, resets the controller, counts it (`i2cerr=`) and recovers the bus. Before, it returned a garbage byte as data, or left every later transfer refused while the lines read idle. **Proven on the rig 2026-09-27:** BusFaultInjection 15/15, twice (results below). Production flashed; `i2cto=0 i2cerr=0` at rest. | a drive: `i2cerr=` before and after the wiring rework |
| Build scripts | `c232b6d` | `check_core.sh` holds the arduino:samd 1.8.14 check. `install_map.sh` and the BusFaultInjection script now apply it too. | — |
| liblog | `07a61d4` | `liblog.h` includes `<cstdint>`, so the tree builds with GCC 13 without `-include cstdint`. | — |
| libcamera: dependencies | `05f3075` | `AttributeDictionary` now lives in libcamera (`libcamera_attributes.cpp`); libconfig no longer includes libcamera. There is no behaviour change. | — |
| libcamera: V4L2 helpers | `bbc0caa` | Discovery, exposure and focus now share `libcamera_v4l2.h`: an EINTR-safe ioctl, an owning fd with `O_CLOEXEC`, and control query/get/set. Checked against the old code on a fake UVC device: identical ioctl sequences in 14 scenarios. The EINTR retry is defensive only: the apps install handlers with `std::signal`, which sets SA_RESTART, so it never fires on the Jetson. **Run on the Jetson 2026-09-28:** `scan_cameras` output is byte-identical to the previous build's, and v0.4 starts, records and stops as before (results below). | — |
| libcamera: Camera_GST | `72e4dc7` | Fixed a **teardown deadlock**: a `start()` that timed out hung forever instead of reporting ERROR. Fixed a leak of refused branch bins, and logging under the state lock. Start and runtime failures now name the element and GStreamer's reason. New `camera_gst_test` (videotestsrc, no camera). v0.4 does not use Camera_GST. **Run on the Jetson 2026-09-28:** `camera_gst_test` passes on GStreamer 1.20.3. The fix for branches that start disabled works only when the sink sees GAP events directly, so not for this project's real branches (review item C, fixed in `f03a13f`). | — |
| libcamera: attributes | `7697f40` | 6 of the 10 USB entries in `camera_attributes.xml` named v4l2src properties that do not exist, so they were ignored with only a GLib warning. They are now V4L2 controls written through `extra-controls` (valueType `v4l2_control`). A write is checked against the **element's** property type and range, and a rejected one reports `INVALID_ATTRIBUTE` plus a WARN. That does not check the **device's** range: the driver clamps out-of-range USB values, and those still report NONE (review item F). **Run on the Jetson 2026-09-28:** `usb_test` Test 4b reads back every control it sets, and the CSI dictionary check passes. `usb_test` changes the camera's settings and leaves them changed (review item A, fixed in `782bdfd`). | — |
| libcommlink: fixes | `68fcceb` | CommLink logged its port open/close lines with the port mutex held, so a log callback that asked the link anything deadlocked (latent; v0.4's does not). The bridge tty and the wake pipe are now close-on-exec (children such as `nmcli` inherited them). `typeName()` now names `CMD_SET_IMU_MODE` in both `HostProtocol.h` copies, which stay byte-identical; the wire is unchanged. Discovery keeps partial results when a node vanishes mid-scan. `Uart::read(0)` no longer makes two `fcntl()` calls per read. New `logPrintf()` in liblog replaces three private log helpers. | **not run on hardware:** `commlink_test` with the C3, and v0.4's bridge line (below) |
| libbus | `5be7a5b` | `ibus.h` (the base of libuart, libspi and libi2c) moved from `lib/libgpio` to `lib/libbus`. There is no code change. | — |
| CSI driver loader | `aa9deb1` | The Jetson's own IMX296 unit stopped nvargus-daemon, failed to load a module built for 5.15.185-tegra into 5.15.199-tegra, and left Argus stopped (found by the 2026-09-28 review). `docker_dev/csi_driver.sh` checks the module against the running kernel before touching Argus, and restarts Argus on every exit path; `csi-driver.service` loads it before Argus at boot. It does **not** fix the mismatch: the module must be rebuilt for 5.15.199-tegra. Host test: `test_csi_driver.sh`, 31 checks. | **not run on hardware:** the rebuild, then the unit (below) |
| libcamera: review fixes | `f03a13f` | Shutdown is bounded when a branch stops consuming (it hung, reproduced on the Jetson) or the source's thread is stuck. A stream that ends reports ERROR (it stayed RUNNING). Branches that start disabled start (item C). frameCount restarts at `start()`. Items B, D, E, F (docs), G, H, J and both older races fixed. `camera_gst_test` 72 → 120 checks. | **not run on hardware:** `camera_gst_test`, the reviewer's reproductions, `csi_test` (below) |
| usb_test | `782bdfd` | Puts the controls it writes back as found, checks that, and reads back what Tests 3 and 4 set (items A, B). | **not run on hardware:** `usb_test` (below) |
| Focus hold | `3e3dc4f` | `UvcFocusControl` refused nothing when it could not read what to hand back: autofocus came back on for a camera found in manual, or the manual lens position was not restored (the review's mocked read failures). It now refuses before writing anything. New `focus_test`: 22 checks against a simulated camera. | **not run on hardware:** v0.4 with `FocusMode=fixed` (below) |
| commlink_sim_test | `0f548d7` | CommLink against a simulated C3 on a pseudo-terminal, with no hardware: 73 checks in about 6 s (handshake, every frame type, commands, watchdog, hot-plug, discovery, libuart). Passed 36 of 36 runs, 16 of them overloaded; ASan, UBSan and TSan are clean. | a run on the Jetson (below) |

## Who does what: read before handing work across

Two coders work on this branch:
- The **cloud session** runs in a virtual machine that holds this repository and nothing else. It wrote this file.
- The **Jetson coder** works on the Jetson, with the MKR rig, the camera and the car.

**What the cloud session cannot do:**
- **Touch hardware.** It has no MKR Zero, BNO055, GNSS, MCP2515 or CAN bus, ESP32-C3, UGREEN camera, Jetson GPU,
  serial console or car. It cannot flash or run firmware, watch a console, measure a line, look at footage or
  drive.
- **See what happened on the car.** It sees the car's logs, the SD card, the deployed `/user/output` configs and
  the Jetson's files only when someone commits them or pastes them into the conversation.
- **Build or run anything that needs CUDA or a camera.**
  - CUDA: `liblanedetector` and `libdriverstate`, and so `dashcam_v0_2` and `dashcam_v0_3`, do not build there.
  - Camera: `record_test` Part B, `csi_test` and `usb_test` cannot run. `camera_gst_test` covers the GStreamer
    camera class over videotestsrc, but not v4l2src or Argus themselves.
- **Build the binary you flash.** It compiles the MKR firmware with GCC 13.2 against the core cloned from git
  (recipe below), not with the Jetson's 7.2.1. That is a compile check, not the shipped image. It has not built
  the ESP32-C3 firmware at all; only the C3 host tests run there.
- **Know timing or electrical behaviour.** Anything at register level (bus states, clock stretching, what a glitch
  does) is reasoned from the datasheet and stays a hypothesis until the rig runs it. `1e93aeb` is exactly that
  case.
- **Keep anything between sessions.** The machine is rebuilt every session: installed toolchains are gone, and
  only pushed commits survive.
- **Count on the network.** `downloads.arduino.cc` is blocked. GitHub and apt worked on 2026-09-27, but that
  depends on the environment's settings, not on anything the session controls.

**Cloud sessions, this one included, leave hardware tests to the Jetson coder.**
- Deliver the code, the host tests, and the rig procedure: which test, which command, what a pass looks like, and
  what each failure means.
- Never report a hardware result that nobody measured.
- Mark every change that still needs the rig as "not run on hardware" in the status table.

**Jetson coder: be careful what you hand to a cloud session.**
- Give it work it can finish and check by itself: code with host tests, reviews, refactors, docs and compile checks.
  Do not ask it to flash, measure, run on the car or check footage.
- Put the hardware facts it needs into the handover: console lines, test output, measured numbers, failure
  records. It cannot go and look for them.
- Treat its firmware changes as unproven until you have run them. It compiled them with a different compiler, and
  none of that code has run on a real bus.

## Done 2026-09-28 (Jetson coder): the libcamera pass run on the Jetson

Everything was built from `ef45a22` in the l4t-ml-gpio container with plain `make` into `bin/build_20260928_074644`.
The build is clean with GCC 11.4; the only warnings are the two known `read()` ones. The five steps the cloud
session asked for:

1. **`camera_gst_test`: `RESULT: PASS`.** It prints 73 `ok` lines, not 70: the file has 70 `check()` calls, the
   capture-racing-stop check runs 3 times, and the dictionary check runs once per source element found. It takes
   about 4.5 s. It passed 8 of 8 runs, and 3 of 3 with every core busy.
   - Valve `drop-mode` exists on 1.20.3 (0 drop-all, 1 forward-sticky-events, 2 transform-to-gap).
     `start() links both branches` passes.
   - CSI line: `ok    CSI: all 10 entries name a writable nvarguscamerasrc property of a fitting type, or a V4L2
     control`. The USB line reports all 11 entries.
   - Two `(Argus) Error FileOperationFailed: Connecting to nvargus-daemon failed` lines appear on stderr. They are
     expected: creating nvarguscamerasrc tries the daemon, and it is not running on this Jetson (no CSI camera).
2. **`usb_test` on the UGREEN: Tests 1–7 and 4b PASS** (8 to 10 are skipped with one camera). The Test 4b lines:
   ```
   brightness               set 32 (range 1..64), device reads 32, error=0 (OK)
   gain                     set 7 (range 0..15), device reads 7, error=0 (OK)
   backlight_compensation   set 0 (range 0..3), device reads 0, error=0 (OK)
   ```
   Gain was **not** skipped. uvcvideo never flags gain inactive; only `exposure_time_absolute`,
   `white_balance_temperature`, `focus_absolute` and hue follow an auto control. **The run changed the camera's
   settings and left them changed:** brightness 32 → 64 and gain 0 → 7. A UVC camera keeps these until it loses
   power, and usb_test runs on the dashcam's own camera. Both were put back by hand
   (`v4l2-ctl -d /dev/video0 --set-ctrl=brightness=32,gain=0`), and `--list-ctrls` then matched the snapshot taken
   before the run. Cause and fix: review item A.
3. **`scan_cameras`:** the output of the new build and of the previous one (`bin/build_20260927_102633`) is
   byte-identical: 1 camera, 20 controls, 28 formats.
4. **v0.4:** the new binary ran for 25 s with the service's mounts, and stopped with
   `timeout --preserve-status -s INT`.
   - Its start log has the same `exposure: frame-rate priority on /dev/video0 (exposure <= 32.3 ms, gain 0..15,
     target luma 120)` line. The only WARN is the known obsolete-settings one. There is no `focus:` line, because
     the deployed `dashcam.xml` does not set `FocusMode`.
   - The bridge connected: `bridge fw 1.0 proto 7`.
   - It recorded 713 frames and exited with `rc=0`, so the graceful-exit contract holds.
   - The camera's controls were unchanged after the hand-back.
   - The `dashcam-v04` service was already stopped when this session began, and is left stopped. Its next start
     runs this build, the newest.
5. **Deployed `camera_attributes.xml`:** there is nothing to replace, and the step's premise was wrong.
   - `/user/output/configs`, i.e. `/media/jetson/backup/configs`, holds only `dashcam.xml`.
   - The build seeds a fresh `bin/build_<ts>/config/`, so `cp -n` never keeps a stale copy. The new build's copy is
     identical to the repo's.
   - v0.2 and v0.3 resolve only CSI entries (v0.3 applies attributes to its CSI lane camera only), so the USB change
     does not reach them.
   - Only a copy placed by hand in `/user/output/configs` would be stale. If one ever appears, delete it rather
     than replace it.

Also run, all PASS: `dashcam_v0_4 --self-test`, `config_test`, `bridge_overlay_test`, `exposure_test`,
`liblog_test`, and `record_test` Part A and Part B (UGREEN). `record_test` leaves the camera's controls unchanged.
The MKR and C3 host suites were not rerun, because this pass does not touch `peripherals/`.

`bbc0caa` also changed discovery (`getCameraList()`), which v0.4 runs at every recording start. The note in the old
step list missed that; steps 3 and 4 cover it.

## Next (Jetson coder): the libcamera review fixes and the CSI driver

The cloud session fixed the 2026-09-28 review (`aa9deb1`..`3e3dc4f`) and ran the fixes only against videotestsrc, a
simulated focus camera and stubbed `systemctl`. None of it has run on the Jetson. In the order the review asked for:

1. **CSI driver.** The unit that loads the IMX296 driver is on the Jetson, not in the repo, so `csi-driver.service`
   was written from the review's description of it. Check it against the real one first.
   - Find the old unit (`grep -l nvargus /etc/systemd/system/*.service`) and commit a copy (`systemctl cat <unit>`)
     to `docker_dev/`, so the next session can see it.
   - If Argus is stopped now: `sudo systemctl start nvargus-daemon`.
   - `docker_dev/csi_driver.sh check` changes nothing. Expect `cannot load: no nv_imx296 for the running kernel
     5.15.199-tegra; built for: 5.15.185-tegra`. If it finds no module at all, the driver has another name: set
     `CSI_MODULE=<name>`, the `.ko` name without its extension.
   - Rebuild the driver for 5.15.199-tegra: the vendor's source,
     `make -C /lib/modules/$(uname -r)/build M=$PWD modules`, install the `.ko` under
     `/lib/modules/$(uname -r)/updates/`, then `sudo depmod`. `check` must then say `would load`.
   - Replace the old unit: disable it, then install per the header of `docker_dev/csi-driver.service`, and reboot.
     Expect `systemctl status csi-driver` to show success, `nvargus-daemon` active, and `csi_test` to pass.
   - **Decision for the user:** hold the kernel packages (`apt-mark hold`), or build the driver with DKMS, so the
     next kernel upgrade does not break the camera again. With `csi-driver.service`, an upgrade only costs the CSI
     camera, and Argus stays up.
2. **Bounded shutdown and the other Camera_GST fixes.** In the container, `make`, then from the repo root with
   `B=$(ls -td bin/build_* | head -1)`:
   - `$B/camera_gst_test`: expect `RESULT: PASS` and 121 `ok` lines in about 11 s. (Here 120; the Jetson adds the
     nvarguscamerasrc dictionary line.) GStreamer 1.20.3 is the real check: this was run on 1.24.2. Watch in
     particular:
     - `stalled branch shutdown`: stop() about 0.25 s.
     - `wedged source shutdown`: about 0.85 s, and the stuck pipeline released.
     - `finite stream`: ERROR, and close() under 1 s.
     - `disabled branches`: x264enc is present on the Jetson, so both shapes run.
   - A test that hangs is failed by the watchdog, which names it. `$B/camera_gst_test <part of a name>` runs only the
     matching tests.
   - **Rerun your own reproductions:**
     - the blocked consumer with a 250 ms budget: stop() now returns in about 250 ms plus the NULL transition, with
       one `EOS not delivered ... flushing the branches` WARN;
     - the finite stream: ERROR with `reached end of stream`;
     - the `videorate` branch that starts disabled: PLAYING.
   - **Once CSI works:** `$B/csi_test`, and v0.3 start/stop, go through the new teardown on nvarguscamerasrc, which
     the cloud cannot run. Normal stops should log no `EOS not delivered` WARN and take as long as before. If Argus
     posts its CANCELLED error instead of EOS, that still ends the wait early.
3. **usb_test** (items A and B; stop `dashcam-v04` first):
   - `v4l2-ctl -d /dev/video0 --list-ctrls > /tmp/before`, run `$B/usb_test`, then
     `v4l2-ctl -d /dev/video0 --list-ctrls > /tmp/after`, and `diff /tmp/before /tmp/after`. The diff must be empty.
   - Expect `Controls restored as found: PASS`.
   - Tests 3 and 4 now print `device reads N` for an in-range brightness.
   - Test 4b adds `put back to N, reads N (OK)` lines.
   - If Test 3 fails, the extra-controls path does not apply a value queued before start() on 1.20.3: report the
     line.
4. **Focus:** `$B/focus_test` needs no hardware (22 checks). The deployed `dashcam.xml` does not set `FocusMode`,
   so v0.4 never runs this code today. With `FocusMode=fixed`, stop v0.4 and check that
   `v4l2-ctl -d /dev/video0 -C focus_automatic_continuous` reads as it did before the run.

Report back in this file: each test's RESULT, the stop() times, and whether the CSI unit came up after a reboot.

## Next (Jetson coder): run the libcommlink pass on the Jetson

The cloud session changed libcommlink and libuart on 2026-09-28 (`68fcceb`..`0f548d7`) and ran them only
against a simulated bridge. The link's wire traffic is unchanged. Stop `dashcam-v04` first: it holds the bridge
port exclusively. In the container, `make`, then from the repo root with `B=$(ls -td bin/build_* | head -1)`:

1. **`$B/commlink_sim_test`** needs no hardware. Expect `RESULT: PASS`, 73 checks in about 6 s. It runs the pty
   fake bridge on the Jetson's GCC 11.4 and kernel.
2. **`$B/commlink_test`** with the C3 plugged in, for about 30 s (`$B/commlink_test "" 30` auto-discovers).
   Expect:
   - `bridge fw 1.0 proto 7`, then telemetry and status lines;
   - pulling and reinserting the C3's cable: `bridge port closed`, then `bridge port open`, a new HELLO,
     streaming again, and `reconnects=1` in the stats line;
   - no `NACK` lines. If one appears, it now names the command (`CMD_SET_IMU_MODE`, not `UNKNOWN`).
3. **v0.4:** after the rebuild and restart, the start log shows `bridge fw 1.0 proto 7` as before, and the overlay's
   vehicle fields fill in.
4. **The C3 needs no reflash.** The only `HostProtocol.h` change is a name in `typeName()`, which the C3 never
   calls. `md5sum lib/libcommlink/HostProtocol.h peripherals/esp32-c3/lib/hostLink/HostProtocol.h` must print the
   same sum twice.

Report back in this file: the sim test's RESULT and the `commlink_test` reconnect lines.

**Proposed, not done:** each changes behaviour on the car, so it is the Jetson coder's or the user's call.
- **Back off after a failed handshake.**
  - Today a bridge on the wrong firmware, or a silent device, is reopened every reconnectMs + handshakeMs
    (about 3 s) forever.
  - Each round logs about four lines. v0.4's `quietRepeats` demotes the repeats to DEBUG, but DEBUG is the default
    level, so the log file still grows.
  - Proposal: double the interval per consecutive handshake failure, capped at about 30 s, and reset it on a
    compatible HELLO. It would slow reconnecting to a bridge that was just reflashed by up to the cap.
- **Twelve more libraries** keep their own truncating `doLog()`: libcan, libconfig, libdriverstate, libgpio (x2),
  libi2c, liblanedetector, libmidi, librecord (x2), libspi and libstereocam. Switch each to `logPrintf()` in its
  own pass.
- **`Uart::readLine()`** costs a poll and a read per character. That is fine for `gpio_test`'s NMEA; buffer it if
  a real consumer appears.

## Done 2026-09-28 (cloud): the libcamera review, and the Jetson coder's hardware findings

Each fix has a `camera_gst_test`, `focus_test` or `test_csi_driver.sh` check that fails with the old code. In
camera_gst_test the fixes were reverted one at a time, and 15 of 17 such mutants fail it. The two that pass are
guards that no test can reach now:
- teardown skipping the second EOS for a stream that already ended: videotestsrc lets a second EOS through fast;
- `close()` waiting for `start()`: ERROR can no longer appear while `start()` builds.

ASan, UBSan and LSan are clean. TSan is clean with the uninstrumented GLib/GStreamer libraries suppressed, and it
does report the old unlocked branch-list clear. None of it has run on the Jetson (the procedure above).

**The findings from the 2026-09-28 hardware runs:**
- **High: shutdown hung behind a blocked branch** (`f03a13f`).
  - `gst_element_send_event(EOS)` waits for each source's streaming thread, which a full blocking branch queue
    holds in the tee. It also holds the pipeline's state lock, so the NULL transition cannot run either.
  - EOS now goes out on its own thread, and `eosTimeoutMs` starts at once.
  - Still stuck at the deadline: every branch is flushed from its head, past its valve, which drops FLUSH_START.
    That frees the tee.
  - Still stuck after `stateChangeTimeoutMs`: the pipeline is left to the EOS thread, which sets it to NULL and
    releases it when it frees (logged as ERROR).
  - **Cost:** when a branch has stalled the tee, every branch had stopped getting frames, and none of their files is
    finalised at shutdown.
- **Medium: a finished stream reported RUNNING.** `gst_bus_pop_filtered()` discards what does not match, and the
  mask excluded EOS. EOS now turns RUNNING into ERROR, and teardown does not wait for a second one. The capture
  appsink's `wait-on-eos` is off, so the EOS does not wait for a pull either.
- **Medium: the focus hold took over what it could not hand back** (`3e3dc4f`): refused now, see the status table.
- **High: the CSI driver unit** (`aa9deb1`): see the procedure above.
- **Low: frameCount survived restarts:** it restarts at `start()`.
- **Found by the new test under load:** a start failure could log `(no error on the bus)`. GstBin wakes the state
  wait with FAILURE before it forwards the ERROR. `logFirstBusError()` now waits up to 250 ms for it.

**The review items** (`f03a13f` unless noted):
- **A** (`782bdfd`): usb_test puts brightness, gain and backlight_compensation back as found, and fails if they do
  not read back. Tests 3 and 4 use values inside the range. The false "gain is not active" comment is gone.
- **B:** USB brightness, contrast, saturation and hue are V4L2 controls (`extra-controls` is kept while the device
  is closed). usb_test Test 3 reads the value back.
- **C:** branches that start disabled use drop-mode 1 again, and every sink in them is `async=false`. A videorate
  branch and an x264enc branch both start, and no GAP reaches them.
- **D:** `start()`'s flushes write the V4L2 controls in one structure, auto controls first. A write while running
  carries only its own control, so it no longer re-sends, and undoes, the others.
- **E:** a USB `white_balance_automatic` entry. `white_balance_temperature` needs it at 0.
- **F:** documented only (`camera_attributes.xml`, `libcamera.h`): a V4L2 control value gets no device feedback.
  Checking it with `VIDIOC_QUERYCTRL` is not done (below).
- **G:** `setCameraAttribute()` queues while `starting_`.
- **H:** the teardown comment now says a prerolled branch's file stays unfinalised when `start()` times out.
- **I:** the capture-valve check drains until a pull times out.
- **J:** the `aelock` / `awblock` comments are no longer inverted.
- **The older races:** `close()` waits for `start()`, and teardown swaps the branch lists out under the lock. Also,
  `checkBusErrors()` now does nothing while `start()` builds: a status poll used to take the start error off the
  bus, flip to ERROR, and let `close()` free the pipeline under `start()`.

## Done 2026-09-27 (Jetson coder): the bus-error fix proven on the rig

BusFaultInjection, built instrumented with the command below, passed **15 of 15** on the boot run and on a
rerun (`a`):

- **T1b:** `endTransmission=4 in 16 us, timeouts +0, bus errors +1`, as predicted.
- **C7:** the premise holds on this silicon. With the stock path, the bus read Ready 3/3 while CHIP_ID failed 3/3
  (`state busy`): the dead bus.
- **T7/T8:** a glitch START makes the next read fail in 18–19 µs as one bus error. The bus then comes back by
  itself, and CHIP_ID `0xA0` and the GNSS both answer.
- **T9/T9b:** SDA glitched mid-read gives 0 bytes in 873–953 µs, one bus error and no bytes, then recovery.
- **T0–T6b:** unchanged from 2026-09-27. T5 took 25.02 ms and T5b 25.41 ms; both recover.
- **Longest completed bus wait:** 682–756 µs against the 25 ms deadline.
- **No SKIP lines.**

Production (IMUPLUS, 121,684 B) was re-flashed afterwards. At rest it shows `imu=up`, `cal=33`, `gps=up`,
`i2cto=0 i2cerr=0`, `ovf=0`.

The procedure below is kept for reruns after a Wire change.

## Rig procedure: the bus-error proof

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

## After flashing (Jetson coder): one drive answers the rest

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
- **Seen during the libcamera pass, left for the next libraries:**
  - `libcan.cpp:319` and `libgpio.cpp:311` ignore `read()`'s return (the only two warnings in the build).
  - `Camera_GST::getCameraStatus()` drains the bus through a `const_cast` (it works, but it is a smell).
  - `cameraAttribute` stores control ranges as `float`, which is lossy past 2^24 (no such control on the UGREEN).
- **Proposed after the review fixes, not done:**
  - **Item F, properly:** check a USB V4L2 control against the device (`VIDIOC_QUERYCTRL` / `QUERYMENU` on the node,
    by its normalised name) and report `INVALID_ATTRIBUTE` for a missing control or an out-of-range value. Today
    the driver clamps silently.
  - **A branch that stalls the tee stops every branch.** Recording branches use blocking queues so no frame is
    dropped; one stuck sink (an SD card stalling) starves the others, and at shutdown none is finalised. A leaky
    recording queue, or a per-branch watchdog, trades that for dropped frames. It is a design decision; no app on
    the car uses a blocking branch today.

## Cloud sessions: compile-checking the MKR firmware

This is a compile check only (see "Who does what"). The sandbox blocks `downloads.arduino.cc`, so the toolchain is
put together from GitHub and apt, and has to be rebuilt every session (a few minutes). `1e93aeb` was
compile-checked from these parts:

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

## Environment (the Jetson rig, as of 2026-09-27)

Collected from the machine, not from memory: `/etc/nv_tegra_release`, `dpkg-query`, `arduino-cli core/lib list`,
and the tools' own `--version`. Re-collect after an upgrade.

**Hardware**
- **Jetson:** NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super. 6 cores, 7.4 GB RAM, 937 GB NVMe
  root. No RTC coin cell, so the clock comes from NTP or GPS. It runs on **its own battery** in the car.
- **Camera:** UGREEN Camera 4K, USB UVC, `eba4:6579`, `/dev/video0`. It records 1080p30 MJPEG (`FormatIndex 4`),
  pinned by `/dev/v4l/by-id`. Controls are standard UVC only (focus, exposure, gain, backlight compensation).
  Frames from the evening of 2026-09-26 are upright.
- **Telemetry master:** Arduino MKR Zero (SAMD21G18A), **powered from the car's OBD2 port**, with USB to the Jetson
  as the dev console.
  - **CAN:** MKR CAN Shield, an MCP2515 with a 16 MHz crystal, 500 kbit/s, sniffing the Honda Brio (map `brio`,
    id 0x0B, on the SD card).
  - **I2C:** at 400 kHz. BNO055 IMU at 0x29 on a **breadboard**, INT pin not wired. SparkFun u-blox GNSS at 0x42
    over ESLOV.
  - **SD card:** on SPI1, holding the CAN map and the IMU calibration profile.
- **Bridge:** Seeed XIAO ESP32-C3. It talks to the MKR over UART1 at 115200 and to the Jetson over native USB CDC.

**Jetson host**
- JetPack 6.2.3 (`nvidia-jetpack 6.2.3+b81`), L4T R36.5.2, kernel 5.15.199-tegra, Ubuntu 22.04.5 LTS.
- CUDA 12.6.11, TensorRT 10.3.0.30 (`libnvinfer10`, built for CUDA 12.5), GCC 11.4.0, Python 3.10.12.
- GStreamer 1.20.3, v4l-utils 1.22.1, Docker 29.8.1.
- `nvargus-daemon` was not running. The 2026-09-28 review found why: an IMX296 is fitted, but its driver is
  built for 5.15.185-tegra, and the unit that loads it stopped Argus and left it stopped (procedure above). While
  Argus is down, anything that loads nvarguscamerasrc prints two `(Argus) Error ... Connecting to nvargus-daemon
  failed` lines.
- Host Python's `cv2` is broken (numpy mismatch). Use the container.

**Dev container** `l4t-ml-gpio:latest`
- Built 2026-09-25 from `docker_dev/Dockerfile`, base `dustynv/l4t-ml:r36.4.0`. The repo is mounted at
  `/user/dashcam`. It builds everything: plain `make` gives a timestamped `bin/build_<ts>/`.
- GCC 11.4.0, Python 3.10.12, CUDA 12.6 (nvcc), OpenCV 4.10.0-dev, NumPy 1.26.4, PyTorch 2.6.0, GStreamer 1.20.3.
- spdlog 1.9.2, fmt 8.1.1, pugixml 1.12.1, pyserial 3.5.

**Firmware toolchains (inside the container)**
- **arduino-cli:** 1.5.1.
- **MKR Zero:**
  - Core `arduino:samd` **1.8.14**, pinned; `check_core.sh` refuses anything else. On aarch64 it is installed by
    `docker_dev/install_samd_aarch64.py`, because arduino-cli cannot install it there.
  - arm-none-eabi-gcc **7.2.1** (package `7-2017q4`), CMSIS 4.5.0, CMSIS-Atmel 1.2.0, bossac 1.7.0-arduino3.
  - Build and flash with `peripherals/mkr_zero/build_and_upload.sh [auto] [amg]`.
- **ESP32-C3:**
  - Core `esp32:esp32` **3.2.0**, esptool 5.4.0.
  - FQBN `esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app,CPUFreq=160,FlashMode=qio,FlashFreq=80,FlashSize=4M,DebugLevel=none,UploadSpeed=921600`.
- **Arduino libraries**, pinned in the Dockerfile and installed `--no-deps`:
  - Time 1.6.1
  - SparkFun u-blox GNSS Arduino Library 2.2.29
  - Servo 1.3.0
  - RTCZero 1.6.0
  - Adafruit BusIO 1.17.4
  - Adafruit GFX Library 1.12.6
  - Adafruit SSD1306 2.5.17
  - Adafruit LED Backpack Library 1.5.1
- **Vendored** in `peripherals/mkr_zero/vendor`:
  - Wire: the arduino:samd 1.8.14 copy, patched with the `busOwner` fix, bounded transfers and bus errors. See its
    README.
  - CAN 0.3.1, patched.
  - SdFat 2.3.1.
  - BNO055 1.2.1.

**Protocol and firmware versions**
- MKR↔C3 `CommProtocol` and C3↔Jetson `HostProtocol` are both at version **0x07** (180-byte telemetry payload).
- The C3 bridge firmware reports **1.0**.
- The MKR production image is this branch's HEAD, IMUPLUS, 121,684 bytes.
- `dashcam-v04.service` runs the newest `bin/build_*/dashcam_v0_4` it finds at start.

**Where the cloud sandbox differs** (see "Cloud sessions" above): arm-none-eabi-gcc 13.2 and host g++ 13 instead of
7.2.1 and 11.4.0, no CUDA, and no hardware.

## Tests

```bash
make -C peripherals/mkr_zero/tests/host check        # switch 61, imu 540, can_probe 98, bno_init 602
make -C peripherals/mkr_zero/tests/host mutations    # 47 mutants, all must be caught
make -C peripherals/esp32-c3/tests/host check        # bridge 79
make                                                 # every target, inside the l4t-ml-gpio container
B=$(ls -td bin/build_* | head -1)                    # the build just made; run from the repo root
$B/dashcam_v0_4 --self-test && $B/config_test && $B/bridge_overlay_test
$B/camera_gst_test [name]                            # Camera_GST over videotestsrc: no camera needed
$B/focus_test                                        # the fixed-focus hold against a simulated camera
docker_dev/test_csi_driver.sh                        # csi_driver.sh against stubs: changes nothing on the machine
docker_dev/csi_driver.sh check                       # on the Jetson: the IMX296 module vs the running kernel
$B/commlink_sim_test                                 # CommLink + libuart against a simulated C3 on a pty: no hardware
$B/commlink_test "" 30                               # the real C3 bridge (stop dashcam-v04 first)
$B/record_test                                       # Part A needs GStreamer base/good/bad/ugly plugins; Part B a camera
$B/usb_test                                          # a USB camera (stop dashcam-v04 first); puts its controls back
$B/scan_cameras                                      # lists the cameras, their controls and formats
```

`usb_test` puts brightness, gain and backlight_compensation back as found (`782bdfd`). Until it has passed on the
Jetson, compare `v4l2-ctl -d /dev/video0 --list-ctrls` before and after it anyway.

The Wire changes have no host test: they are register-level SAMD21 code, proven only on the rig
(BusFaultInjection). Running that is the Jetson coder's job.

Where each test can run:
- **Anywhere, cloud included:** the host suites, `mutations`, `--self-test`, `config_test`,
  `bridge_overlay_test`, `camera_gst_test`, `focus_test`, `commlink_sim_test`, `test_csi_driver.sh` and
  `record_test` Part A.
- **Jetson only:** CUDA targets, `record_test` Part B, `usb_test`, `csi_test`, `scan_cameras` (with a camera),
  `commlink_test` (with the C3), `csi_driver.sh`, and everything on the rig or the car.
