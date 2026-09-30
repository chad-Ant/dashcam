# Handover — branch `camera`, 2026-09-30

## Where things stand

| Item | Commit | Status | Still needs |
|---|---|---|---|
| Platform | — | **Rootfs, kernel and userspace back on L4T R36.5.0 (JetPack 6.2.2, kernel 5.15.185-tegra) on 2026-09-30**, for the IMX296 driver; the 65 NVIDIA packages are held. **The QSPI bootloader/UEFI is still 36.5.2** (slot B; the 36.5.0 capsule did not take effect). On this mix, the IMX296 in CAM1 streams through Argus again (`csi_test` 5/5), and v0.4, the UGREEN and CUDA all work (below). | Decide whether to keep UEFI 36.5.2 under the R36.5.0 rootfs (everything tested works) or re-apply the 36.5.0 bootloader (root). |
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
| libcommlink: fixes | `68fcceb` | CommLink logged its port open/close lines with the port mutex held, so a log callback that asked the link anything deadlocked (latent; v0.4's does not). The bridge tty and the wake pipe are now close-on-exec (children such as `nmcli` inherited them). `typeName()` now names `CMD_SET_IMU_MODE` in both `HostProtocol.h` copies, which stay byte-identical; the wire is unchanged. Discovery keeps partial results when a node vanishes mid-scan. `Uart::read(0)` no longer makes two `fcntl()` calls per read. New `logPrintf()` in liblog replaces three private log helpers. | **Run on the Jetson 2026-09-28:** `commlink_test` recovers from a cable pull on the C3's by-id path (what v0.4 uses), and v0.4's bridge line is as before. Auto-discover mode fails on this rig, which predates the pass (item U). Open: items U, V. |
| libbus | `5be7a5b` | `ibus.h` (the base of libuart, libspi and libi2c) moved from `lib/libgpio` to `lib/libbus`. There is no code change. | — |
| CSI driver loader | `aa9deb1` | The Jetson's own IMX296 unit (`imx296-reload.service`) stopped nvargus-daemon, then failed at `modprobe -r imx296`, because 5.15.199-tegra has no imx296 module at all (only 5.15.185-tegra has one), and left Argus stopped (found by the 2026-09-28 review; corrected by the Jetson run). `docker_dev/csi_driver.sh` checks the module against the running kernel before touching Argus, and restarts Argus on every exit path; `csi-driver.service` (in the repo, not installed) would load it before Argus at boot. The script does not fix a kernel/module mismatch by itself: on 5.15.199 the module would have had to be rebuilt, and the 2026-09-30 rollback to 5.15.185 removed the mismatch instead. Host test: `test_csi_driver.sh`, 31 checks. | Not needed on R36.5.0: there, the vendor's `imx296-reload.service` works as intended (active since the rollback). Items K–P before `csi-driver.service` replaces it, if ever. |
| libcamera: review fixes | `f03a13f` | Shutdown is bounded when a branch stops consuming (it hung, reproduced on the Jetson) or the source's thread is stuck. A stream that ends reports ERROR (it stayed RUNNING). Branches that start disabled start (item C). frameCount restarts at `start()`. Items B, D, E, F (docs), G, H, J and both older races fixed. `camera_gst_test` 72 → 120 checks. | **Run on the Jetson 2026-09-28:** `camera_gst_test` PASS 14/14; the reviewer's shapes redone on the UGREEN. **2026-09-30 on CSI:** `csi_test` 5/5, `csi_rtp_test` PASS, and Camera_CSI stops no slower than before, with no WARN (below). Open: items Q–T. |
| usb_test | `782bdfd` | Puts the controls it writes back as found, checks that, and reads back what Tests 3 and 4 set (items A, B). | **Run on the Jetson 2026-09-28:** PASS, the before/after control diff is empty; again on 2026-09-30 (R36.5.0). Open: item Z. |
| Focus hold | `3e3dc4f` | `UvcFocusControl` refused nothing when it could not read what to hand back: autofocus came back on for a camera found in manual, or the manual lens position was not restored (the review's mocked read failures). It now refuses before writing anything. New `focus_test`: 22 checks against a simulated camera. | **Run on the Jetson 2026-09-28:** v0.4 with `FocusMode=fixed` holds the lens and hands autofocus back. Open: items X, Y. |
| commlink_sim_test | `0f548d7` | CommLink against a simulated C3 on a pseudo-terminal, with no hardware: 73 checks in about 6 s (handshake, every frame type, commands, watchdog, hot-plug, discovery, libuart). Passed 36 of 36 runs, 16 of them overloaded; ASan, UBSan and TSan are clean. | **Run on the Jetson 2026-09-28:** PASS 16/16, 5 of them with every core busy. Open: item W. |

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

## Done 2026-09-30 (Jetson coder): back on L4T R36.5.0, and the CSI camera works again

**Why.** The IMX296 vendor driver exists only as prebuilt modules for 5.15.148 and 5.15.185. There is no build for
5.15.199, and no source package, so the user chose to go back. The `apt upgrade` of 2026-09-23 had moved the whole
NVIDIA BSP from R36.5.0 to R36.5.2 (JetPack 6.2.2 → 6.2.3), not just the kernel. Along the way it:
- deleted every package-owned 5.15.185 module, including `uvcvideo`, nvgpu and the camera stack; only the vendor's
  `imx296.ko` survived;
- switched extlinux `DEFAULT` from `JetsonIO` to `primary`, which dropped the camera overlay.

**How.** `~/drive_logs/tools/l4t_rollback_36_5_0.sh` reverses exactly the NVIDIA part of that upgrade: 65
packages, each back to the version `/var/log/apt/history.log` records for it. The script is outside the repo. It
has `--list`, `--check`, `--simulate`, `--apply` and `--hold`.
- `--check`: every 36.5.0 version was still in the r36.5 repo.
- `--simulate`: 65 downgrades, nothing removed, and no R36.5.2 package left behind.
- `--apply` (the user, 2026-09-30):
  - all 65 packages downgraded;
  - the bootloader capsule staged, the initrd rebuilt for 5.15.185, the device tree replaced;
  - a new `/lib/modules/5.15.185-tegra` with 1037 `.ko` files (1023 in `modules.dep`), including `uvcvideo`,
    `cdc-acm`, nvgpu, `nv_imx219`, `tegra-camera`, and `imx296` in `modules.dep`;
  - `dpkg -C` clean.
- The script's own pre-reboot check then stopped silently: `strings | grep -m1` under pipefail + `set -e`. It is
  fixed, and the checks were run by hand.
- `--hold`: all 65 packages are held, so `apt upgrade` cannot repeat 2026-09-23. Release one with
  `apt-mark unhold`.
- After the reboot: `uname -r` is `5.15.185-tegra`, and `/etc/nv_tegra_release` reads R36 REVISION 5.0.
- **The bootloader was not rolled back.**
  - The 36.5.0 capsule was staged (`Trigger Capsule update is done`) but did not take effect.
  - Every boot since then (15:13, 15:23, 15:44, 16:11) logs, from nv-l4t-bootloader-config,
    `System version(deb version): 2360576, BSP version(QSPI version): 2360578`, i.e. 36.5.0 against 36.5.2.
  - The UEFI reports `36.5.2-gcid-46426093` (`/sys/class/dmi/id/bios_version`) and runs from slot B.
    `SystemFwVersions` gives slot A 36.5.0 (left from before 2026-09-23) and slot B 36.5.2.
  - `/boot/efi/EFI/UpdateCapsule` is empty again. The boot service updates only when the deb version is higher than
    the QSPI one, so it will not retry.
  - So the rootfs, kernel and userspace are R36.5.0, and the bootloader is R36.5.2. Every test below ran on that mix.
    `sudo nvbootctrl dump-slots-info` shows the capsule status.

**The camera slots.** The user had lost track of which slot is which, and the IMX296 was in CAM0. There were four boots
on R36.5.0:
- **15:13:** still the combined IMX219-A + IMX296-C overlay. Neither sensor probed: `imx219 10-0010` and
  `imx296 9-001a` both returned -121.
- **15:23:** jetson-io's dual-IMX296 overlay. It found the IMX296 in CAM0 (`10-001a`) but got no frames.
- **15:44:** the camera was moved to CAM1, still on the dual overlay (`9-001a`).
- **16:11 onwards:** `imx296-cam1.dtbo`.

This is how the device tree maps the slots:

| jetson-io | Linux I²C bus | DT node | CSI port | Lane polarity |
|---|---|---|---|---|
| **CAM0** | `i2c-10` | `cam_i2cmux/i2c@0` | `serial_b` | 6 |
| **CAM1** | `i2c-9` | `cam_i2cmux/i2c@1` | `serial_c` | 0 |

The I²C bus numbers are assigned at boot and depend on the overlay. With a two-camera overlay (the dual one, or the
combined IMX219-A + IMX296-C), CAM0 is `i2c-10` and CAM1 is `i2c-9`. With `imx296-cam1.dtbo` alone, cam_i2cmux has
one port: CAM1 is `i2c-9`, there is no CAM0 bus, and `i2c-10` is the display's.

- **In CAM0**, the IMX296 was detected (its chip ID read fine) but delivered no frames. Argus timed out in
  `waitCsiFrameEnd`, and a raw `v4l2-ctl` stream got `uncorr_err: request timed out`.
- In CAM0 the sensor's register writes also failed at stream start (`8-bit write to 0x3008 failed: -121` /
  `imx296_set_group_hold: group hold control error`, 7 times). No boot with the camera in CAM1 has shown one.
- The CAM0 half of `imx296-dual.dtbo` is identical to the vendor's `imx296-cam0.dtbo`. So the device-tree settings
  are the vendor's own, and the slot's connection or ribbon is the likelier cause. It was not pursued.
- **In CAM1**, where it worked in July, it works: overlay `imx296-cam1.dtbo`
  (`'2=Camera IMX296-C Cam1'`), sensor `9-001a`, IMX296LQ colour, `/dev/video0`.
- The UGREEN is now `/dev/video1`. v0.4 finds it by-id.
- The IMX219 is not connected. The combined `imx219-A-imx296-C.dtbo` (IMX219 in CAM0, IMX296 in CAM1) is still in
  `/boot` for when it goes back.

**Tests on R36.5.0, all PASS** (build `bin/build_20260928_104806`, unchanged; container builds do not depend on the
host BSP):
- **`csi_test 0`: 5/5.**
  - Pipeline start: 182 frames.
  - 1080p30: 91 frames in 3 s.
  - 720p60: 181 frames in 3 s.
  - Attributes changed mid-stream: accepted.
  - Argus rapid restart: OK.
- **`csi_rtp_test`:** Camera_CSI with the libnetwork x264 → RTP branch. 124 H.264 access units in 8 s, then a clean
  stop, with no WARN.
- **`camera_gst_test`:** 121 `ok` lines. Both dictionary lines pass: USB 12 entries, CSI 10 against a live Argus.
- **`usb_test`:** all PASS, `Controls restored as found: PASS`, and an empty `--list-ctrls` diff. It picked the
  UGREEN at `/dev/video1` by itself.
- **`scan_cameras`:** 2 cameras. `/dev/video0` is Jetson CSI (14 attributes, 1 format); `/dev/video1` is the USB
  UGREEN (20 attributes, 28 formats).
- **`record_test`:** Parts A and B.
- **v0.4 (the service, from boot):** it recorded `/dev/video1` in 3-minute segments. The run the user stopped at
  16:29 took 31,855 frames and exited 0.
- **CUDA / TensorRT (container):** `torch.cuda` sees the Orin, and the lane engine (`culane_res18_fp16`) runs at
  14.9 ms mean in `trtexec`.

**The new teardown on nvarguscamerasrc** (the check that needed CSI; HANDOVER asked for "no `EOS not delivered`
WARN, as long as before"). Camera_CSI `stop()` on the IMX296, timed with a throwaway harness against the build
before the fixes (`ef45a22`) and the current one:

| Rounds | eosTimeoutMs | Before (`ef45a22`) | Now (`1888a94`) |
|---|---|---|---|
| Capture only (×5, ×3) | 5000 / 1000 | 5.040–5.047 s | 5.037–5.049 s |
| Plus a leaky `videorate ! appsink` branch nobody pulls (×3) | 5000 | 5.110–5.111 s | 5.110–5.125 s |
| The same | 1000 | 1.078–1.085 s | 1.077–1.086 s |

- No WARN or ERROR in any of 28 stops, in either build. Item R's false WARN did not trigger.
- **Capture only: about 5 s inside nvarguscamerasrc.** Changing `eosTimeoutMs` or `stateChangeTimeoutMs` (2000)
  does not change it. A plain `gst-launch-1.0 nvarguscamerasrc ! fakesink` also takes about 5 s to stop, with or
  without `-e` (9.69 s against 9.70 s total for 4 s of streaming). That is Argus's own shutdown cost:
  nvargus-daemon logs `PowerServiceCore:handleRequests: timePassed = 5027` (±20 ms) as each streaming session ends.
  It does not match the July note, though. That ~5 s block came and went with the daemon's state, in the CANCELLED
  path. This one comes on every stop, and nothing logged CANCELLED.
- **With a branch appsink nobody pulls, the stop waits out `eosTimeoutMs`.** appsink holds EOS back until it is
  drained, so the pipeline EOS never comes. It waits silently, because the EOS *send* did return. This predates the
  pass.
- **Unexplained:** those branch rounds then reach NULL quickly (77–86 ms after the EOS wait at `eosTimeoutMs`
  1000, 110–125 ms at 5000), without the 5 s Argus cost. It is noted here, not investigated.

**Not run:** `commlink_test`. The MKR and the C3 were unplugged: no `/dev/serial/by-id` at all, and v0.4 logged
"telemetry bridge (ESP32-C3) not present — retrying in the background". Plug them back in for the car.

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

## Done 2026-09-28 (Jetson coder, afternoon): the review fixes and the libcommlink pass on the Jetson

Built from `1888a94` in the l4t-ml-gpio container with plain `make` into `bin/build_20260928_104806`. The build is
clean; the only warnings are the two known `read()` ones.
- `dashcam-v04` was stopped throughout and is left stopped. Its next start runs this build.
- The UGREEN's controls match this session's first `--list-ctrls` snapshot.
- The SD card (`mmcblk0p1`) is still not mounted. That is left for later, on request.

### 1. CSI driver: not fixable from here

**Superseded on 2026-09-30:** the Jetson went back to 5.15.185, and the IMX296 works in CAM1 (see "Done 2026-09-30").
What follows was true on 5.15.199.

It needs sudo, and the vendor's source package, which is not on the Jetson.

**The old unit.** It is `imx296-reload.service` (enabled), byte-identical to the vendor's file. It is committed
for reference as `docker_dev/imx296-reload.service.jetson` (2026-09-30, with the rollback's HANDOVER).
- After boot it sleeps 5 s, stops Argus, runs `modprobe -r imx296` then `modprobe imx296`, and starts Argus again.
- On 5.15.199 the `modprobe -r` fails (`modprobe: FATAL: Module imx296 not found.`), so the unit exits with Argus
  stopped. `nvargus-daemon` has been inactive since boot (2026-09-27 17:42).
- It never tried to load the 5.15.185 module: 5.15.199 has no `imx296` at all.

**The module name.** The module is `imx296`, not `nv_imx296`. `/etc/modules-load.d/imx296.conf` also loads it at
boot.
- `docker_dev/csi_driver.sh check` with the default name reports
  `no nv_imx296 under /lib/modules for any kernel (running 5.15.199-tegra)`.
- With `CSI_MODULE=imx296` it reports
  `cannot load: no imx296 for the running kernel 5.15.199-tegra; built for: 5.15.185-tegra`, exit 2.
- `test_csi_driver.sh` passes. Run it with `bash`, because it is committed without the executable bit.

**No rebuild was possible.** The vendor package on the Jetson
(`~/Downloads/IMX296_driver/cam-imx296raw-trigger/1-1jetson_orin_nano_driver/`) has prebuilt `imx296.ko` for
5.15.148 and 5.15.185 only. Its README says another kernel needs "the source package", which is not here.

**A second cause that the review missed: the camera overlay is not booted.**
- `/boot/extlinux/extlinux.conf` has `DEFAULT primary`, which has no overlay. The IMX219-A + IMX296-C overlay is
  only in the `JetsonIO` entry.
- The kernel upgrade on 2026-09-23 (10:32) switched `DEFAULT JetsonIO` to `DEFAULT primary` (nv-update-extlinux).
  This was seen in `extlinux.conf.nv-update-extlinux-backup`, but the rollback's own nv-update-extlinux run
  overwrote that file on 2026-09-30 14:54. The 09-23 result is kept as
  `/var/backups/l4t-rollback-20260930-141437/extlinux.conf`.
- `/proc/device-tree/tegra-camera-platform` does not exist, so no sensor would probe even with a matching module.
- `nv_imx219` exists for 5.15.199 (`updates/drivers/media/i2c/`), so the IMX219 needs only the boot entry.
- Booting 5.15.185 instead is not an option: it has no `uvcvideo.ko`, so the UGREEN would disappear (seen
  2026-09-23).

**For the user (sudo), in this order:**
1. Back up `/boot/extlinux/extlinux.conf`, set `DEFAULT JetsonIO`, and reboot. The IMX219 should then enumerate.
2. Get the vendor's IMX296 source package, build it for 5.15.199
   (`make -C /lib/modules/$(uname -r)/build M=$PWD modules`), install the `.ko` under
   `/lib/modules/$(uname -r)/updates/`, then run `sudo depmod`. `CSI_MODULE=imx296 docker_dev/csi_driver.sh check`
   must then say `would load`. Only the colour overlay is configured, so `imx296.ko` alone is enough; the vendor's
   patched `tegra-camera.ko` is only for mono Y10.
3. Replace `imx296-reload.service` only after items K–P below are fixed.
4. Decide between holding the kernel packages and using DKMS.

Until step 1 is done, `csi_test` and v0.3 cannot run, so neither can the CSI half of the new teardown.

### 2. camera_gst_test: PASS

121 `ok` lines in 11.9 s. It passed 14 of 14 runs, 3 of them with every core busy. On GStreamer 1.20.3:
- **Stalled branch:** `stop()` takes 0.25 s, and `close()` from ERROR takes 0.25 s.
- **Wedged source:** 0.85 s. The stuck pipeline is released 0.02 s after its thread frees.
- **Finite stream:** ERROR, and `close()` takes 0.00 s, with and without a puller.
- **Disabled branches:** the videorate and x264enc shapes both start, and no GAP reaches either. Both deliver once
  enabled.
- **Dictionary:** all 12 USB entries check out against v4l2src, and all 10 CSI entries against nvarguscamerasrc.

**The reviewer's reproductions, redone on the real UGREEN.** These use Camera_USB, v4l2src and 1080p30 MJPEG. The
harness is a throwaway program built against each build's objects, and is not committed.

| Shape | Before (`ef45a22`) | Now (`1888a94`) |
|---|---|---|
| A blocking branch that is never pulled, `eosTimeoutMs=250` | `stop()` hung (killed at 30 s) | 0.261 s, one `EOS not delivered ... flushing the branches` WARN |
| A videorate branch that starts disabled | ERROR: `did not reach PLAYING within 5000 ms` | PLAYING in 0.60 s; 10 of 10 frames once enabled |
| A normal start/stop, capture only (×3) | 8–10 ms | 9–12 ms, no WARN |

The finite stream was run only on videotestsrc, in `camera_gst_test`; a real v4l2src does not end.

### 3. usb_test: all PASS

`Controls restored as found: PASS`, and the before/after `--list-ctrls` diff is empty.
- Test 3: `brightness queued 31 (range 1..64, was 32), device reads 31`. On 1.20.3, extra-controls applies a value
  queued before `start()`.
- Test 4: `device reads 32`.
- Test 4b: three `put back to N, reads N (OK)` lines.

### 4. Focus

`focus_test`: 22 checks PASS.

This build's v0.4 ran for 25 s with a temporary copy of the deployed `dashcam.xml` that adds `FocusMode=fixed` and
`FocusAbsolute=300`. The deployed file was not touched.
- At start: `focus: fixed at 300 on /dev/video0 (autofocus off; range 0..1023)`.
- Mid-run: `focus_automatic_continuous=0`, `focus_absolute=300`.
- At SIGINT: `focus: handed back to the camera on /dev/video0 (autofocus on)`, and `rc=0`.
- `focus_automatic_continuous` reads 1 again, as it did before the run.
- One difference: `focus_absolute` stays at 300, where it was 449 (item X). The camera was found with autofocus on,
  so by design only the autofocus state is handed back. On the UGREEN, `focus_absolute` is the stored manual
  position and does not follow the autofocus. A later switch to manual would therefore move the lens to 300. It was
  put back to 449 by hand.

### libcommlink

- **`commlink_sim_test`: PASS.** 73 checks in 5.4 s. It passed 16 of 16 runs, 5 of them with every core busy.
- **HostProtocol.h:** both copies have the same md5 (`79ca61e3…`), and the C3 host suite passes 79/79. The C3
  needs no reflash.
- **`commlink_test` with the C3.** The cable pull was simulated by de-authorising the C3's USB device for 6 s
  (`echo 0`, then `echo 1`, to `/sys/bus/usb/devices/1-1/authorized`). That is a logical unplug on the host side;
  the C3 stays powered.
  - **With the C3's by-id path (what v0.4 uses):** on the unplug, a hangup and `bridge port closed`. It reopened
    1.0 s after the replug with a new HELLO and `reconnects=1`. Every check was `[ OK ]`, there was no NACK, and it
    exited 0.
  - **Auto-discover (`commlink_test "" 50`): FAIL.** The build before this pass fails the same way, so 68fcceb did
    not cause it (item U).
    - This mode sets `allowAcmFallback=true`. On this rig the other ACM device is the MKR's USB console,
      `ttyACM0`.
    - After the unplug, it probes the MKR console: its HELLO bytes go into that console, and it reports
      `did not identify itself in 2000 ms`.
    - The close then takes 30.9 s. The kernel's `closing_wait` waits for the unsent bytes on a port nobody reads.
    - After the replug, the probe cursor points at `ttyACM0` again, because the candidate list changed shape. That
      costs a second 30 s probe, and the run ends before it reaches the C3.
    - `stop()` then waits for the 30 s close, and `timeout` killed it 20 s later.
    - v0.4 does not hit the candidate problem, because it uses by-id only (`allowAcmFallback=false`).
- **v0.4 start log:** `bridge fw 1.0 proto 7`, and `[c3] bridge ready`. The overlay's vehicle fields stay dashed:
  indoors the MKR reports `sats=0 fix=0`, and there is no speed with the car off (the IMU is live, |a| 9.7 m/s²).
  Seeing them fill in needs open sky or the car.

**Also run, all PASS:** `--self-test`, `config_test`, `bridge_overlay_test`, `exposure_test`, `liblog_test`,
`record_test` Parts A and B, and `scan_cameras` (output identical to the 2026-09-27 build's).

## Next: items from the Jetson run and a second review

These come from the Jetson run plus a second 8-agent review of `68fcceb..1888a94`, where each finding went to a
skeptic told to refute it. **Deployed v0.4 cannot reach any of them.** The letters continue from the first
review's A–J.

**CSI loader (`aa9deb1`): fix these before installing `csi-driver.service`.**
- **K. The default module name is wrong.**
  - `csi_driver.sh:39` and `csi-driver.service:31` default to `nv_imx296`; on this Jetson the module is `imx296`.
    Shipped as is, the unit fails at every boot.
  - Also correct the root-cause text in the script header (`csi_driver.sh:6-8`) and the unit
    (`csi-driver.service:3-4`): on 5.15.199 the old unit failed at `modprobe -r`, and never loaded a 5.15.185
    module. On 5.15.185 it works. (The status row is corrected.)
- **L. Starting it by hand deadlocks.**
  - The unit is `Type=oneshot`, which has no start timeout on this systemd (249), and `Before=nvargus-daemon`.
  - With Argus active, `systemctl start` (or `restart`, or `enable --now`) makes the script stop Argus.
  - Its EXIT trap then runs a blocking `systemctl start nvargus-daemon`, and that job waits for csi-driver's own
    start job. Both hang, and Argus stays down. Boot is not affected, because Argus is not active yet.
  - Fix: use `systemctl --no-block start` when running under systemd (`$INVOCATION_ID`).
- **M. It is not a like-for-like replacement.**
  - The vendor unit is a post-boot reload: `After=nvargus-daemon`, a 5 s sleep, `modprobe -r` plus `modprobe`, then
    Argus restarted. It exists for a black screen when Argus starts before the sensor is ready.
  - `imx296.conf` already loads the module at boot, so `csi-driver.service` would always find it loaded and do
    nothing.
  - Suggestion: a `reload` mode that checks first, then does what the vendor unit does, installed
    `After=nvargus-daemon`. That also avoids L.
- **N. The vermagic check can judge the wrong file.**
  - It reads the first file by name order, so `kernel/` comes before `updates/`. That is not the file modprobe
    loads. It also cannot tell when depmod has not run.
  - Fix: use `modinfo -k "$KREL" -F filename "$MODULE"`.
- **O. A failed Argus restart still exits 0.** If Argus cannot be started again, the script still exits 0.
- **P. `test_csi_driver.sh` is not executable.** It is committed without the executable bit (mode 100644), but
  the procedure and its own usage line run it directly.

**Camera_GST (`f03a13f`).** These affect v0.2/v0.3 only; v0.4 does not use Camera_GST.
- **Q. The shutdown bound does not cover a sink stuck in a syscall.**
  - The branch flush runs on the calling thread, before the abandon timer starts.
  - FLUSH_START needs the sink's preroll and stream locks. A filesink stuck in `write()` holds them: the SD-card
    stall that the open list names.
  - Fix: flush on a helper thread under the timer, or say in the header that the bound excludes this case. No app
    has a blocking branch today.
- **R. A false WARN after an early ERROR.**
  - When an ERROR ends the EOS wait early (the Argus CANCELLED case), teardown flushes at once and logs
    `EOS not delivered within <eosTimeoutMs> ms`. The time in that line is not the elapsed time, and nothing has
    stalled.
  - The CSI check expects normal stops to log no such WARN, so this matters when CSI is verified.
  - Fix: after an ERROR, wait for the send until the deadline, and log the elapsed time.
- **S. `stop()` overlaps with `close()` and `start()`.** This predates the pass, but the window is now longer.
  - `stop()` reports OPEN before its longer teardown starts.
  - A concurrent `close()` or `start()` then runs a second `teardownPipeline()`, which is not a no-op: it touches
    the tee pads and the handles, and `teePads_` without the lock. The comment saying the second call "exits
    immediately" is false.
  - Fix: a `tearingDown_` flag that `close()` and `start()` wait on, or document that the three must not overlap.
- **T. "Auto controls first" holds only for `start()`'s batch.**
  - `camera_attributes.xml:21-22` states it without that qualification.
  - A write while running carries only its own control, so `auto_exposure=1` must be set before
    `exposure_time_absolute`.

**libcommlink (`68fcceb`, `0f548d7`).**
- **U. Discovery in fallback mode picks the wrong device after a replug** (measured above; it predates the pass).
  - Fix the close: `tcflush(TCOFLUSH)` before `close()` in `Uart::close()`, or `closing_wait` NONE through
    `TIOCSSERIAL` for ACM ports.
  - Fix the order: try the by-id matches before rotating through fallback nodes, or reset the cursor when the list
    changes.
  - Give `commlink_test` a by-id-only option, or document that auto-discover fails on a rig with the MKR console
    attached.
  - The review judged v0.4 not exposed to the 30 s close: the C3's HWCDC ISR always drains the USB FIFO, so its
    writes complete even if `loop()` hangs.
- **V. A log call under `m_statsMtx`.** The malformed MSG_TELEMETRY path (`libcommlink.cpp:446-451`) calls the log
  callback with that mutex held, so a callback that calls `stats()` deadlocks. It is the same class of bug 68fcceb
  fixed for the port lock. v0.4's callback does not call `stats()`.
- **W. `commlink_sim_test` discovery depends on pty numbers.**
  - It expects name order, but `enumerate()` sorts the resolved `/dev/pts` paths as strings.
  - So it fails when the two pty numbers straddle a power of ten, e.g. 9 and 10.
  - Fix: sort the by-id entries by name, then resolve them.

**Focus (`3e3dc4f`) and usb_test (`782bdfd`).**
- **X. The stored manual position is not handed back.** For a camera found with autofocus on, restore the lens
  position too, when it could be read: write it while autofocus is still off, then turn autofocus back on.
- **Y. A failed rollback is not reported.**
  - `libcamera_focus.cpp:75` ignores the rollback's result. If the lens write fails and autofocus cannot be turned
    back on, autofocus stays off, while v0.4 logs "left to the camera".
  - It needs two control writes in a row to fail with the camera still attached.
- **Z. An aborted usb_test leaves the camera changed.** It restores the controls only when `main()` finishes, so
  Ctrl-C or a crash leaves them changed. Fix: print the restore command at start, or set a signal flag checked
  between tests.

**Refuted by the skeptics, so not items:**
- Attributes are not re-applied across `stop()`/`start()`. No contract promises that, and 5aa56fd behaved the same.
- An abandoned pipeline holds the device. That is documented, and the source is wedged anyway.
- The 30 s close in v0.4. See U.

## Done 2026-09-28 (cloud): the libcamera review, and the Jetson coder's hardware findings

Each fix has a `camera_gst_test`, `focus_test` or `test_csi_driver.sh` check that fails with the old code. In
camera_gst_test the fixes were reverted one at a time, and 15 of 17 such mutants fail it. The two that pass are
guards that no test can reach now:
- teardown skipping the second EOS for a stream that already ended: videotestsrc lets a second EOS through fast;
- `close()` waiting for `start()`: ERROR can no longer appear while `start()` builds.

ASan, UBSan and LSan are clean. TSan is clean with the uninstrumented GLib/GStreamer libraries suppressed, and it
does report the old unlocked branch-list clear. The Jetson run is in "Done 2026-09-28 (Jetson coder, afternoon)"
above.

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
  - The deployed `dashcam.xml` still carries `<Encoder>`, `WarmupFrames` and `RecordWidth`/`RecordHeight` (all
    removed in `409c87a`). Expect one
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
- **Proposed after the libcommlink pass, not done** (each changes behaviour on the car, so it is the user's call):
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

## Environment (the Jetson rig, as of 2026-09-30)

Collected from the machine, not from memory: `/etc/nv_tegra_release`, `dpkg-query`, `arduino-cli core/lib list`,
and the tools' own `--version`. Re-collect after an upgrade.

**Hardware**
- **Jetson:** NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super. 6 cores, 7.4 GB RAM, 937 GB NVMe
  root. No RTC coin cell, so the clock comes from NTP or GPS. It runs on **its own battery** in the car.
- **CSI camera:** IMX296LQ (colour, global shutter, 1456×1088 @ 60) in **CAM1** (`i2c-9`, `serial_c`), `/dev/video0`,
  Argus sensor-id 0. The vendor prebuilt driver is for 5.15.185; overlay `imx296-cam1.dtbo`. No IMX219 is connected.
- **Camera:** UGREEN Camera 4K, USB UVC, `eba4:6579`, `/dev/video1` since the CSI camera came back. It records
  1080p30 MJPEG (`FormatIndex 4`), pinned by `/dev/v4l/by-id`. Controls are standard UVC only (focus, exposure,
  gain, backlight compensation). Frames from the evening of 2026-09-26 are upright.
- **Telemetry master:** Arduino MKR Zero (SAMD21G18A), **powered from the car's OBD2 port**, with USB to the Jetson
  as the dev console.
  - **CAN:** MKR CAN Shield, an MCP2515 with a 16 MHz crystal, 500 kbit/s, sniffing the Honda Brio (map `brio`,
    id 0x0B, on the SD card).
  - **I2C:** at 400 kHz. BNO055 IMU at 0x29 on a **breadboard**, INT pin not wired. SparkFun u-blox GNSS at 0x42
    over ESLOV.
  - **SD card:** on SPI1, holding the CAN map and the IMU calibration profile.
- **Bridge:** Seeed XIAO ESP32-C3. It talks to the MKR over UART1 at 115200 and to the Jetson over native USB CDC.

**Jetson host**
- JetPack 6.2.2 (`nvidia-jetpack 6.2.2+b24`), L4T R36.5.0 rootfs (`/etc/nv_tegra_release` R36 REVISION 5.0), kernel
  5.15.185-tegra, Ubuntu 22.04.5 LTS.
- The QSPI bootloader/UEFI still reports 36.5.2 (`/sys/class/dmi/id/bios_version` = `36.5.2-gcid-46426093`, running
  from slot B).
- The rig was rolled back from R36.5.2 on 2026-09-30, and the 65 NVIDIA packages are on `apt-mark hold`.
- CUDA 12.6.11, TensorRT 10.3.0.30 (`libnvinfer10`, built for CUDA 12.5), cuDNN 9.3.0.75, VPI 3.2.4, GCC 11.4.0,
  Python 3.10.12. The rollback did not change these; only their JetPack meta packages moved.
- GStreamer 1.20.3, v4l-utils 1.22.1, Docker 29.8.1.
- `nvargus-daemon` and the vendor's `imx296-reload.service` are active. The boot entry is `DEFAULT JetsonIO` with
  `imx296-cam1.dtbo`. A kernel package change resets `DEFAULT` to `primary` (nv-update-extlinux); set it back
  afterwards. The v0.4 container does not mount `/tmp/argus_socket`, so its start log still shows two harmless
  `(Argus) Error FileOperationFailed` lines (`Connecting to nvargus-daemon failed`, then
  `Cannot create camera provider`).
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
bash docker_dev/test_csi_driver.sh                   # csi_driver.sh against stubs (not executable yet, item P)
CSI_MODULE=imx296 docker_dev/csi_driver.sh check     # on the Jetson: the IMX296 module vs the running kernel (item K)
$B/commlink_sim_test                                 # CommLink + libuart against a simulated C3 on a pty: no hardware
$B/commlink_test "" 30                               # the real C3 bridge (stop dashcam-v04 first)
$B/record_test                                       # Part A needs GStreamer base/good/bad/ugly plugins; Part B a camera
$B/usb_test                                          # a USB camera (stop dashcam-v04 first); puts its controls back
$B/scan_cameras                                      # lists the cameras, their controls and formats
```

`usb_test` puts brightness, gain and backlight_compensation back as found (`782bdfd`). It passed on the Jetson on
2026-09-28 and again on 2026-09-30, with an empty before/after `--list-ctrls` diff. To check by hand, diff
`v4l2-ctl -d /dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0 --list-ctrls` before and after the
run: `/dev/video0` is the IMX296 now.

The Wire changes have no host test: they are register-level SAMD21 code, proven only on the rig
(BusFaultInjection). Running that is the Jetson coder's job.

Where each test can run:
- **Anywhere, cloud included:** the host suites, `mutations`, `--self-test`, `config_test`,
  `bridge_overlay_test`, `camera_gst_test`, `focus_test`, `commlink_sim_test`, `test_csi_driver.sh` and
  `record_test` Part A.
- **Jetson only:** CUDA targets, `record_test` Part B, `usb_test`, `csi_test`, `scan_cameras` (with a camera),
  `commlink_test` (with the C3), `csi_driver.sh`, and everything on the rig or the car.
