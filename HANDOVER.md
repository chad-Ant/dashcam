# Handover — branch `camera`, 2026-10-12

This file has three parts:
- **Start here:** the rig as it is now, the state of every workstream, and the open work in priority order.
- **Reference:** who does what, the environment, the tests, a rig procedure and the code backlog.
- **Log:** dated sections, newest first, kept as written at the time. Later corrections are marked inline. Where a log
  section disagrees with Start here, Start here is current.

# Start here (2026-10-12)

## The rig now

| Item | State |
|---|---|
| **Attached now** (boot of 2026-10-12 11:25, on the bench, not in the car) | **The UGREEN and the CYD only.** The UGREEN is `/dev/video0` while no CSI camera is present (v0.4 pins it by-id, so that is fine). The CYD is `/dev/ttyUSB0` since the out-of-tree `ch341.ko` was installed and brltty-udev masked (2026-10-12 log section). The CSI camera, MKR and C3 are unplugged. Reconnect them, reseat the CSI ribbon, power-cycle, and check `csi_test 0` before relying on anything below. **Update 2026-10-25 (bench):** the IMX296 (`/dev/video0`, Argus id 0), the UGREEN (`/dev/video1`), the MKR (`/dev/ttyACM0`, its console reports GNSS and IMU up) and the C3 (`/dev/ttyACM1`) were all attached and worked for the lane test suite; `dashcam-v04` was inactive throughout. **Update 2026-10-26 (bench):** same devices plus the CYD (`/dev/ttyUSB0`). **Update 2026-10-27 (bench):** the degradation hunt ended at the first fault (06:02); the IMX296 tests that followed (2026-10-27 log section) left driver candidate C loaded: reload the stock driver (`sudo docker_dev/csi_driver.sh reload`) or reboot before using the CSI camera. `dashcam-v04` stayed stopped. |
| Platform | L4T R36.5.2 / JetPack 6.2.3+b81 / kernel 5.15.199-tegra, UEFI 36.5.2 (upgraded 2026-10-01). The 65 NVIDIA packages are held. |
| CSI camera | IMX296 in CAM1 (`/dev/video0`, Argus id 0). Rebuilt `imx296.ko` for 5.15.199 (still built with `#define DEBUG 1`). Overlay `imx296-cam1.dtbo` caps gain at 200 (20 dB) and exposure at 1000 lines (14.8 ms). ISP override **c8_sh15** (sha256 `43ec0c81…`). `csi-driver.service` is enabled and replaces `imx296-reload.service` (disabled). **At low analogue gain the sensor resets itself** mid-stream: it drops off I2C for ~170 ms and comes back with its power-on defaults, and Argus reports `waitCsiFrameStart Timeout … Sensor GUID 0 is in error state`. The hazard rises steeply as the gain falls: GAIN register 16 (Argus 1.0, the DT floor) resets within seconds to a minute; in the dark none was seen at 64 and above, but in midday sun the module also reset at 64 and on stream-on. Heat is the main factor: cooled to a ~34 °C die it ran gain 16 clean (2026-10-29). Bright scenes hit it because AE drives the gain to the floor. Light, exposure, register traffic, GPU load, frame rate, module age and the driver build are ruled out; the cause is on the module (power or bias margin, unit or design; not testable without hardware). **Mitigation, not a fix, in `Camera_CSI` since 2026-10-27:** every CSI pipeline sets `gainrange="4 12.5"` (register 64–200); a `gainrange` attribute replaces it. It passed a 30-min daylight `trtexec` soak that otherwise faulted within 1–3 min, but a midday two-camera `lane_drive` soak (run_018, camera in the sun) still reset 6 times at gain 64 plus 3 times at stream start; the harness kept CSI lanes at a mean 19.75 Hz through 9 container restarts. Expect CSI gaps on a sunny drive until the module is replaced or fixed. The 2026-10-26 "degraded driver state, reboot before a drive" diagnosis was wrong (2026-10-27 log section). Driver candidate C (no group hold, no gain delay) is not a fix; do not install it. **Staged, not installed:** a debug-off module and two overlay candidates, with install/rollback scripts (`~/drive_logs/tools/imx296_driver/staging_20261010/`, 2026-10-11 log section). |
| UGREEN | `/dev/video1` next to the CSI camera (`/dev/video0` without it), pinned by `/dev/v4l/by-id`. Deployed config: `FocusMode=fixed`, `FocusAbsolute=690`. `usb_test`, `record_test` Part B and `scan_cameras` pass on it with the 2026-10-11 build (2026-10-12). It has **no** `exposure_dynamic_framerate` control. |
| v0.4 | `dashcam-v04.service` is **enabled but inactive**. Its last run (2026-10-09 16:01–17:33) found neither the UGREEN nor the C3, retried every 3 s, and was stopped cleanly at 17:33:36 (`log_20261009_160103.txt`). **The newest build is now `bin/build_20261011_003045`**, which holds the uncommitted 2026-10-11 library changes (behaviour-neutral, all tests pass); the service runs it at its next start. `bin/build_20261011_002908` is a test build of the old float code; delete it through the container if wanted. The deployed `/user/output/configs/dashcam.xml` has `TelemetryCsv` on, logs at 16 MB × 8, and since 2026-10-10 no obsolete settings (backup `dashcam.xml.bak-20261010-before-obsolete-cleanup`). Its wrong "autofocus stuck at 512" comment was corrected on 2026-10-12 (comment only; backup `dashcam.xml.bak-20261012-before-focus-comment`). |
| MKR Zero | Production raw-CAN build (`3a702a8`). It boots into DISCOVER: listen-only, accept-all, raw `F`/`FS` lines on native USB, no decode. Its millis() clock is **not** held to the crystal (+1300 → −140 ppm within a drive; 2026-10-11 log section). |
| ESP32-C3 | Bridge firmware 1.0; `HostProtocol` and `CommProtocol` are at 0x07. |
| Lane engine and image (2026-10-25) | `models/culane_res18_fp16.engine` is now a **native FP16 build made on this Jetson** (413 MB, sha256 `d048fbde…`, 9.4 ms GPU vs 14.0 ms); the old 803 MB engine (FP32 head, built on another device) is kept beside it as `.prev803_bak`. It is also v0.2/v0.3's engine. `models/` is gitignored: the build command is in `liblanedetector.h`. `l4t-ml-gpio:latest` was rebuilt with matplotlib, addict and psmisc (`24110f6`); the previous image is tagged `build_20261025_143839`. The newest build is `bin/build_20261026_053024` (all targets, including `lane_drive` and `lane_status`; `bin/build_20261025_195258` is the P5a build). Run data: `/media/jetson/backup/lane_tests/`, replay packs `/media/jetson/backup/lane_replay/`, goldens and tiny engines in `~/drive_logs/lane_golden`, `~/drive_logs/lane_engines`. |
| Drive logging | `~/drive_logs/tools/drive_session/` (outside the repo). The `@reboot` crontab line is **disarmed**. Since 2026-10-10: `drive_watch.sh` watches the raw-CAN files, tegrastats runs every 1 s, and `drive_session.sh stop` refuses while the car is moving (`--force` overrides). |
| Storage | `/media/jetson/backup` sits on the root NVMe: 428 GB free. |
| Uncommitted | `peripherals/mkr_zero/tools/canstream/` (logger hung-disk bound, tests, README); `peripherals/mkr_zero/libraries/EncoderPan/` (standalone servo/encoder prototype); `lib/libcan/libcan.cpp`, `lib/libgpio/libgpio.cpp`, `lib/libcamera/libcamera.{h,cpp}`, `lib/libconfig/libconfig.{h,cpp}`, `src/tests/test_libconfig.cpp`; since 2026-10-12: `tools/isp_tuning/chart/v2/` (capture helper, tests, docs), `peripherals/cyd/` (the dashboard app the user copied in from a colleague's uncommitted work, with my fixes; bring-up sketch fixes; CH340 driver script; README), `docker_dev/Dockerfile`, `build.sh` and `install_samd_aarch64.py`; this file. Since 2026-10-13/14: `peripherals/es3c28p/` (colleague's board port + firmware video side; my protocol headers, vectors, tests, Python playback), `lib/libvideopreview/`, `lib/libes3display/`, the preview tap in `lib/librecord/`, `<DisplayPreview>` in `lib/libconfig/`, `src/dashcam_v0_4.cpp`, the Makefile, their tests, and the Dockerfile JPEGDEC pin (image rebuilt). |

## Workstreams

| Workstream | Commits | State | Next |
|---|---|---|---|
| v0.4 recorder (UGREEN) | `7293abe` and earlier | Segmented MJPEG passthrough, auto night mode, fixed focus, every telemetry field in the `.ass` and in CSV. The 2026-10-03 night drive recorded cleanly; the overlay matches the telemetry and every segment has its `.ass`. | Start the service before a drive. |
| UGREEN image | — | Fixed 690 is verified, and autofocus also works (parked). Night blur is exposure (33–50 ms at 20 fps), not focus. 45% of night driving frames are below luma 35. The camera has no `exposure_dynamic_framerate` control, so the proposed A/B with it is impossible; v0.4's frame-rate-priority loop is the existing way to cap exposure (`libcamera_exposure.h`). | Autofocus while driving is untested. |
| CSI IMX296 | — (driver and ISP files live outside the repo) | c8_sh15 is clean indoors. At night it is gain-limited by the device tree: noisy and banded on unlit roads, 18.3 MB/s. **The mount is loose: 23 pose changes in 91 min**, mostly on road shocks. Candidates for the gain/exposure caps and the debug flood are staged. | Re-mount rigidly; install cand1 and A/B (sudo); then the debug-off module. |
| MKR raw CAN over USB | `3a702a8`, `9b5c00e` | Bench-proven: 0 loss at 2400 and 4400 fr/s. The production build is on the MKR. Orin logger and offline decoder in `peripherals/mkr_zero/tools/canstream/`; the logger is now bounded under a hung disk (uncommitted). | In-car loss test. |
| Turning demo, dead reckoning, EKF | `cd89438`, `8838ead` | Read-only demo plus a C++ library. Synthetic and decoder-parity tests only. The 2026-10-03 "clock break" it refuses is one stale frame, not a clock fault. | Measure `antenna_x_m`; run the measured course; let `fusion_input` drop stale rows (proposal). |
| IMX296 v2 chart | `7e45172`; helper uncommitted | Offline analyser; synthetic tests only. Printed by the user (banding, crushed shadow steps 0–32: fine for relative A/B, not absolute). Capture helper `capture_chart.py` (2026-10-12): UGREEN path run live, **CSI path not run**. | Capture the three sheets once the CSI camera is back. |
| GRF-250 ranging | `e46bf71` | Sensor parser, bench bridge sketch and Orin association library; host and container tests only. | Check the converter's IC and levels before wiring; bench test. |
| Encoder-supervised pan servo | Uncommitted | `peripherals/mkr_zero/libraries/EncoderPan`: fixed-point controller, AS5048B reader, gated PWM adapter and default-disabled MKR bench sketch. Hardware choice: Hitec D646WP + enclosed AS5048B. | Buy/check hardware, build sealed output-shaft mount and independent power cutoff; calibrate/bench before production transport integration. |
| CYD dashboard | `2f6b3f6`; Linux tooling uncommitted | Bring-up sketch (display + read-only SD); its wiring was fixed 2026-10-12 (backlight, SD bus, reset pin), still never run on a board. The image now builds it (Adafruit ILI9341 1.6.4); `peripherals/cyd/build_and_upload.sh` is the Linux twin. The host has no CH340 driver: `ch341.ko` built and verified, **not installed** (sudo). | Driver installed and flash backed up (2026-10-12). Dashboard app source now in `peripherals/cyd/` (copied in by the user); boot-crash fixed; **flashed 2026-10-13**, boots clean (SD 20 MHz, calibration loaded, splash 130 ms, audio starts, LDR logs). Still to do at the board: hear the alert, check touch and the LDR fade. |
| ES3C28P display preview | `c90778d` | Jetson side done 2026-10-14 (log section): wire protocol v1 frozen (shared headers + golden vectors), recorder preview tap, `<DisplayPreview>` config (off by default), `lib/libvideopreview` (6.5 ms per 1080p frame), `lib/libes3display` (USB link, tested against a fake board on a pty), wired into v0.4. A colleague wrote the firmware side (`peripherals/es3c28p`, 2026-10-13); compatible with it (firmware-logic fake + UGREEN end-to-end run). No board here yet. | Flash the board, add its udev alias, enable `<DisplayPreview>`, soak at 10 and 15 fps. |
| ES3C28P map | `5a04a0a`, `91f01aa`; panning uncommitted | Jetson side done 2026-10-14: map wire extension `Es3MapProtocol.h` (0x0A-0x0C, vectors), `lib/libnavigation` (GPS state, Mercator, SQLite tile cache, compositor, service with warm map), link page-driven camera/map switching, `<Navigation>` config, v0.4 wiring, launcher `navcache` mount. All suites pass; end-to-end with a map-capable fake board. **Extension v2, board-side panning** (smooth map, same day, uncommitted): the board scrolls a 512x384 background from 20 Hz poses interpolated between GPS fixes; Jetson and ES3C28P firmware sides implemented and software-tested (2026-10-15 handover entry); firmware supports retained backgrounds, separate strips and local markers, with whole-frame fallback. Reviewed/refactored 2026-10-16, including a default-page startup fix (review log below). No physical board validation. | Flash/bench both USB modes; verify transitions, panning, GPS states and actual throughput; then enable `<Navigation>`. |
| Stereo prototype | `e46bf71` | Orin-side 100 m feasibility tool; 30/30 tests in the container. | Evaluate with synchronised cameras (its README). |
| Lane detection test suite | `24110f6`, `650701a`, P5b in the commit after this file's 2026-10-26 update | Plan `~/drive_logs/lane_test_suite_plan_20261025.md` (lean cut chosen); results `~/drive_logs/lane_p0/P0_REPORT.md`. P0–P5a done on the bench (2026-10-25 log section): library test seams, `lane_test --unit/--gpu/--replay`, 12 robustness fixes, the `lane_drive` + `lane_run.sh` drive harness. **P5b done on the bench (2026-10-26 log section):** clip rings, tensor dumps, mount monitor, live CAN triggers, supervisor (restarts, degrade ladder, status), status display program, calibration, bench tools; B0 / B3m / 60-min soak pass (UGREEN), the 10-03 CSI slip is detected. Never run in the car. | The IMX296 module still resets in bright sun even with the gain floor (2026-10-27 log section): replace or fix the module before relying on CSI lanes, or accept CSI gaps; `lane_run.sh calibrate` in the parked car; then the first lane drive (day, then night). |
| IMU (BNO055) | `fda019c`, `ff69f84` | Corrupt bursts with the engine running. All 66 High-G latches of the night drive were real road shocks: at 2 g it counts road roughness (44/h), not incidents. | Wiring rework; decide the incident threshold. |

## Open work, in priority order

1. **Before the next drive:**
   - reconnect the UGREEN, MKR and C3, reseat the CSI ribbon, power-cycle; check `csi_test 0` 5/5 and the
     `/dev/v4l/by-id` and `/dev/serial/by-id` entries;
   - start `dashcam-v04` (or reboot);
   - **re-mount the CSI on a rigid bracket.** It moved 23 times on 2026-10-03, mostly at bumps: its roll against the car
     spanned about 20°, and single jumps reached 250 px. Level it within 2°, keep the dashboard to at most 30% of the frame, and take reference frames
     before and after the drive;
   - decide whether to re-arm the drive crontab. The session's stop guard now refuses a stop while the car moves;
   - **for a lane test drive** (`tools/lane_eval/DRIVE_PROCEDURE.md`): use a build from 2026-10-27 or later, whose
     CSI pipeline floors the IMX296 gain (below it the sensor resets itself in bright light; 2026-10-27 log section),
     with the stock driver loaded; in sun it still resets about every 5 min (the harness recovers in ~25 s). Then, parked on a marked road, `tools/lane_eval/lane_run.sh calibrate`: it proposes crops (CSI default rows
     544–864 of 1088 were mostly dashboard on 2026-10-03) and writes the mount template; start with
     `--csi-crop T,B --usb-crop T,B --mount-ref CAL_NNN/mount`. `lane_run.sh` refuses to start while `dashcam-v04`
     runs. The CYD shows the status pages only with its navigation firmware (the 2026-10-13 dashboard firmware does
     not answer the ES3 link); otherwise use the ES3C28P or `lane_run.sh status` over ssh.
2. **In-car raw-CAN loss test:** parked, engine on, about 10 min, against the CANRawLog baseline (≥ 0.4%).
3. **CSI night quality** (needs the user's sudo; procedure in
   `~/drive_logs/tools/imx296_driver/staging_20261010/README.md`):
   - `sudo ./install_candidate.sh --apply --module keep --dtbo cand1`, then reboot and run a night A/B;
   - after that, the debug-off module.
4. **Bench checks that need the MKR and C3:**
   - the MKR's DFLL/XOSC32K lock: log `SYSCTRL->PCLKSR`, `DFLLCTRL`, `DFLLVAL` each second, and measure GCLK1;
   - the C3's USB TX path (the five telemetry stalls).
5. **Decisions for the user (data in the 2026-10-11 log section):**
   - the High-G incident threshold: keep 2 g as a roughness counter and add a 3 g or horizontal-axis incident
     trigger;
   - the canmap speed scale: CAN reads 1.0–1.1% low against GNSS, so 0.01 → 0.0101;
   - `fusion_input.py`: drop or re-time late rows rather than abort;
   - lane detection (2026-10-25 log section): switch v0.2/v0.3 to `RateGate::kPts` (true 20 Hz, ~+50% lane inference
     load) or keep `kLegacy` (~13 Hz); B5 / B6 / B8 (ego-lane logic, vehicle-centre offset, line fit) after the drives.
6. **Turning EKF:** measure the rear-axle-centre-to-antenna distance, then run the measured private course with a
   passenger.
7. **GRF-250:** verify the converter's IC and levels before wiring (the MKR is not 5 V tolerant), then bench the
   sensor.
8. **IMX296 v2 chart:** capture the three sheets with `capture_chart.py` (CALIBRATION.md, "Capture") once the CSI
   camera is back. Its first CSI run is also the helper's first CSI test.
9. **ES3C28P map** (`peripherals/es3c28p/NAVIGATION_PLAN.md`; 2026-10-14 navigation log sections): **Jetson side
   implemented, including ES3C28P whole-frame and panning firmware** (panning changes uncommitted). Decided by the user:
   MAP tab (page 5), north-up, phone hotspot / car WiFi with online OSM tiles + cache, warm map while hidden.
   Software tests and HWCDC/TinyUSB builds pass; no board was flashed for this update. Next: bench the real
   board with the Jetson, including pose-only freshness, background/strip swaps, low-speed dots, reconnects,
   camera/map exclusion and memory fallback; measure actual redraw rate and USB traffic, then enable `<Navigation>`.
   **ES3C28P preview** (2026-10-14 log section): committed. When the board
   arrives: flash it, install `peripherals/es3c28p/host/99-dashcam-display.rules` with its serial, run the bench sketches
   (USB throughput hwcdc vs TinyUSB, JPEG decode/blit), then set `<DisplayPreview><Enabled>true` in the deployed config
   (the section is absent there, so the preview stays off until added) and soak at 10 fps, then 15 fps.
10. **CYD** (2026-10-12 log section): dashboard app flashed 2026-10-13 and boots clean. At the board: listen for the
   boot alert, tap-test touch, cover/uncover the LDR (it read ~980 in the room and held the backlight at 32/255).
11. **Smaller items:**
   - tape over the in-car status LEDs, add a dark dash mat, clean the CSI lens and its windscreen patch;
   - a daylight CSI A/B (c5 / c7n) and the CSI properties in the app's config;
   - a night plate test on an unlit road, and autofocus while driving;
   - whether a link loss minutes after a recovered link-up should stay DEBUG-only (the 00:06 MKR reboot was);
   - item U with a replug, and the optional item O step (both need sudo).
12. **Code backlog:** see "Backlog: deliberately not done" under Reference. Done 2026-10-11: the two `read()`
    warnings and the float control ranges. Left: the `const_cast` in `getCameraStatus()`.

# Reference

## Who does what: read before handing work across

Two coders work on this branch:
- The **cloud session** runs in a virtual machine that holds this repository and nothing else. It started this file; Jetson sessions add to it.
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

## Environment (the Jetson rig, as of 2026-10-10)

Collected from the machine, not from memory: `/etc/nv_tegra_release`, `dpkg-query`, `arduino-cli core/lib list`,
and the tools' own `--version`. Re-collect after an upgrade.

**Hardware**
- **Jetson:** NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super. 6 cores, 7.4 GB RAM, 937 GB NVMe
  root. No RTC coin cell, so the clock comes from NTP or GPS. It runs on **its own battery** in the car.
- **CSI camera:** IMX296LQ (colour, global shutter, 1456×1088 @ 60) in **CAM1** (`i2c-9`, `serial_c`), `/dev/video0`,
  Argus sensor-id 0. Driver rebuilt from the vendor source for 5.15.199 (`~/drive_logs/tools/imx296_driver/`);
  overlay `imx296-cam1.dtbo`. No IMX219 is connected.
- **Camera:** UGREEN Camera 4K, USB UVC, `eba4:6579`, `/dev/video1` since the CSI camera came back. It records
  1080p30 MJPEG (`FormatIndex 4`), pinned by `/dev/v4l/by-id`. Controls are standard UVC only (focus, exposure,
  gain, backlight compensation). Frames from the evening of 2026-09-26 are upright.
- **Telemetry master:** Arduino MKR Zero (SAMD21G18A), **powered from the car's OBD2 port**. Its native USB to the
  Jetson carries the raw CAN stream and the console.
  - **CAN:** MKR CAN Shield, an MCP2515 with a 16 MHz crystal, 500 kbit/s, sniffing the Honda Brio (map `brio`,
    id 0x0B, on the SD card).
  - **I2C:** at 400 kHz. BNO055 IMU at 0x29 on a **breadboard**, INT pin not wired. SparkFun u-blox GNSS at 0x42
    over ESLOV.
  - **SD card:** on SPI1, holding the CAN map and the IMU calibration profile.
- **Bridge:** Seeed XIAO ESP32-C3. It talks to the MKR over UART1 at 115200 and to the Jetson over native USB CDC.
- **Dashboard (bench only, 2026-10-12):** CYD, ESP32-2432S028R (classic ESP32, ILI9341 2.8", XPT2046 touch, microSD).
  USB through a CH340 (`1a86:7523`), which needs the out-of-tree `ch341.ko` (`peripherals/cyd/README.md`).

**Jetson host**
- JetPack 6.2.3 (`nvidia-jetpack 6.2.3+b81`), L4T R36.5.2 (`/etc/nv_tegra_release` R36 REVISION 5.2), kernel
  5.15.199-tegra, Ubuntu 22.04.5 LTS. UEFI `36.5.2-gcid-46426093`. Upgraded 2026-10-01; the 65 NVIDIA packages are on
  `apt-mark hold`. Rollback and upgrade tools: `~/drive_logs/tools/` (2026-10-01 log section).
- CUDA 12.6.11, TensorRT 10.3.0.30 (`libnvinfer10`, built for CUDA 12.5), cuDNN 9.3.0.75, VPI 3.2.4, GCC 11.4.0,
  Python 3.10.12.
- GStreamer 1.20.3, v4l-utils 1.22.1, Docker 29.9.0.
- `nvargus-daemon` is active. `csi-driver.service` (installed 2026-10-03) reloads `imx296` after Argus starts;
  the vendor's `imx296-reload.service` is disabled. The boot entry is `DEFAULT JetsonIO` with `imx296-cam1.dtbo`.
  A kernel package change resets `DEFAULT` to `primary` (nv-update-extlinux); set it back afterwards. The v0.4
  container does not mount `/tmp/argus_socket`, so its start log still shows two harmless
  `(Argus) Error FileOperationFailed` lines (`Connecting to nvargus-daemon failed`, then
  `Cannot create camera provider`).
- ISP override `/var/nvidia/nvcam/settings/camera_overrides.isp` = `c8_sh15.isp` (sha256 `43ec0c81…`).
- Host Python's `cv2` is broken (NumPy mismatch). Use the container; do not change system Python packages.

**Dev container** `l4t-ml-gpio:latest`
- Built 2026-09-25 from `docker_dev/Dockerfile`, base `dustynv/l4t-ml:r36.4.0`; **rebuilt 2026-10-12**
  (`e8686f613fba`, from the final Dockerfile; the 2026-09-25 image is kept as `:build_20261012_115503`). The first 70 of 74 layers are
  byte-identical: only the MKR layer (re-run because `install_samd_aarch64.py` changed in `a04f749`; same
  123,236 B MKR image) and the new CYD layer differ. The repo is mounted at `/user/dashcam`. It builds everything:
  plain `make` gives a timestamped `bin/build_<ts>/`.
- `docker_dev/build.sh` no longer untags `:latest` before building. `docker build --tag` moves it only on success,
  so a failed or interrupted build leaves it in place (2026-10-12: a dropped download from downloads.arduino.cc left
  no `:latest`, which the service needs; restored by hand).
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
- **CYD (classic ESP32):** the same `esp32:esp32` 3.2.0 core (Xtensa toolchain), FQBN `esp32:esp32:esp32` (ESP32 Dev
  Module defaults). `peripherals/cyd/build_and_upload.sh [PORT|auto]` builds the dashboard app (442,402 B; the
  colleague built it on Windows with core 3.3.11). The bring-up sketch builds with plain `arduino-cli compile`
  (384,186 B).
- **Arduino libraries**, pinned in the Dockerfile and installed `--no-deps`, with retries and a name@version check:
  - Time 1.6.1
  - SparkFun u-blox GNSS Arduino Library 2.2.29
  - Servo 1.3.0
  - RTCZero 1.6.0
  - Adafruit BusIO 1.17.4
  - Adafruit GFX Library 1.12.6
  - Adafruit SSD1306 2.5.17
  - Adafruit LED Backpack Library 1.5.1
  - Adafruit ILI9341 1.6.4 (CYD, since 2026-10-12)
- **Vendored** in `peripherals/mkr_zero/vendor`:
  - Wire: the arduino:samd 1.8.14 copy, patched with the `busOwner` fix, bounded transfers and bus errors. See its
    README.
  - CAN 0.3.1, patched.
  - SdFat 2.3.1.
  - BNO055 1.2.1.

**Protocol and firmware versions**
- MKR↔C3 `CommProtocol` and C3↔Jetson `HostProtocol` are both at version **0x07** (180-byte telemetry payload).
- The C3 bridge firmware reports **1.0**.
- The MKR production image is the raw-CAN build (`3a702a8`): DISCOVER at boot, 123,236 B flash, 23,924 B RAM.
- `dashcam-v04.service` runs the newest `bin/build_*/dashcam_v0_4` it finds at start.

**Where the cloud sandbox differs** (see "Cloud sessions" above): arm-none-eabi-gcc 13.2 and host g++ 13 instead of
7.2.1 and 11.4.0, no CUDA, and no hardware.

## Tests

```bash
make -C peripherals/mkr_zero/tests/host check        # 11 suites: switch, imu, can_probe, bno_init, can_ring, can_drain,
                                                     #   can_telemetry, can_stream, can_selftest, port, sketch policy
make -C peripherals/mkr_zero/tests/host mutations    # 102 mutants, all must be caught
make -C peripherals/esp32-c3/tests/host check        # bridge 79
make                                                 # every target, inside the l4t-ml-gpio container
B=$(ls -td bin/build_* | head -1)                    # the build just made; run from the repo root
$B/dashcam_v0_4 --self-test && $B/config_test && $B/bridge_overlay_test
$B/camera_gst_test [name]                            # Camera_GST over videotestsrc: no camera needed
$B/focus_test                                        # the fixed-focus hold against a simulated camera
$B/usb_test --self-test                              # usb_test's stop-and-restore path on a simulated camera
docker_dev/test_csi_driver.sh                        # csi_driver.sh against stubs (123 checks)
docker_dev/csi_driver.sh check                       # on the Jetson, read-only: the IMX296 module vs the running kernel
$B/commlink_sim_test                                 # CommLink + libuart against a simulated C3 on a pty: no hardware
$B/commlink_test --by-id 30                          # the real C3 bridge, by-id only as v0.4 (stop dashcam-v04 first)
$B/commlink_test "" 30                               # auto-discover with the ttyACM fallback
$B/record_test                                       # Part A needs GStreamer base/good/bad/ugly plugins; Part B a camera
$B/usb_test                                          # a USB camera (stop dashcam-v04 first); puts its controls back
$B/scan_cameras                                      # lists the cameras, their controls and formats
make -C peripherals/mkr_zero/tools/canstream check   # raw-CAN logger, converter, decoder, contract, turning + fusion (~2 min; container)
$B/lane_test --unit --strict --allow-xfail B5,B6,B8  # liblanedetector CPU tier (<1 s); B5/B6/B8 are known, deferred bugs
$B/lane_test --gpu --strict --engine models/culane_res18_fp16.engine --tiny /user/lane_engines/tiny.engine \
    --fp16-in /user/lane_engines/tiny_fp16in.engine --fp16-out /user/lane_engines/tiny_fp16out.engine
                                                     # GPU tier (~90 s, no camera; mount ~/drive_logs/lane_engines)
$B/lane_test --replay --pack DIR --engine E --out DIR --golden DIR   # R1 replay vs a golden (packs/goldens outside the repo)
$B/lane_test models/culane_res18_fp16.engine         # live IMX296 smoke (10 s); `--video FILE W H [s [stride]]` for clips
tools/lane_eval/lane_run.sh start|status|mark|stop|calibrate   # host: lane test run (cameras, MKR logger, can_live,
                                                     #   supervisor, status display, sysmon), see DRIVE_PROCEDURE.md
python3 tools/lane_eval/check_run.py RUN_DIR [--bench]   # host: the plan's health/resource gates on a run
python3 tools/lane_eval/phase_table.py RUN_DIR           # host: B0 resource deltas per phase
python3 tools/lane_eval/sync_check.py RUN_DIR            # host: B5 cross-camera offset (torch / hand passes)
$B/lane_drive --mount-replay --out DIR --files A.mkv,... --ref-file A.mkv --ref-at S   # B9 mount monitor on footage
make -C lib/libdeadreckoning check                   # dead-reckoning library (55 checks); `sanitize` for ASan/UBSan
make -C lib/liblidartracking test                    # GRF-250 parser/FSM + Orin association library (container for camera tests)
make -C lib/libstereoprototype test                  # stereo feasibility prototype (container)
tools/isp_tuning/chart/v2/run_calibration.sh ...     # v2 chart analyser; tests: test_calibrate.py in the container (CALIBRATION.md)
/usr/bin/python3 tools/isp_tuning/chart/v2/test_capture_chart.py   # chart capture helper, host, no camera (18 tests)
peripherals/cyd/host/ch341_module.sh check           # on the Jetson host, read-only: CH340 driver, tty, brltty
bash ~/drive_logs/tools/verify_platform.sh           # on the Jetson, not in the container: after a platform change
bash ~/drive_logs/tools/test_l4t_checks.sh           # on the Jetson: the pre-reboot checks of the upgrade/rollback
```

`usb_test` puts brightness, gain and backlight_compensation back as found (`782bdfd`). It passed on the Jetson on
2026-09-28 and again on 2026-09-30, with an empty before/after `--list-ctrls` diff. To check by hand, diff
`v4l2-ctl -d /dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0 --list-ctrls` before and after the
run: `/dev/video0` is the IMX296 now.

The Wire changes have no host test: they are register-level SAMD21 code, proven only on the rig
(BusFaultInjection). Running that is the Jetson coder's job.

Where each test can run:
- **Anywhere, cloud included:** the host suites, `mutations`, `--self-test`, `config_test`,
  `bridge_overlay_test`, `camera_gst_test`, `focus_test`, `usb_test --self-test`, `commlink_sim_test`,
  `test_csi_driver.sh` and `record_test` Part A.
- **Jetson only:** CUDA targets (`lane_test --gpu`, `--replay`, live; `lane_drive`), `record_test` Part B, `usb_test`, `csi_test`, `scan_cameras` (with a camera),
  `commlink_test` (with the C3), `csi_driver.sh`, and everything on the rig or the car.

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

## Backlog: deliberately not done

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

# Log (newest first)

## 2026-10-27 (Jetson, bench): the IMX296 resets itself at low analogue gain; gain floor in Camera_CSI

The user: check the degradation hunt; then reload the stock driver and test in daylight, then candidate C; lens
capped; the module is an InnoMaker IMX296 colour board, bare PCB, stuck circuitry-down on the bench; no hardware to
swap the module or feed it from a bench supply; then "go with the gain range limit" and correct HANDOVER. Data and
tools outside the repo: `~/drive_logs/lane_p0/P0_REPORT.md` (sections from "Degradation hunt result"),
`~/drive_logs/lane_p0/driver_ab/` (`fault_test.sh … trt|usb|none`, `AE_EXPRANGE` / `AE_SRCPROPS` / `AE_FPS`,
`shs_poll.py`, `fps_test.sh`), captures in `/media/jetson/backup/lane_tests/flicker/`.

- **The hunt** (stock driver, fresh boot): 15 probes 22:05–05:24 clean (night, gain median ~120), then three at dawn
  05:51–06:02 all faulted (gain median ~60, min 16). It tracked the light, not uptime (8.8 h) or sessions (198).
- **Module age is ruled out:** a freshly reloaded stock driver, camera at the window, faulted 3/3 (51–111 s). Every
  2026-10-26 "stock passes after a reload" test had run at night. Candidate C faulted 4/4 in daylight too.
- **The sensor resets itself.** `shs_poll.py` reads SHS1 / VMAX / GAIN on i2c-9 0x1a every 5 ms (read-only,
  `I2C_RDWR`; the jetson user is in group `i2c`). At every fault the sensor NACKs I2C for 166–173 ms, exactly when
  frames stop, and then reads back its power-on defaults (SHS1 14, VMAX 1130, GAIN 0); the driver's `power_off` comes
  only with the Argus teardown 3.4 s later.
- **The trigger is the gain.** Faults with no GPU load, with exposure pinned (85 or 999 lines), with one register
  write in the whole run, and with the lens capped (dark sensor; the board's parts face the bench). Lens covered,
  exposure pinned, no load: GAIN register 16 faults in seconds to a minute; 24 / 32 / 40 clean and 48 faulted once in
  5 min; 64 clean for 20 min, 128 and 200 clean. AE in the dark settles at 200 and runs clean. Light only matters
  because bright scenes drive AE to the floor (register 16 = Argus 1.0, the DT minimum).
- **Frame rate:** 8 pairs at gain 16: 60 fps 3.3 resets/min, 30 fps 4.3/min (ratio 1.32, 95 % CI 0.50–3.52). A
  per-readout current spike (ratio 0.5) is disfavoured, not excluded; two 30 fps runs reset before the first frame.
  Times to reset lengthened as the module cooled after a sunlit soak, which suggests temperature raises the hazard.
- **Driver vs mainline** (a workflow plus skeptic checks): the vendor init sequence is the mainline / Raspberry Pi
  one (41-entry init table byte-identical, the same gain table); no register difference links to gain or a reset.
  Leading explanation: a power or bias margin on the InnoMaker module (no DT regulators; FPC pin 11 = CAM1_PWDN is
  its power enable, 3.3 V on pin 15, 1.2 / 1.8 V made on the board), in this unit or in the design. Swap tests (the
  module on a Pi, a second module, a bench 3.3 V feed, bulk decoupling at pin 15) would separate them.
- **Workaround:** `Camera_CSI::buildPipelineString()` now sets `gainrange="4 12.5"` (register 64–200) on every CSI
  pipeline; a `gainrange` attribute still replaces it. The same range passed a 30-min daylight soak with `trtexec`
  load and AE stepping (gain on the floor most of the time) that faulted within 1–3 min without it. Cost: a little
  more noise in bright scenes (AE shortens the exposure instead; exposure ran 63–1000 lines). `camera_gst_test`'s
  exact pipeline string is updated.
- **Side findings, not changed:** `set_exposure` clamps the exposure only from above (SHS1 can exceed VMAX − 2 for an
  exposure under 2 lines; not reached with DT limits); SHS1 is not recomputed when only VMAX changes (at 30 fps the
  999-line request ran at 2124 lines); `fault_test.sh` missed one Argus wording ("Skipping requests on sensor GUID 0"),
  now matched.
- **Soak with the floor** (stock driver reloaded, build `build_20261027_040632`, camera at the window in midday sun,
  `lane_run.sh start --minutes 30 --display none --no-can`, run_018): the floor held (gain exactly 64 for 96 % of
  the streaming time, never lower), yet the sensor reset 6 times at 64 (mean ~5 min apart) and 3 more times
  16–20 ms after stream-on at gain 192, each ~23 s after a reset. So the floor lowers the hazard but does not remove
  it; the module also resets on the stream-on current step when stressed (sun, heat; Jetson tj 54–58 °C, VDD_IN
  4.89 V at the 14 W reset moments). The harness recovered every time: CSI 31 640 results over 29.7 min, mean
  19.75 Hz, 12 in-process and 9 container restarts; the UGREEN ran clean. **Open:** two CSI instances ended with
  exit 134 (abort) instead of 3 after a reset at start; their logs were overwritten (`sys/lane_csi.log` keeps only
  the last instance).
- **State:** the stock driver is loaded again; `dashcam-v04` stopped.
- **Addendum 2026-10-29: heat is the main factor.** The user detached the module and put it under an AC airflow
  (20 °C); fresh boot, stock driver. The IMX296's die temperature is readable over I2C (write 0x301d = 1, read
  0x301e, 10 bits, T = 246.312 − 0.304 × TMDOUT, write 0x301d = 0; works in standby). Gain register 16, exposure
  pinned, no load: 3 × 5 min clean with the die at 26.5–34.7 °C, where the warm module had reset 18 times out of 18
  within ~70 s. No die temperature was logged during any faulting run; a warm-up test to find the reset temperature
  is running. Cooling the module (shade, airflow, a heatsink) is likely the real fix; the gain floor adds margin.

## 2026-10-26 evening (Jetson, bench): the IMX296 fault is a degraded driver/sensor state, not hardware

> **Corrected 2026-10-27:** wrong. The fault is a sensor self-reset at low analogue gain; every "stock driver
> passes after a reload" test below ran at night, when AE kept the gain high (2026-10-27 log section).

The user, after P5b: restart nvargus-daemon and test the CSI again; "the rig is physically unchanged from yesterday
so could be a software issue"; then find the root cause. Data: `~/drive_logs/lane_p0/P0_REPORT.md` (P5b section,
"IMX296 fault root cause" and "CORRECTION"); tools `~/drive_logs/lane_p0/driver_ab/` (outside the repo).

- **The fault:** Argus `waitCsiFrameStart Timeout … Sensor GUID 0 is in error state`; in-process restarts never
  recover, a fresh process only briefly. Seen all day from 07:52, at ~24 h uptime.
- **Ruled out:** the daemon (a fresh `nvargus-daemon` still failed, run_010); clock scaling (`jetson_clocks` still
  failed, run_011); the UGREEN, USB and CPU (a GPU-only `trtexec` load was enough); buffer starvation (a held-buffer
  pipeline ran 8 min clean); our pipeline (a bare `nvarguscamerasrc → fakesink` failed under load).
- **What it needs while degraded:** GPU load **and** frequent AE gain writes. AE locked or gain fixed: clean; AE free
  under load: fails in 1–6 min (40–180 gain writes); AE free without load: 226 updates clean. A dim scene pins gain at
  the 200 cap (few writes), which is why the 2026-10-25 evening soak was clean.
- **A driver candidate was a red herring.** Three variants of the vendor driver without its REGHOLD group hold and/or
  `GAINDLY_1FRAME` were built (`~/drive_logs/tools/imx296_driver/staging_20261026/`); "C" (both removed), loaded with
  `CSI_MODULE_PATH=… csi_driver.sh reload`, passed 15 + 60 min dual runs and 5 433 forced gain writes. But the
  **stock** driver, after the same reload and after a reboot, passed too: strong tests (`trtexec` or UGREEN load +
  exposure compensation stepped ±1 EV every 2 s, ~500 gain writes/min) with 3 715–3 856 writes, and run_014 (30-min
  dual `lane_drive`, 615 gain writes, 0 faults), the workload that failed in 2–3 min in the afternoon. Every fault
  came before the first module reload. **Do not install the candidates:** under the identical AE stimulus the stock
  driver's transitions are monotonic (0 one-frame reversals in 90), C's are not (6, ≤ 8.5 luma), consistent with
  `GAINDLY_1FRAME` aligning gain with the sensor's one-frame-later exposure.
- **Now:** rebooted 21:08 (stock driver, default clocks). `degrade_hunt.sh 30` (detached, since 21:59) probes every
  30 min (5-min strong test; every 4th probe also a 10-min dual `lane_drive`) with a 5 s Argus session every 2 min in
  between, and keeps its own kernel and Argus logs (the journal is volatile here). It stops at 30 h, on a `STOP` file,
  or at the first fault + 2 confirmations, leaving the degraded state for inspection. Results:
  `/media/jetson/backup/lane_tests/hunt_20261026_2159/probes.csv`.
- **Also seen:** run_013 UGREEN RSS steps (+87 MiB at ~40 min) and fds 75 → 87, not in run_005 / run_008 / run_014;
  CSI event-dump queue drops (3–4 per hour, bursts > 16 jobs).

## 2026-10-26 (Jetson, bench): lane detection test suite P5b (drive harness extras)

The user asked to start P5b (plan section 14: clip rings, dumps, mount monitor, status display and alerts, degrade
ladder, `.ass` banner, ROI calibration, bench tests B0/B3/B3m/B5/B9). Full numbers: `~/drive_logs/lane_p0/P0_REPORT.md`
(P5b section); runs in `/media/jetson/backup/lane_tests/run_003…009`.

- **Library:** `LaneDetectorTestHook::onInference` hands the frame, the published result and the four output tensors
  to a test harness (thread and `processFrame` paths; GPU test PF4). Replay still bit-identical (250/250).
- **`lane_drive`:** CSI encode branch at 30 fps NVJPG feeding the review (decimated by a PTS probe) and a byte-capped
  RAM clip ring; UGREEN ring of every MJPEG frame. Event clips (t − 10 … t + 10 s, merged, ≤ 60 s) on stall, camera
  error, latency spike, error counter, mount shift, CAN stalk episodes, stop→go, operator marks, flicker bursts and a
  random sample, rationed by a 1.5 GB/h-per-camera token bucket (faults first). Tensor dumps (`LTD1` .bin + input
  band + the camera frame with the same PTS) every 5 s and on events. `image_stats.csv`, `mount.csv`, `.ass` lane
  banner per review part, `status.json`, `RUN/ctl/degrade`, `--lanes-schedule` (B0), `--stills` (calibration),
  `--instance K` (restart into the same run), `--mount-replay` (B9). On `cam_lost` it finalises and exits 3. The
  camera-free parts are in `src/tests/lane_drive_aux.h` with unit tests A1–A5.
- **Mount monitor:** gradient cross-correlation of a car-fixed region against a parked reference (1 Hz, 5 s median,
  FAIL when > 6 px or > 0.5° for 5 s, latched). A view that moved beyond the ±48 px search (the 10-03 slip was
  ~230 px, ~9°) counts as a shift when edges are still visible. Its verdict raises alerts **only for a car-fixed
  region** (`roi_calib.py` found the hood edge, or `--mount-roi`): the UGREEN's view is all road and would
  false-alarm once the car moves. Replay of the 10-03 night: the CSI slip is flagged 5 s after 22:20:13.
- **Host tools:** `can_live.py` (follows `can_raw.log` through the unmodified `can_decode`, writes
  `can/live_state.json` and clip triggers), `lane_supervisor.py` (container restarts up to 10, ≥ 20 s apart; CAN
  logger restart into `can/partN`; degrade ladder; GPU-saturation step; `status.json` with display lines),
  `lane_status` (own binary: PAGE text and ALERT beeps through `libes3display`), `roi_calib.py`, `balloon.py`,
  `sync_check.py`, `phase_table.py`; `lane_run.sh calibrate|mark`, `--mount-ref`, `--display`, `--expect-can`,
  `--plan night`, `--idle-lead`, `--csi-args/--usb-args`; `sysmon.py` logs VDD_IN and the soctherm OC counters;
  `check_run.py` adds ring, losses, clip/review ffprobe, dump pairing, mount and supervisor gates.
- **Bench results:** B0 — idle 5.4 W; cameras + recording 7.9 W, 136 % CPU (of one core); both lanes 13.7 W, 183 %,
  GPU 56 % at 715 MHz, ≈ 145 mJ per inference. B3m — degrade steps 1–5 in order at MemAvailable ≈ 0.9 GB, step 6 at
  0.55 GB stopped the run, exit 0, no OOM kill (OOM-victim test not done). 60-min dual soak — the UGREEN passes every
  gate (20.0 Hz, p99 52 ms, 0 errors/drops, RSS flat, 21 clean clips, 100 % exact-PTS dumps, ≤ 8.7 GB/h, tj 58 °C).
  `can_live` on a real CAN recording replayed live: 124 394 frames decoded, stalk and stop→go episodes found. Kill
  tests: logger → part1, camera container → instance 1, both recovered.
- **Pitfalls found:** libjpeg in `lane_drive`'s process (OpenCV, `libvideopreview`) breaks `nvjpegenc` ("JPEG
  parameter struct mismatch") → no OpenCV/libjpeg in `lane_drive`, band JPEGs via GStreamer `jpegenc`, the status
  display a separate binary. A `videorate max-rate=5` after a tee negotiated the upstream 30 fps `videorate` down to
  ~5 fps → PTS probe decimators.
- **Open:** the IMX296 sensor timeouts (found the same evening to be a degraded state: section above; corrected
  2026-10-27, a low-gain sensor reset); the CYD needs its navigation firmware for the status
  pages; the single-camera TensorRT p99 gate (25 ms) fails at the governor's low GPU clock (29.6 ms); not done: B5
  (needs a person with a torch), a dark-room soak, B1b, GNSS logger, a lane-based mount check.

## 2026-10-25 (Jetson, bench): lane detection test suite P0–P5a (committed `24110f6`, `650701a`)

The user asked for a test suite for `liblanedetector`: performance in real driving, resource use, bug catching. Plan
(user decisions, tiers T0–T6, phases P0–P8): `~/drive_logs/lane_test_suite_plan_20261025.md`. The user chose the
**lean cut**: P0 → seams → bug tests and fixes → core drive harness → first drive. Full results, numbers and findings:
`~/drive_logs/lane_p0/P0_REPORT.md`. Binding decisions: the suite owns both cameras and the MKR during a run (v0.4
stopped); lanes on the IMX296 *and* the UGREEN; CAN stays raw (DISCOVER) and is joined offline; library seams allowed
with production behaviour unchanged; ground truth = CAN events + the UFLD demo.py oracle + ~250 hand labels (outside
the repo); the CSI mount is now rigid (a view shift is a FAIL); day and night drives; ≤ 20 GB/h.

- **P0 risk spikes.** The deployed lane engine (803 MB) had an FP32 FC head (64% of 14 ms) and was built on another
  device; a native FP16 rebuild (413 MB) runs 9.4 ms with the same accuracy against the ONNX FP32 reference, and now
  **replaces it** (old one kept as `.prev803_bak`). At 20 Hz the GPU governor holds 306–714 MHz, so one inference takes
  ~27 ms in practice. Two engine processes cost 1.3 GB → one process per camera. CSI gain/exposure cannot be read
  through V4L2 while Argus streams. `/var/tmp/ufld_calib` holds training-geometry crops of UFLD's `example.mp4`, not
  own-car frames. The image was rebuilt (matplotlib, addict, psmisc). `lane_test --video` was broken (decodebin's
  NVMM output into videoconvert) and is fixed.
- **P1 seams** (each step replayed 848 recorded frames bit-identical to a golden from the unrefactored code):
  `processFrame()`, `computeLaneGeometry()` (`lane_geometry.*`), `decodeUfldV2()`/`egoFromSlots()` (`ufld_decode.*`),
  per-slot geometry, `LaneTiming` (capture/pull/done on CLOCK_MONOTONIC, CUDA-event stage times), `LaneStats`, result
  sink, thread name, `LaneDetectorTestHook`. Replay packs (UFLD example, 10-03 night CSI and UGREEN) in
  `/media/jetson/backup/lane_replay`. Finding: with the CSI crop defaults the UGREEN night pack shows lanes in 0 of
  298 frames — the bottom crop removes its nearest markings; the UGREEN gets its own crops (`lane_drive` default 0.5/0).
- **P2 tests.** `lane_test --unit` (23 tests) and `--gpu` (29 tests, tiny engines from `make_tiny_engine.py`) with
  known-bug XFAIL discipline; every one of the 15 suspected bugs reproduced. Notable: a wrong-size NVMM stream was
  cropped from the wrong pixels and yielded results silently (B7); the wall-clock rate gate gave 13 Hz for 20 (B3).
  `--video` now runs every frame through `processFrame()` (deterministic).
- **P4 fixes** (API: `setLogCallback()` returns bool; `processedFrameCount()` restarts at `start()`): B1 B2 B7 B9 B10
  B12 B13 B14 B15 B17 B19, each with its failing test first. B3 as `LaneDetectorConfig::rateGate`: `kPts` gives exactly
  20 Hz; **the default stays `kLegacy`** until the user approves the higher load for v0.2/v0.3. Remaining known bugs:
  B5 (ego lane from x-sorted boundaries), B6 (vehicle centre hard-coded), B8 (fragile line fit) — after the drives.
- **P5a harness.** `lane_drive` (one camera per container: per-frame and 1 Hz CSVs, review MKVs, lanes-off lead-in,
  restart policy, graceful exit with a finalising watchdog; UGREEN with v0.4's exposure loop and focus 690, handed
  back) and `tools/lane_eval/` (`lane_run.sh`, `sysmon.py`, `check_run.py`, `DRIVE_PROCEDURE.md`). An 81-min soak,
  both cameras + MKR logger, passed every gate: 20.0 Hz each, capture→result p99 39 ms (CSI) / 54 ms (UGREEN), 0
  errors, drops or restarts, RSS falling, MemAvailable ≥ 3.18 GB, tj ≤ 58.8 °C, 7.5 GB/h, 13.4 W.
- **Pitfalls found** (all handled in the code): `gst_parse_bin_from_description(ghost_unlinked=TRUE)` breaks
  `splitmuxsink`; a bin description cannot start with a bare caps string; `videorate` stamps output on its own timeline
  (~1 s ahead on the UGREEN path) — that path restamps on arrival and decimates in a probe; **`nvv4l2decoder` on the
  UGREEN MJPEG path is broken** (13 Hz, latency growing ~1 s per 3 s) — the UGREEN uses `jpegdec` (45% of a core).
- **Not done (lean cut):** live CAN in the logs, instrumentation-neutrality (B1b), a dark-room night-bitrate soak,
  physical fault injection (unplug), status display, clip rings, tensor dumps, mount monitor, analysis/report (P6).
- **Next:** crop check in the parked car (P0b), then the first lane drive with `lane_run.sh` (day, then night).

## 2026-10-17 — Original CYD whole-frame navigation port

- Added `peripherals/cyd/lib/cydNavigation/`: shared ES3 protocol wrappers,
  bounded single-buffer receiver, baseline JPEG MCU rendering, and UART0 controller.
  The original CYD uses 115200-baud CH340 USB, advertises a 32 KiB image limit and
  one credit, and declines panning/camera streaming. The user approved whole-map
  fallback: the Jetson supplies the marker, uncertain-heading dot and attribution.
- Added the sixth MAP tab with a 320×224 viewport; retained local tabs, calibration,
  SD, audio and LDR control. Runtime Serial diagnostics are suppressed for binary
  transport. Hidden/calibrating demand, stale/placeholder display, bounded partial
  writes, heartbeat expiry and reconnect are handled by the application loop.
- `lib/libes3display` now accepts the explicit CYD HELLO identity. Added a CYD
  fallback regression to its full suite. ES3C28P source/protocol/plans unchanged.
- Build helpers include the new library and pin JPEGDEC 1.8.4. Setup, slower-link
  timeouts and limitations are documented in `peripherals/cyd/NAVIGATION_IMPLEMENTATION.md`
  and the CYD README. No deployed config, service or device firmware changed.
- Validation: Arduino ESP32 3.2.0 compile passed (475042 bytes flash, 79744 bytes
  static RAM); receiver C++11/C++17 and ASan/UBSan passed; real JPEG decoder and
  actual controller native tests passed; complete Jetson display-link suite passed
  (0 failures). Windows/PlatformIO not executed. No physical board validation.
- Next: flash/bench deliberately, verify CH340 throughput, marker/attribution,
  transitions/reconnect, six-tab calibration and audio/SD/backlight heap margin.
  At the 32 KiB cap, payload transmission alone takes about 2.85 s; do not expect
  ES3C28P panning performance. Work remains uncommitted.


## 2026-10-17 (Jetson): navigation (map) Jetson side reviewed and refactored

User asked for a thorough recheck and refactor of the Jetson navigation code. Multi-agent review: 53 findings (50
confirmed by two verifiers each), then a review of the fixes (29 more, fixed except two test-only items). Summary and
the list of what was not done: NAVIGATION_PLAN.md, "Jetson-side review and refactor (2026-10-17)". Nothing committed.

- **Code:** `lib/libnavigation` split into `libnavigation.cpp` (API, bounded stop), `nav_service.h`, `nav_fetch.cpp`,
  `nav_render.cpp`; fixes in `nav_tiles.cpp`, `nav_compose.cpp`, `nav_pan.cpp`. Map/pan parts of
  `lib/libes3display/libes3display.cpp` (resend counter, caps re-ask, VISIBILITY page, margin guard, session helper;
  done on top of the colleague's 2026-10-17 link review, see `lib/libes3display/VIDEO_REVIEW_2026-10-17.md`).
  `libconfig` validation, `src/bridge_overlay.h` toNavSample, `src/dashcam_v0_4.cpp` wiring, Makefile.
- **Behaviour changes worth knowing:** a parked whole-frame map refreshes once a second (it used to re-send 4 fps);
  `<Navigation>` StaleAfterMs is clamped to >= 1500 and PanBackgroundWidth/Height to >= 384/256 (with a WARN);
  ProviderUrl placeholders other than {z}{x}{y} and an attribution over 52 characters now turn the map off with a
  clear message; a redirect from the tile server turns downloads off for the run.
- **Tests (all PASS, plain `make`, 0 warnings, two repeat runs stable):** navigation_test (new [3b], [10] and many
  checks), es3display_test (new [24], [21] additions), config_test, bridge_overlay_test; firmware `run_tests.sh`;
  v0.4 SIGINT -> exit 0 with navigation on.
- **Open:** commit; the bench with the real board; the not-done items in the plan section.
- **Deploy note:** `kSupervisorStuckMs` (30 s) is longer than `docker stop -t 20`, so the app's own clean-exit backstop
  can never fire before docker's SIGKILL. navigation.stop() is now bounded (3 s), which removes the known cause, but
  aligning the two timeouts is still worth doing (not changed here).


## 2026-10-17: Jetson video streaming review and refactor

Review scope and severity-ranked findings: `lib/libes3display/VIDEO_REVIEW_2026-10-17.md`.
The camera tap, preview converter/pools, USB delivery and v0.4 lifecycle were reviewed.
Pending navigation/panning and firmware changes were preserved; plans and wire headers are unchanged.

- Fixed the producer wake/close race using a dedicated eventfd mutex; descriptor allocation now follows
  allocating setup. No producer can write to a descriptor that stop has closed and the OS has reused.
- Fixed artificial 100 ms sleeps after the bounded TX batch. The worker checks RX/timers before resuming
  immediately when it has yielded for fairness; real write/kernel-queue blocking still waits normally.
- Credit recovery checks the current camera/map/panning queues. START_MAP/START_PAN negotiation and
  already-handled timeout states no longer leave obsolete deadlines driving recovery/polling.
- Duplicate RESULT Ok no longer drops a frame awaiting PRESENTED. A real post-assembly failure still drops it.
- Split image eligibility, selection, START and continuation into smaller functions. JPEG conversion, pools,
  credit limits, wire format and page-driven camera/map exclusion remain unchanged.
- The four targeted fault regressions fail against the pre-review transport. The full preview suite and
  all 23 display-link scenarios pass with the refactor. `dashcam_v0_4` and test builds pass with
  `-Wall -Wextra -Werror`. ASan/UBSan preview and final link regressions passed, with leak detection enabled.
  GCC 11 analyzer: two false-positive possible-null reports inside throwing `std::make_unique`; no other reports.
- Recorder synthetic Part A tests passed (0 failures), including slow/failing preview consumers, rate
  independence and EOS/shutdown. Part B was skipped because no USB camera was passed into the containers.
  No board was flashed and no deployment changed. Real UGREEN→ES3C28P FPS,
  capture latency, recorder continuity under load and thermal behaviour remain bench/drive checks.
- Reproduce in `l4t-ml-gpio:latest`: build `videopreview_test`, `es3display_test`, `record_test` and
  `dashcam_v0_4` with an isolated `BUILD_DIR`; run the three test binaries. The focused transport regressions
  can be run with `es3display_test --streaming-regressions`. Do not pass camera devices for synthetic-only testing.
- This review's changes remain uncommitted. Next: physical UGREEN→ES3C28P testing at 10–15 fps while recording,
  including reconnects and camera/map tab switches; measure latency and thermal behaviour before deployment.

## 2026-10-16: ES3C28P focused video pipeline review

Follow-up to `6fdb0c0`; details in `peripherals/es3c28p/VIDEO_IMPLEMENTATION.md`,
"Focused streaming review and refactor". No protocol or design-plan changes; no board flash.

- Fixed the view/acquire session race: a token-specific acquisition leaves a replacement session's first
  image queued instead of taking it and dropping it as Hidden. Covers camera and map/panning consumers.
- Split DECODING and DRAWING ownership. Validate completion/release handles before touching buffers or
  credits; reject duplicate completions, stale/reused-slot releases and invalid indices. Do not overwrite
  a live handle during acquisition. The old implementation fails the new duplicate-credit regression.
- Scoped mutex guards make worker lock boundaries explicit; decode and SPI operations remain outside locks.
  Portable pixel helpers validate decoder block output and viewport-sized TFT strip copies, with canary tests.
- Native C++11/C++17 protocol/state tests, 39 Python tests, receiver/pixel sanitizers and JPEGDEC fixtures passed.
  All 22 host display-link integration scenarios passed against the current workspace (0 failures), including
  reconnect/credit handling and navigation panning with the firmware receiver.
- Both Arduino 3.2.0 builds passed with all warnings enabled and no warnings: HWCDC flash 544,194 bytes,
  static RAM 54,748 bytes; TinyUSB flash 580,786 bytes, static RAM 73,908 bytes. Static RAM grew by 8 bytes.
- PSRAM pools, task stack reservations and eight-row SPI budget are unchanged. Hardware soak, throughput,
  tearing and stack/heap measurements remain outstanding; these changes do not establish a new frame-rate claim.

## 2026-10-16: ES3C28P code review and refactor

User requested a full review/refactor after the panning work. Detailed findings, ownership map, test commands and
hardware limits: `peripherals/es3c28p/REVIEW_2026-10-16.md`. Existing uncommitted work was retained; the three design
plans and shared protocol headers were hash-checked unchanged. No board was flashed and no deployment was changed.

- **P1 startup defect fixed:** the shipped Settings text `Tap SET tab to calibrate` was 24 characters, exceeding
  the 23-character page limit. `setPages()` failed and short-circuited graphics initialization. The shorter label
  and actual default table (`src/DashboardConfig.h`) now have a portable regression test.
- **Other fixes:** calibration saves verify a temporary file and preserve a backup instead of deleting the old
  record first; malformed/oversized calibration text is rejected safely. Missing SD files no longer remount the
  filesystem underneath WAV playback. BMP height/extent arithmetic and header overlap are validated. Python
  control requests/playback poll idle parsers so a buffered reply behind a truncated record is recovered without
  replaying the command. Startup allocation cleanup resets capacities, WAV metadata scanning is bounded, and
  successful USB initialization is idempotent.
- **Refactor:** dashboard, touch/calibration, SD images and streaming presentation now have separate graphics
  source files with the same single loop-task panel owner. Portable allocation, persistence, page validation
  and media-header helpers permit direct fault-injection tests. Wire format, map/camera behavior, PSRAM pool
  sizes and task stack reservations are unchanged; voice remains unimplemented.
- **Checks:** expanded C++11/C++17 suite and 39 Python tests; ASan/UBSan resource/media/state/raster tests;
  pinned JPEGDEC fixtures; HWCDC and TinyUSB builds all pass. No compiler warnings. Final flash/static RAM:
  HWCDC 543,610/54,740 bytes, TinyUSB 580,190/73,900 bytes. Static RAM and PSRAM are unchanged; flash is
  about 15.7 KB smaller than the prior panning builds. Commands and evidence are in the review record.
- **Hardware next:** verify first boot/calibration and SAVE/reboot/backup recovery; missing assets during audio;
  map/camera/tab/reconnect transitions; throughput, tearing, touch latency and stack/heap minima during a soak.
  Card replacement now requires restart or explicit remount after all File handles are closed. No claim of FAT
  power-loss atomicity or measured panel performance is made.

## 2026-10-15 handover: ES3C28P navigation panning firmware completed

Firmware implementation and validation completed 2026-10-14; recorded here 2026-10-15. This supersedes the older
entries saying that firmware panning is pending or rejects 0D. Detailed implementation and hardware acceptance:
`peripherals/es3c28p/NAVIGATION_IMPLEMENTATION.md`. No commit, deployment or board flash was performed.

- **Negotiation and compatibility:** implements PAN_CAPABILITIES (0D) and START_PAN (0E) against the already
  updated shared headers. Advertises backgrounds up to 512x384, 16-pixel top/bottom strips and up to 30 pose
  inputs/s. STOP_MAP ends either map mode. Whole-frame maps and camera preview remain supported; map and camera
  sessions exclude each other. The navigation/video/voice plans and shared protocol headers were unchanged
  by this firmware work (hash-checked).
- **Receive/decode:** backgrounds and 320x32 status/attribution images use independent pairs of RGB565 buffers;
  only the same image kind supersedes a queued image. Valid increasing MAP_POSE sequences for the current
  visible session can arrive between JPEG chunks without credits or replies. Foreign, stale and invalid poses
  are ignored. Decoder output capacity is checked explicitly for the larger images.
- **Display:** new portable `lib/es3Map/PanRaster.h` crops the retained background and draws the dot, heading
  arrow or last-known ring locally, clipped to the map area. The board applies the host pose directly; GPS
  smoothing stays on the Jetson. Each draw freezes its pose and image leases, writes at most eight rows per
  loop and coalesces subsequent poses. Unchanged strips remain on screen. PRESENTED is sent once per image;
  retained buffers cannot be overwritten by decode. Valid poses refresh freshness even without new JPEGs;
  stale/disconnect notices affect only the top strip. Visibility/session changes release retained leases.
- **Memory/fallback:** two compressed inputs, two 512x384 backgrounds and two 320x32 strip buffers total
  958,464 bytes of PSRAM (startup upgrade peak 1,265,664 bytes). If the optional allocation fails, the original
  438,272-byte whole-frame pipeline remains available and panning returns NotReady for host fallback.
- **Validation passed:** `tests/run_tests.sh` (C++11/C++17 wire, receiver, surface and raster checks, shared-header
  parity, 38 Python tests); pan receiver/raster ASan + UBSan; JPEGDEC sanitizer fixtures including 512x384 and
  511x383 baseline 4:2:0 backgrounds and 320x32 4:4:4 strips. Tests cover interleaved poses, retained leases,
  same-kind replacement, temporary buffer contention, crop bounds, marker directions/colors and malformed JPEGs.
  Arduino ESP32 3.2.0 / JPEGDEC 1.8.4 builds pass with `--warnings all` and no warnings in both modes:
  HWCDC 559,370 bytes flash / 54,740 bytes static RAM; TinyUSB 595,914 / 73,900. PSRAM is additional.
- **Still needs hardware:** connect to the actual Jetson sender, verify long pose-only intervals, independent
  asset swaps, marker trails/clipping, low-speed dots, GPS loss, reconnects, tab changes and whole-frame fallback.
  Measure touch responsiveness, tearing, actual redraw cadence and USB bandwidth. The host's ~5 KB/s simulation
  and 20 Hz pose delivery are not measured board performance or a promise of 20 screen redraws/s.

## 2026-10-14 (Jetson, bench): ES3C28P map extension v2, board-side panning (Jetson side)

The user asked for a map that moves like the camera stream, with interpolation between GPS fixes, and chose the
board-scrolls option ("plan B"). Design, wire table and the firmware to-do list: NAVIGATION_PLAN.md, "Extension v2:
board-side panning". Nothing committed. The firmware half followed the same evening (entry above).

- **Wire (additive; v1 bytes and vectors unchanged, both header copies cmp-checked):** 0D PAN_CAPABILITIES (9 B),
  0E START_PAN (START_MAP's 27 B + background size), background and chrome images as ordinary E3 29 frames (told
  apart by size; the background START carries its world origin), E3 29 type 10 MAP_POSE (16 B, 1/16 px world
  position, course, marker, seq). Shared integer view math `computePanView()`. 20 vectors in
  `tests/pan_golden_vectors.txt`, new `pan_protocol_test` in `run_tests.sh`. Firmware without v2 answers 0D with
  BadCommand and keeps getting whole frames (checked against the pre-panning firmware: 555,014 B, suites pass).
- **Link:** asks 0D after 0A, START_PAN when available, `panSink()` with one pending background/chrome/pose, poses
  sent between any two records at most at the board's rate; falls back to whole frames if START_PAN is refused.
- **Navigation:** `PoseInterpolator` (one fix interval + 50 ms behind, never extrapolates), background planner (new
  background before the old one runs out, centred ahead), pan background/chrome compositing, 20 Hz service loop,
  next background's tiles fetched ~3 s early. `<Navigation>` gains Panning / PoseHz / PanBackgroundWidth/Height /
  PanJpegQuality; v0.4 wires `display.panSink()`.
- **Tests (all PASS, plain `make`, 0 warnings):** es3display_test [21] pan surface, [22] navigation -> link -> fake
  board over the pty; navigation_test [8] helpers, [9] service with a fake panning sink; config_test round trip.
- **Real-tile simulation:** `navigation_test --pan-drive OUTDIR` (needs `docker run --dns 1.1.1.1` on this bench)
  drives the Champs-Elysees route through a simulated board at 30 fps: 15 backgrounds in 155 s (51-60 KB at q75
  4:2:0, one every ~12 s at 60 km/h), 3,094 poses, 25 tile downloads, interpolation delay 350 ms. Video and frames:
  `~/drive_logs/es3c28p_20261014/pan_demo2/` (not in the repo).
- **Checked 2026-10-15 against the colleague's v2 firmware (working tree, no board):** both header copies still
  byte-identical; `run_tests.sh` (incl. pan vectors and the panning receiver/raster checks) and
  `run_decoder_tests.sh` pass; firmware build 0 warnings, 559,370 B. The firmware advertises 512x384 / 16+16 /
  30 poses/s, which are the Jetson's defaults, so no Jetson change is needed. Not reviewed line by line; the
  Jetson tests use a contract fake, not this firmware code, so the first real check of the pair is on the bench.
- **Open:** commit both halves (Jetson + firmware, all uncommitted); bench the pair (USB throughput, 4:2:0
  decode/draw time, redraw rate at 20 poses/s, fallback); CAN-aided motion between fixes (would cut the 0.35 s
  lag) is a possible later step.

## 2026-10-14 (Jetson, bench): ES3C28P map, Jetson side

Plan: `peripherals/es3c28p/NAVIGATION_PLAN.md` (the colleague's; reviewed and updated with the user's answers: MAP tab
as page 5, north-up, internet from a phone hotspot / car WiFi; a "warm map" section added at the user's request). The
plan's "Implementation record" lists everything below in more detail, plus the firmware to-do list. Nothing committed.

- **Map wire extension v1** (`Es3MapProtocol.h`, both copies cmp-checked): MAP_CAPABILITIES 0x0A (16 B), START_MAP
  0x0B (27 B, fixed map viewport 0,0,320,224), STOP_MAP 0x0C; map frames are ordinary E3 29 records; one receiver
  (NotReady while the other session exists). 17 golden vectors; C++11/17 and Python agree; Xtensa compile OK.
- **`lib/libnavigation`**: GPS freshness/marker state, Web Mercator (checked against the slippy-map formula),
  PNG validation, HTTP caching (max-age/Age/Expires/heuristic, stale-if-error/-while-revalidate, ETag/Last-Modified
  revalidation), SQLite LRU cache sparing the view, 320x224 compositor with a 5x7 font (Adafruit GFX glcdfont, BSD),
  4:4:4 JPEG, and NavigationService (4 fps shown, warm 1 fps hidden from disk only, no HTTP while hidden, 403 = off
  for the run, jittered backoff with Retry-After, stall-based HTTP timeout for slow hotspots, bounded stop()).
- **`lib/libes3display`**: `DisplayLinkConfig::map`, `mapSink()`, STATUS page polling (250 ms / 1 s / at once on
  hidden), camera on LIVE and map on page 5 with STOP-then-START switching, page 5 gated on the board's page count;
  firmware without the extension behaves exactly as before.
- **Config / app**: `<Navigation>` (14 settings, off by default, `validateNavigation()`); v0.4 starts one link for
  preview and/or map (link settings from `<DisplayPreview>`), one telemetry callback fan-out (CSV, GPS time, map), the
  map goes "last known" when the bridge drops; shutdown navigation -> preview -> link. `launchcode_v0_4.sh` mounts
  `$MEDIA_DIR/navcache` (the default `CachePath` was outside every mount).
- **Tests (all PASS, full plain `make`, 0 warnings)**: navigation_test (new: 7 groups incl. a fake HTTP tile server:
  no requests while hidden, User-Agent, 304 revalidation, restart on the cache, 403, 429+Retry-After, server down,
  HTML instead of PNG, bounded stop), es3display_test [20] (map surface vs a map-capable fake board: tap -> first map
  frame 40 ms, no camera frame after the switch, nothing sent on other tabs), config_test 15, run_tests.sh (37 Python
  + map vectors), v0.4 self-test, SIGINT -> exit 0 with navigation on (board absent; invalid settings).
  End to end (`es3display_test --fake-map-board`): v0.4 + UGREEN 1080p30 + preview + map, tabs cycling: recording
  kept 30 fps, 3 clean source switches, camera p95 49 ms, map 1 fps without GPS (redraws on change or 1 s).
- **Measured** (synthetic tiles): compose ~1 ms, encode 1.4-3.9 ms; warm map shown 4 ms after the tab, cold 53 ms.
- **Real tiles (later on 2026-10-14, network better):** the bench router (192.168.0.1) returns NXDOMAIN for every
  openstreetmap.org name (8.8.8.8 and 1.1.1.1 resolve them; the tile server answers in ~0.45 s per 34 KB tile). With
  `docker run --dns 1.1.1.1`, `navigation_test --live 48.8584 2.2945 16`: 4 tiles (128 KiB), frame 33.8 KB at q80
  4:4:4 (half the 64 KiB cap), encode ~6 ms. Fixed a stop-log bug (cache counters read after closing: "cache 0 tiles").
  Before relying on the map in the car, check `getent hosts tile.openstreetmap.org` on the hotspot.
- **Demo drive (2026-10-14):** `navigation_test --drive OUTDIR` spoofs 4 Hz GNSS along the Champs-Elysees (1,870 m:
  parked, 60 km/h, 3 s GNSS outage, stop) through the real service and OSM tiles: 479 frames in 129 s, 14 tile
  downloads (446 KiB), frames <= 35.8 KB, encode <= 6.1 ms; dot -> arrow at 5 km/h, last-known 1.5 s into the outage,
  arrow back at once, dot at the stop. Possible tweaks: marker colour vs OSM's blue POIs; small-font legibility on the
  panel; zoom 15 for more look-ahead. Outputs (not in the repo): `~/drive_logs/es3c28p_20261014/map_demo/`.
- **Saved outside the repo** (`~/drive_logs/es3c28p_20261014/`): both work logs (preview, navigation), the e2e/SIGINT
  scripts, the UGREEN bench frames (camera captures: never commit), the stage-1/2 agent reports, the demo outputs.
- **Colleague's firmware map work went into `91f01aa`** (committed with my changes on the user's "commit and push";
  not reviewed by me): `lib/es3Map/MapSurface.h`, `NAVIGATION_IMPLEMENTATION.md`, `tests/map_surface_test.cpp`, and
  changes in es3Graphics/es3UsbLink/es3Video/main/build scripts/tests. Checked before power-off: shared headers still
  identical, `run_tests.sh` passes (38 Python + map surface test), firmware compiles with 0 warnings (555,038 B).
  Next session: review it against the plan's "Firmware work" list and run the Jetson link against it.
- **Not done / open**: firmware side (plan's list; partly in progress by the colleague, see above); GPS-to-map end to end through the real C3 bridge (no
  bridge on the bench; covered by unit tests); a heading-up mode (out of scope by decision).

## 2026-10-14 (Jetson, bench): ES3C28P video preview, Jetson side

The plan is `peripherals/es3c28p/VIDEO_STREAMING_PLAN.md` (the colleague's plan plus my "Review additions" and the
"Frozen protocol v1" section). Built in stages by agents with independent reviews; two stages were cut short by usage
limits and finished by hand. Nothing is committed. The board itself is not here: everything below is tested against
fakes on ptys and synthetic JPEGs, plus the real UGREEN frames for timing.

- **Wire protocol v1** (`Es3Protocol.h` + new `Es3VideoProtocol.h`, byte-identical in `peripherals/es3c28p/lib/es3UsbLink/`
  and `lib/libes3display/`): control commands 07 CAPABILITIES, 08 START_VIDEO, 09 STOP_VIDEO; E3 29 video records
  (START/CHUNK/END/ABORT, RESULT/PRESENTED/CREDIT/VISIBILITY), reason codes, CRC-32, a bounded multiplexing parser.
  34 golden vectors in `peripherals/es3c28p/tests/video_golden_vectors.txt`, checked by C++ (C++11 and 17) and Python.
  Old firmware answers 07 with BadCommand, which the host reads as "video unsupported".
- **Recorder tap** (`SegmentedRecorder::setPreviewTap`): a leaky side branch split off before the RecordFps cap; its
  elements are `preview*`, and their errors are WARNs that never stop the recording. Rate limiting is a pad-probe
  limiter, not `videorate`, because `videorate max-rate` takes part in caps negotiation.
- **Pre-existing bug fixed on the way:** with a RecordFps cap below the camera rate *and* the luma tap, the luma
  branch's `videorate max-rate=2` pulled the **recording** down to 2 fps (6 frames in 3 s instead of 15 at RecordFps 5).
  Not live on the car (RecordFps 30 = camera 30 adds no cap). The luma branch now uses the same probe limiter; regression
  tests A9b/A15f. Also fixed: a preview failure just before `stop()` made `stop()` wait out the 4 s EOS timeout (A15g).
- **Config** `<DisplayPreview>`: 15 settings, `Enabled=false`; `validateDisplayPreview()` covers the cross-field rules;
  the timeout maxima match the protocol's 10 s ceiling (a test guards the drift).
- **`lib/libvideopreview`**: libjpeg DCT-scaled decode (1/4 for 1080p) → resize → baseline 4:2:0 JPEG, letterbox
  viewport (16:9 → 0,30 320×180), newest-frame input slots, a non-blocking `submitJpeg()`. On the 60 UGREEN bench frames:
  6.49 ms p50 / 6.51 ms p95 per frame, 8.2 KiB at q65 (a static indoor scene; night/motion still to measure).
- **`lib/libes3display`**: the single owner of the board's tty (exact configured path; never probes, the C3 shares the
  VID:PID). HELLO → CAPABILITIES → START_VIDEO, cumulative credits, one frame in flight by default, control requests
  ahead of video, TIOCOUTQ gating, heartbeats, reconnect, credit-stall resync. `es3display_test`: 18 groups against a
  fake board on a pty, incl. unplug/replug and an end-to-end 1080p JPEG → converter → link → fake board. Two test races
  fixed by hand (7/7 runs pass, 2 under CPU load).
- **v0.4**: validates the section (WARN + preview off on problems), starts link then converter, sets the tap before
  every recording start, stops recorder → converter → link at shutdown. The preview never gates recording.
- **Python** `es3_display.py`: protocol mirror, `capabilities`, and a `play` subcommand for bench playback.
- **Tests (all PASS, `bin/build_20261013_204351`, full plain `make`, 0 warnings):** config_test, videopreview_test,
  es3display_test, record_test (4 extra runs, ~144 s), commlink_sim_test, bridge_overlay_test, timesync_test,
  `dashcam_v0_4 --self-test`, `peripherals/es3c28p/tests/run_tests.sh` (33 Python tests). SIGINT → exit 0 with the
  preview on and the board absent, and with invalid settings.
- **Image:** rebuilt with the colleague's JPEGDEC 1.8.4 pin (`l4t-ml-gpio:latest` = `cbab0f86efe7`; previous archived as
  `build_20261013_211903`). The firmware compiles in it with 0 warnings (552,302 B / 54,036 B).
- **Compatibility with the colleague's firmware: compatible.** `src/tests/es3_firmware_fake.h` runs the firmware's own
  `VideoState.h` and parser on a pty, with `es3UsbLink::service()`'s rules mirrored (USB-task vs loop commands, no RX
  while a loop command is pending, 5 s session, 1 s TX stall, 4 KiB RX budget, events after replies); libjpeg stands in
  for JPEGDEC. `es3display_test` group [19]: handshake/limits, 30 frames in 3 s at 10 fps with 0 drops, credits after
  decode, hidden pause/resume, STATUS while streaming, heartbeats, converter frames pass the firmware's checks (4/4 runs).
  `es3display_test --fake-board PATH` serves the same fake to the real app. **End to end** (`bin/build_20261013_210026`):
  v0.4 recording the UGREEN at 1080p30 with the preview on, 30 s: 887 frames recorded (30 fps kept), 297 to the
  converter, 295 presented, 0 drops, ~6 KB per frame, capture→presented p95 77 ms on the pty, SIGINT → exit 0. The first
  UGREEN buffer of each session is not a JPEG and is skipped (logged once).
- **Compatibility findings (for the user / the firmware author):** (A) the firmware stops reading USB while a loop-handled
  command (STATUS, PAGE, ALERT, STOP_AUDIO) waits for the Arduino loop; one taking longer than the host's 500 ms reply
  timeout makes the host say HELLO again, which resets the video session (it renegotiates; tested). **Decided (user):
  longer host timeout.** `DisplayLinkConfig::loopReplyTimeoutMs` = 2000 ms for those four commands (PING, CAPABILITIES,
  START/STOP_VIDEO keep 500 ms); [19] now checks a 700 ms PAGE is answered with the session intact and a 2.5 s one
  times out and recovers; full build `bin/build_20261013_230621`, es3display_test 3/3, v0.4 self-test and the UGREEN
  end-to-end (295/296 presented, 887 frames recorded, exit 0) pass again. (B) **left to the firmware author (user passed
  the review on).** The two behaviour items were fixed by the firmware author on 2026-10-14 (07:02, both header copies
  still identical): the parser's timeout now rescans and keeps complete records, and `decodeAbort` turns reason Ok into
  Malformed. Verified: firmware tests (34 Python + C++), full build `bin/build_20261014_000916`, es3display_test,
  config_test, videopreview_test, v0.4 self-test, firmware compile (552,414 B, 0 warnings). The original findings were
  four minor protocol review items in the shared header (the parser's
  partial-record timeout also drops complete records it had absorbed; ABORT with reason Ok is accepted; the plan's
  Frozen text and the header disagree on the RESULT sequence; no independent content check of parsed control items).
  The plan says `videorate` for the preview branch; the probe limiter replaces it (TargetFps 11/13/14 give 10/12/13.3 at
  30 fps). Independent reviews of the converter, link and Python playback were cut off by the usage limit.

## 2026-10-12 (Jetson, bench): UGREEN work and the CYD toolchain

Only the UGREEN and the CYD were attached; the rig was not in the car. Nothing was committed, flashed or installed
with sudo, and dashcam-v04 stayed stopped. The captures taken (the room, no chart) stay in the session scratchpad.

**1. UGREEN: the 2026-10-11 library change on the real camera** (`bin/build_20261011_003045`, control ranges as
`double`)
- `usb_test`: 9/9 PASS (open/close, capture, attributes queued/live/reaching the device, invalid name, stop/restart,
  format cycling, controls put back). `--list-ctrls` is identical before and after.
- `record_test`: PASS, 0 failures (Part A and Part B on the UGREEN). `scan_cameras`: every range matches
  `v4l2-ctl` exactly, including pan/tilt −576000..576000 step 3600.
- **The UGREEN has no `exposure_dynamic_framerate` control** (UVC "auto exposure priority" is not exposed). The night
  analysis's "exposure A/B with `exposure_dynamic_framerate=0`" cannot be run as written. Capping exposure is what
  v0.4's frame-rate-priority loop already does (`libcamera_exposure.h`); at night it hands exposure to the camera on
  purpose, because the UVC gain range is far too dark.
- The deployed `dashcam.xml`'s wrong "autofocus stuck at 512" comment now states the 2026-10-06 finding. It is a
  comment-only change, checked well-formed; backup `dashcam.xml.bak-20261012-before-focus-comment`.

**2. v2 chart capture helper** (`tools/isp_tuning/chart/v2/capture_chart.py`; CALIBRATION.md, "Capture")
- Host `/usr/bin/python3`, refuses while dashcam-v04 runs. One continuous session per camera:
  - CSI: `tune_session.py`'s pipeline (`nvjpegenc` q95) at 30 fps, AE/AWB locked after a 7 s settle;
  - UGREEN: its own MJPEG, 1080p30 by default.
  - Output: a new directory under `~/drive_logs/chart_captures/`, named as `run_calibration.sh` expects, with
    `session.json` (controls before/during/after, ISP/overlay/module hashes, per-frame PTS, arrival time and hash,
    and a gap check for noise bursts). It prints the analysis command.
- UGREEN controls are written only on `--ug-set` (standard UVC only): mode switches first, values after. They are
  put back in reverse order, with signals ignored during the restore, and read back; a failure gives exit 3 and the
  fix command.
- **Tests:** `test_capture_chart.py`, 18 tests (simulated cameras, stub `v4l2-ctl`): captures, naming, spacing,
  burst gap detection (PTS and buffer offsets), 19 refusals, write order and restore, failed write, failed restore
  with an unwritable `session.json`, SIGTERM, SIGHUP, a second signal mid-restore, `--csi-prop` read-back, and the
  running-DT record. Ten mutants are each caught.
- **Live on the UGREEN:** a colour capture with no control writes (controls identical afterwards), and a 32-frame
  noise burst with fixed exposure/gain/WB: PTS steps 31.9–36.1 ms, 0 gaps, controls restored identically. The first
  live run found `v4l2-ctl --get-ctrl` printing menus as `3 (Aperture Priority Mode)`; it failed safely (exit 2,
  nothing written) and is fixed.
- **The CSI path has not streamed** (no CSI attached). Checked without the camera: the exact pipeline parses and
  links, all 11 `--csi-prop` keys take values of the right type on a real `nvarguscamerasrc` element, and
  `aelock`/`awblock` set. Its first chart capture is its streaming test.
- The prints: the user's photos show inkjet banding, paper texture, sheen specks and shadow steps 0–32 merged.
  These are fine for relative CSI-vs-UGREEN and base/candidate work (temporal noise cancels static print texture).
  They are not usable for absolute edge width or shadow steps below about 40. Use diffuse light at 30–45° to remove
  the specks.

- **Review** (three read-only agents, one per area; every finding came with a reproduction, and the main session
  re-checked the key ones). Capture helper: 3 medium, 8 low, all fixed except where noted:
  - a failed restore followed by a failing `session.json` write or controls read lost the exit-3 fix message. The
    restore now runs first in `finally` and prints at once;
  - SIGHUP killed it without a restore;
  - `--simulate --ug-set` wrote the real camera;
  - detail allowed more frames than `calibrate.py` takes;
  - noise allowed `--spacing`, and a gap still exited 0;
  - out-of-range `--csi-prop` values were silently kept at their defaults (now read back);
  - on-disk hashes could name a candidate that was not live yet (now the running device tree too);
  - `--out` outside drive_logs (note printed);
  - arrival-stamped PTS can hide a drop (offsets checked);
  - the UGREEN restore no longer waits behind the Argus teardown.

  Re-run live on the UGREEN afterwards: offsets 62, 63, …, 0 gaps, controls identical.

**3. CYD toolchain** (`peripherals/cyd/`)
- **No serial port:** the CYD enumerates as a CH340 (`1a86:7523`), but 5.15.199-tegra has
  `CONFIG_USB_SERIAL_CH341` unset (`usbserial`, `cp210x` and `ftdi_sio` are built). Ubuntu's `brltty` udev rule also
  claims `1a86/7523` (`SYSTEMD_WANTS=brltty-udev.service`; that service was running).
- **`host/ch341_module.sh`:**
  - `build` fetches `ch341.c` for stable v5.15.199 and refuses an unpinned sha256. That file is byte-identical to
    NVIDIA's R36.4.3/R36.4.4/R36.5 public_sources copy.
  - It builds against `/lib/modules/5.15.199-tegra/build`, then checks vermagic and 28/28 imported symbol CRCs.
  - Output: `~/drive_logs/tools/ch341_driver/out-5.15.199-tegra/` (`ch341.ko`, source, SHA256SUMS). It prints the
    sudo install and undo commands.
  - `check` is read-only (driver, tty per CH34x, brltty). Tested: unpinned source refused (exit 3), the NVIDIA copy
    builds, usage, temp dirs cleaned.
  - **Not installed: that needs the user's sudo.**
- **`build_and_upload.sh`:** the Linux twin of the README's Windows command (FQBN `esp32:esp32:esp32`); `auto`
  takes the single CH340 under `/dev/serial/by-id`. The bring-up sketch compiles to 384,162 B with no warnings.
  The upload is not run (no tty yet).
- **Image:** a new last layer adds Adafruit ILI9341 1.6.4 (`--no-deps`; GFX and BusIO come from the MKR layer). The
  Xtensa toolchain was already in the esp32 core.
  - The first rebuild died on a downloads.arduino.cc `EOF` in the MKR layer. That layer re-ran because
    `install_samd_aarch64.py` changed after the image was built (`a04f749`).
  - `build.sh` had already untagged `:latest`, so there was **no `:latest`**, which the service needs. I re-tagged
    the old image by hand.
  - Fixes: `build.sh` re-tags on failure (tested with a stub `docker`). The library installs retry 3 times, then
    check every name@version, POSIX-only since the base image's `/bin/sh` is dash (tested under dash, including a
    wrong version).
  - Rebuilt: `f2304ca58b60`, and after the review fixes `e8686f613fba` (the current `:latest`; no retry was
    needed, the intermediate image is deleted). 70 of 74 layers are byte-identical to the old image. The MKR firmware still builds to
    123,236 B / 23,924 B (the production numbers), and the C3 bridge builds.
- **Review fixes:**
  - The printed undo ran `modprobe -r` after `depmod`, which fails (kmod 29 cannot find a module it has
    forgotten); it now unloads first.
  - `check` prints the brltty fix command, honours an `/etc` rules override and commented lines, and accepts
    `masked-runtime`.
  - `build_and_upload.sh` no longer takes a mistyped sketch path as the port.
  - Image: `build.sh` keeps `:latest` during the build; the index downloads retry too; the Dockerfile comment
    now says RUN uses the base image's bash. Not changed: the version check greps the `lib list` table. The
    reviewer showed it cannot false-pass for libraries installed from the index.
  - The rebuild after these fixes hung for 20+ min on `CMSIS@4.5.0` (about 1.5 KB/s, no retry) and was
    cancelled when the session ended. `:latest` stayed on the working image, as the new `build.sh` intends.
    Cause: `install_samd_aarch64.py`'s `urlretrieve` has no timeout. It now sets a 60 s socket timeout, so a
    stall raises into its existing retry/backoff (shown against a local server that never answers).
- **Bring-up sketch review** (user asked to check the existing CYD code first; the only CYD code anywhere is
  `cyd_bringup_test`, from `2f6b3f6`, never run on a board). Checked against witnessmenow's
  ESP32-Cheap-Yellow-Display pin list and `User_Setup.h`, and the core 3.2.0 / Adafruit sources in the image:
  - The SD card was probed over the display's pins. `SPI.begin(14, 12, 13, 15)` remaps the global (VSPI) bus to
    the display, and `SD.begin(5, SPI)` calls `spi.begin()`, which returns early on a started bus. The card is on
    18/19/23 (CS 5), so it would always read `NOT FOUND`. The display now has its own `SPIClass(HSPI)`; the
    global bus is started on the card's pins.
  - The backlight (GPIO 21, HIGH = on) was never driven, so the panel would most likely stay dark. It is now driven HIGH.
  - `TFT_RST = 4` is the red LED (active low). The panel's reset is tied to EN, so the sketch now passes -1 and
    Adafruit sends a software reset instead.
  - Not changed: the serial `display OK` is printed unconditionally (README now says to look at the screen).
    Some CYD batches use an ST7789 panel rather than an ILI9341; if the screen shows wrong colours or garbage,
    that is the first suspect.
  - Compiles to 384,186 B, no warnings.
- **Driver installed (user's sudo, 16:18):** brltty-udev masked; `ch341.ko` in
  `/lib/modules/5.15.199-tegra/updates/`, loaded; `check` OK, CYD = `/dev/ttyUSB0`
  (`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`). A mangled paste left an empty `/lib/modules/updates`
  (empty `$K`); the user removed it. Give sudo steps with literal paths, one per line.
- **Chip:** ESP32-D0WD-V3 rev 3.1, 4 MB flash (0x68/0x4016); esptool auto-reset works.
- **Flash backed up before any write:** `~/drive_logs/cyd_backup/factory_flash_20261012.bin` (full 4 MB, sha256
  `0c80861d…`, `verify-flash` digest matched), plus the boot log and flash-id output. Not for the repo. Restore:
  `esptool --port /dev/ttyUSB0 write-flash 0 factory_flash_20261012.bin` (in the container).
- **What was on the board is not a factory demo.** `app0` is a custom dashboard app built 2026-07-20 on a Windows PC
  with arduino-esp32 **3.3.11** (IDF v5.5.5): the strings include `DASHCAM LIVE`, `[cyd]` logs for SD splash
  (BMP/RGB565), LDR auto-backlight, touch calibration saved to SD (`CYD_TOUCH_CAL_V3`), and DAC audio on a GPIO.
  Partitions are the default layout (app0 0x10000, app1 empty, spiffs, coredump). **It crash-loops at boot:**
  `E (163) ADC: CONFLICT! driver_ng is not allowed to be used with the legacy driver`, abort, reboot, every
  ~0.2 s, before `setup()`. IDF 5 aborts when the legacy ADC driver and the new oneshot driver (Arduino's
  `analogRead`, e.g. the LDR) are linked together. The legacy one is most likely pulled in by audio through the
  legacy I2S/DAC API; that is inferred from the strings, not seen in source. **Its source is not on the Jetson
  or on any branch**; asked the user where it is.

**4. Dashboard app source** (the user copied a colleague's uncommitted work into `peripherals/cyd/` at 16:34). It is
the app that was on the board. Layout: `cyd.ino` marker, `src/main.cpp` (five placeholder pages), `lib/cydStorage`
(SD on VSPI 18/19/23/5, 20 MHz with 10 MHz fallback, `/cyd_touch.cal`), `lib/cydGraphics` (ILI9341 on HSPI, bit-banged
XPT2046, 12-point touch calibration with a check screen, LDR GPIO34 to backlight PWM on GPIO21, SD splash
`/splash.rgb565` or `.bmp`), `lib/cydAudio` (8-bit mono 8–16 kHz WAV from SD), `assets/`, `tools/bmp24_to_rgb565.py`,
`BuildAndUpload.cmd`, `platformio.ini`.
- **The copy overwrote my uncommitted `build_and_upload.sh` and `README.md`.** Its own `build_and_upload.sh` (builds
  the app with the three `--library` dirs) is kept, plus my `auto` port (tested with a stub `arduino-cli`: no arg,
  `auto`, explicit port, and none found, which exits 1). The README is the colleague's, with my "From the Jetson",
  CH340 driver and bring-up sections merged back in.
- **Boot crash, root cause confirmed:** `cydAudio` used the legacy `driver/i2s.h` built-in-DAC mode, which links the
  legacy ADC driver; `analogRead()` for the LDR links the new one. The original build on core 3.2.0 contains both
  `adc_oneshot_new_unit` and the legacy `check_adc_oneshot_driver_conflict` constructor, the one that aborts.
  **Fix:** `CydAudio` now uses `driver/dac_continuous.h`, with the same public API. GPIO26 = `DAC_CHANNEL_MASK_CH1`;
  APLL clock, because PLL_D2 cannot go below 19.6 kHz; 8×256 B DMA; non-blocking writes (timeout 0, keeping the
  remainder), up to 4 blocks per `update()`; a natural end lets the queued tail play; `stop()` deletes the channel
  (enable/disable could replay stale descriptors). The rebuilt ELF has no legacy ADC or I2S symbols. Also fixed two
  bugs in the old version: `stop()` at end-of-data zeroed the DMA and cut up to ~96 ms of the tail, and one block per
  loop could not keep up with 16 kHz.
- `kTftRst` 4 → -1 (red LED, same as the bring-up sketch).
- Builds on 3.2.0 with `--warnings all`: no warnings, 442,402 B.
- Not changed, noted: `platformio.ini` targets PlatformIO's stock `espressif32` (Arduino 2.x, no `ledcAttach`) and
  declares no `lib_deps`, so it probably does not build. README's `darkRaw`/`brightRaw` "window" only sets polarity in
  the code. A missing splash triggers an SD remount (100s of ms) on every LIVE redraw.
- Assets: `splash.bmp` is an illustration (dashcam over a road), not a capture; `splash.rgb565` is exactly its
  conversion; `alert.wav` is a 0.35 s 8 kHz tone. The artwork's origin and licence are unknown; settle that before the
  public repo gets it.
- **Flashed 2026-10-13** (user's OK) with `build_and_upload.sh auto`. Boot log: `SD mounted at 20 MHz`, `calibration
  loaded`, `RGB565 splash rendered ... 130 ms`, `audio ready: GPIO26 DAC`, `audio playing: /alert.wav`, LDR ~980 and
  backlight 32/255; no crash. The first flash printed ~20 `dac_continuous ... Get available descriptor timeout` lines
  per alert: the driver logs every full queue on a zero-timeout write. `begin()` now sets that tag to ESP_LOG_NONE
  (update() still reports real failures); reflashed, log clean. Audibility and touch not yet checked by a person.

## 2026-10-11 (MKR pan library): encoder-aware servo prototype, no deployment

User requested continued MKR Zero single-axis servo work and delegated hardware
selection, with a US$100 budget **per** servo/encoder and reasonable weather
resistance. Existing `ServoFunctions` was open-loop and unused by production;
left it unchanged. New code is isolated under
`peripherals/mkr_zero/libraries/EncoderPan/`; its README has sources, prices,
wiring, build commands, interface contract and staged bench procedure.

- Choice: Hitec D646WP positional PWM servo (manufacturer lists $64.99, IP67)
  plus AS5048B absolute I2C encoder (adapter listed $19.40, remaining distributor
  stock). Encoder PCB is **not weatherproof**: sealed housing, protected magnet,
  connectors and an output-shaft mounting arrangement remain to be designed.
  Optional differential I2C extenders for the outside cable; never long bare
  I2C or reuse the GRF RS-422 pairs. Separate regulated servo power/cutoff.
- Controller uses integer millidegrees, rate limits and optional bounded P trim
  (default off). Safety uses raw encoder data; integer IIR only feeds trim.
  Explicit calibration/arm, expiring sequenced commands, no homing sweep,
  encoder age/order/jump/travel checks, following-error timer and latched faults.
  A renewed command cannot conceal expiry or restart the stall timer.
- AS5048B read-only driver checks defined diagnostic bits, acquires high-low-high
  with bounded carry retry, accounts for read duration in angle uncertainty and
  requires this repo's bounded Wire with its one-byte-read fix. No OTP writes.
- Gated PWM backend keeps startup pulses hidden and detaches on fault/stop;
  sample uses A5 PWM and proposes A4 for a default-off enable circuit. Those
  pins were the two servo reservations; GRF/C3/CAN pins are untouched. Servo
  uses TC4, distinct from CAN's TC3. **PWM suppression is not guaranteed torque
  removal**; independent cutoff/watchdog still required. Third-party Servo timer
  synchronization waits remain unbounded; do not claim unattended hardening.
- Feedback is measured angle plus uncertainty and a past observed stable
  interval, compatible in meaning with Orin `ServoPort`. Clock mapping, boot
  identity, protocol framing/acknowledgements and shared production scheduling
  are **not integrated**. No second native-USB reader; raw-CAN capture owns it.
- `PanBench` is a standalone encoder monitor by default. Both calibration and
  installation interlocks are false. Optional manual targets have 250 ms TTL;
  this is not a host command protocol. Stock sensor resolution/backlash does
  not establish adequate 100 m pointing: measure repeatability and beam-to-camera
  alignment under vibration and temperature first.

Validation: host tests pass **633 checks**, both GCC static analyzer/strict
warnings and ASan+UBSan. MKR compile with the existing container, SAMD 1.8.14,
Servo 1.3.0 and vendor Wire passes; two unused-parameter warnings are in the
upstream Servo source, not this library. Default-disabled sketch: 20,628 bytes
flash, 3,328 bytes static RAM. Existing GRF protocol/FSM tests and all 36 Orin
LiDAR tests also pass in the container. No servo/encoder live test, firmware
flash, service restart, commit or push. Stereo and production telemetry are
unchanged; unrelated dirty work has been preserved.

## 2026-10-11 (Jetson, rig stripped): the hardware-free open items

The rig had only the Jetson attached. A workflow took every open item that needs no hardware. Usage limits cut it off
twice; the main session finished the rest inline and re-checked what the agents had left. The three analyses below
did not get their independent reviewer; their headline numbers were spot-checked instead. Nothing was installed,
flashed, committed or restarted. Outputs that hold camera frames or vehicle data stay in `~/drive_logs/`.

**1. The CSI mount is loose, not "slipped once"** (`~/drive_logs/night_drive_20261003_analysis/verify2/mount/`)
- **Method:** a car-fixed dashboard mask, ECC and coarse alignment between 15 s windows every 10 s, refined to 1 s,
  plus the dashboard edge's row and tilt (`changes_raw.csv`, `change_brackets.csv`, `periods_pose.csv`).
- **Result:** **23 pose changes** (C01–C22) and 24 stable periods over 22:08–23:39. The dashboard edge tilt went:
  - 19.4° parked;
  - 14.9° after the 22:10 handling;
  - 7.5° after 22:20:13;
  - drifting through 10–12° to 15.7° by 23:01;
  - 11.8°, then 1.4° at 23:19, −1.4° at 23:21;
  - 6.3–7.1° to the end.

  Single jumps reached 250 px. Some changes are excursions that return, e.g. 23:05:46–:52 (+257 px, then −274).
- **Shocks:** 12 of the 31 steps fall in the top 10% of IMU linear-accel peaks (about 3 expected by chance), and 18
  in the top 20%. The camera moves when the car hits something.
- **Checked by eye** on frames at 22:40, 23:10, 23:20:10 and 23:25: the dash edge is clearly tilted, then nearly
  level and lower, then at a third angle.
- **Recommendation:** a rigid bracket. The earlier "about 4° for the rest of the drive" (REPORT F1) is wrong.

**2. MKR clock, telemetry stalls and the "clock break"** (`verify2/clock/v3/`, scripts in `verify2/clock/scripts/`)
- **The drift is real.** Three references agree within 48 ppm per 10-min block: GPS seconds, the C3's millis
  (neither uses host time) and the host. The MKR runs **+1278…+1300 ppm fast at 22:15, falling to −58…−211 ppm by
  23:45–23:54**; cumulative +2.9 s. REPORT's −2.64 s was too low.
- **What the firmware does:**
  - `master_ms` is plain `millis()` at frame build; the C3 copies it verbatim, and it is never re-based.
  - Core 1.8.14 locks the DFLL to the 32 kHz crystal (MUL 1465), which would give a constant +106.7 ppm.
- **What the logs show instead:**
  - +230…+1034 ppm in the first minute after each of 8 boots;
  - about 800 ppm apart between boots at similar temperatures;
  - within a boot it tracks temperature and time (block r 0.99 / −0.98).
- **So the DFLL is not regulated to the crystal.** A bench check needs the MKR (open work).
- **The "0.078 s host / 5.211 s master" break** in the EKF section is row 1 of `telemetry_20261003_221449.csv`:
  - a frame built 22:14:44.8 that sat in the C3's USB buffer across the manual v0.4 restart and arrived at
    22:14:49.933 (its GPS time reads 15:14:44Z; the next row 15:14:49Z);
  - host−master offsets on both sides agree within 6 ms;
  - with that row dropped, `fusion_input` accepts all 58,686 rows of the boot.
- **The five stalls (REPORT F13) are confirmed:** 403, 819, 912, 357 and 581 ms, 22 frames, exactly the C3's
  `host_tx_dropped` rise (48 → 70).
  - Each time, one frame is held 253–815 ms in the C3's USB CDC TX buffer (room for about one 186-byte frame), then
    2–7 are dropped.
  - The MKR, the Jetson's USB and the UGREEN were all fine; the cause is on the C3 USB link.
  - Code reading: arduino-esp32 3.2.0's HWCDC only restarts sending from its IN-empty interrupt or a `write()`, and
    hostLink skips `write()` when a frame does not fit. A larger TX buffer, or re-arming on a refused frame, is the
    candidate fix; bench it.
- **`host_ms` is not an absolute clock either:** ±30 ms wander on 10-03; +466 ppm for 90 min on 10-08. Time events
  through a lower envelope of host−master, or GPS UTC when the fix is valid.
- **Proposal for `fusion_input.py`** (not edited):
  - drop or re-time rows more than about 150 ms above the running envelope, including the first row after a reopen;
  - split rather than abort on a master reset or a persistent step over 1 s.

**3. Earlier camera issues against this drive** (`verify2/coverage/coverage_status.csv`, about 30 items)
- **Most earlier claims hold.** Corrections:
  - the UGREEN did return to 30 fps once, for 1.9 s under a lit toll-plaza canopy at 22:39:37 (the same second as
    the CSI's only gain dip); night mode correctly stayed;
  - "0 v0.4 warnings" holds for the log files only. The journal had the obsolete-settings WARN at both starts (fixed
    since; see 6) and two Argus connect errors from camera discovery;
  - the analyst's UGREEN flicker test could not see 100 Hz at 20 fps. A proper banding test at five lamp-lit stops
    finds none;
  - the CSI stopped in 0.88 s at the session end, not ~5 s (one sample).
- **Newly measured:**
  - the overlay matches the telemetry (speed 0.03 km/h, heading 0.2°, position 0.05 m median error);
  - dashes appear exactly when the bridge data is missing;
  - all 42 segments have their `.ass`, at about 5 Hz with gaps ≤ 0.23 s;
  - CSI warmer than the UGREEN, R/G ×1.11–1.17 (pixel-matched);
  - a faint 100 Hz line in the CSI at one stop (0.2–0.5% of brightness, invisible);
  - the 60 fps device-tree default does not affect the 30 fps recording;
  - no first-frame timeout or stall in either run;
  - a plate was legible at 8–10 m in the UGREEN in one lit scene, not in the CSI.
- **Not checkable:** v0.4's exit status at 00:10:35 (the journal after 23:39 is gone), autofocus in motion, absolute
  colour.

**4. High-G events and CAN speed** (`verify2/highg_speed/`)
- **66 latches, not 64:** 64 in the MKR console window, 2 after it ended. Spot-checked: the telemetry `highg_count`
  rises by 66.
- **All 66 were road shocks:** 64 confirmed, 2 unconfirmed; 0 harsh braking (peak deceleration 0.25 g); 0 spurious.
  - The vertical axis dominated in 56.
  - 0 latches while stopped with the engine on, although corrupt bursts kept coming.
  - CSI frame motion separates events from speed-matched controls (AUC 0.96, lit scenes).
  - What was hit: rumble strips, humps, raised crossings, expansion and bridge joints.
  - e02/e03 are one bump counted on both axles (the 500 ms hold is shorter than the axle gap below ~17 km/h).
- **The threshold:** 2 g per axis includes gravity, so the vertical axis trips at +1 g of shock. It fires 44 times
  per moving hour: a roughness counter. Polled peaks say at least 10 / 3 / 1 of the 66 would still fire at 2.5 / 3 /
  3.5 g.
- **Recommendation:** keep 2 g as a counter, and add an incident trigger at 3 g or on the horizontal axes with
  gravity removed. Leave the duration alone until a burst log exists.
- **Speed:**
  - CAN/GNSS = **0.98896** (95% interval 0.98860–0.98940), spot-checked 0.9898; 0.9937 at 30–50 km/h down to 0.9875
    above 90.
  - GNSS arrives 0.46 s after CAN for the same instant.
  - CAN distance is 1.05% short. The 2026-09-26 drive gives 0.9912.
  - For true road speed the map scale would be 0.0101 (×1.010 ± 0.003): needs the user, an SD-card reinstall, and a
    daylight check.

**5. Raw-CAN logger bounded under a hung disk** (`peripherals/mkr_zero/tools/canstream/`, uncommitted)
- **The bound:** the reader → writer hand-over is now `Handover`, capped at 64 MiB charged as bytes plus 160 B per
  event. That is at least 4.7 min of the Brio's bus in the worst case (one frame line per read).
- **When full:**
  - the reader drops whole lines, never blocks, and keeps draining the port;
  - the counts are exact per destination file;
  - it queues again only after the writer has freed 1/8 of the bound, so one hang costs one contiguous gap.
- **Reporting:**
  - `can_stats.csv` gains `rx_dropped`/`fs_dropped` at the end (an older file keeps its header and width);
  - an episode is reported at its start on stderr (from its own thread) and at its end in `mkr_console.txt`,
    `can_sync.csv` and stderr;
  - the end note gives totals and the queue's peak.
- **Tests:** `make check` in the container: **145 tests OK** (134 before), 1 intentional skip. Four mutants (bound
  removed, drop counter, byte count, rx_frames accounting) are each caught. The README documents it.
- **Consumers:** no other tool reads `can_stats.csv` by position; `drive_watch.sh` reads it by header name.

**6. Drive tooling and the deployed config** (outside the repo)
- **`drive_watch.sh`:**
  - new checks: `can_stats.csv` growth, MKR ringdrop, host loss (Δ`streamed` − `rx_frames`, plus `rx_dropped`),
    and `can_raw.log` not growing while the MKR drains frames;
  - columns by header name; grace for files that do not exist yet; path variables for tests.
  - Tested on a fake growing session with stub `docker`/`systemctl`: quiet when healthy; each new alarm fires after
    two strikes.
- **`drive_session.sh`:**
  - tegrastats every 1 s;
  - `stop` refuses (rc 3, loud warning) while v0.4's telemetry shows the car moving in the last 120 s; `--force`
    overrides; shutdown and signals still stop at once.
  - Tested: moving → refused; `--force`; parked; stale data; empty cells.
- **Deployed `dashcam.xml`:** `<Encoder>`, `WarmupFrames`, `RecordWidth` and `RecordHeight` removed (exactly those;
  well-formed; backup `dashcam.xml.bak-20261010-before-obsolete-cleanup`). This ends the obsolete-settings WARN at
  every start.

**7. IMX296 candidates staged** (`~/drive_logs/tools/imx296_driver/staging_20261010/`, README there; nothing
installed)
- **`imx296_debugoff_5.15.199.ko`:**
  - only `#define DEBUG 1` removed;
  - 40/40 symbol CRCs match 5.15.199;
  - the preprocessed sources differ only where `dev_dbg` became `if (0)`; `set_exposure`, streaming and `write` are
    instruction-identical.
- **Overlays:**
  - `cand1` (gain cap 300 = 30 dB, exposure cap 1500 lines ≈ 22 ms) and `cand2` (gain 480);
  - the stock overlay round-trips through `dtc` byte-identically, and each candidate differs in exactly those two
    properties.
- **The unit mismatch, which decides the choice:**
  - Argus models gain as linear (value/16) while the driver writes 0.1 dB: 300 is 18.75× to Argus, 31.6× real; 480 is
    30× vs 251×.
  - Exposure is in lines while Argus thinks microseconds. At 60 fps the 1500 cap meets the vmax clamp (SHS1 = 0):
    stay at 30 fps until that is seen to behave.
  - Recommendation: cand1 first.
- **`install_candidate.sh` / `rollback.sh`:**
  - dry run by default; sha256-checked; refuse unknown files or a busy camera;
  - every backup goes to its own directory; `rollback.sh --stock` always returns to stock.
  - Tested on a fake root, which caught and fixed a backup collision for two installs in the same second.
- **For the A/B night, keep the DEBUG-on module:** the gain time series comes from its debug lines.

**8. Code backlog**
- **`read()` warnings:** `libcan.cpp` and `libgpio.cpp` ignored `read()` on the wake pipe; GCC does not honour a
  `(void)` cast for glibc's warn_unused_result. The result is now bound and documented. **The build has 0
  warnings.**
- **Control ranges:** `cameraAttribute` and `CameraAttributeInfo` hold them as `double`, so every int32 is exact
  (`as_double()` in libconfig).
  - A new `config_test` check round-trips int32 min/max and a 2^24+1 step: it **fails on the old float code**
    (shown) and passes now.
  - Plain `make` (`bin/build_20261011_003045`): `config_test`, `bridge_overlay_test`, `dashcam_v0_4 --self-test`,
    `focus_test`, `usb_test --self-test`, `commlink_sim_test`, `camera_gst_test` and `record_test` (Part B skipped,
    no camera) all PASS.
- **Left as is:** the `const_cast` in `Camera_GST::getCameraStatus()`. Removing it needs a non-const `iCamera`
  interface or `mutable` state, and the comment already explains it.

**Not done:**
- the v2 chart acquisition helper;
- a units-correct imx296 driver;
- the `fusion_input.py` stale-row change;
- the canmap speed scale;
- any firmware change (High-G trigger, C3 TX buffer, DFLL logging).

All of these need the user's decision or hardware to validate.

## 2026-10-09 (Orin): separate GRF-250 / IMX296 ranging prototype

User ordered GRF-250, requested an RS-422 link through MKR and a separate
single-pan-servo interface; subsequently supplied a photo of the **3.3 V variant**
of the converter board and plans an **encoder-equipped servo**. Stereo work is
preserved, including the pre-existing untracked `lib/libstereoprototype/`.

Implemented (committed 2026-10-10 at the user's request, with `lib/libstereoprototype/`):

- `lib/liblidartracking/`: Orin Python association library, optional OpenCV
  rectification + COCO YOLOv8 ONNX detector adapter, bounded IoU tracks, explicit
  target selection, conservative beam/ROI and timing gates, range/closing-speed
  filter, CRC USB decoder, round-trip clock mapping, synthetic demo/JSONL replay.
- `peripherals/mkr_zero/libraries/GRF250/`: portable heap-free C++ UART parser
  and startup FSM, checked against the user's Rev3 guide and manufacturer Rev5.2.
  Identifies device, reads firmware, writes/reads back raw first/last + strength,
  lost count1, 20 Hz and stream5. Preserves lost-return sentinel. No flash-save,
  sensor-reset/upgrade or laser-enable command. ID45 five-return layout remains
  deliberately unsupported because the guides disagree on its byte count.
- `peripherals/mkr_zero/helper_scripts/GRF250Bridge/`: **bench-only replacement
  sketch**, SERCOM0 A3/D18 TX and A6/D21 RX (ALT, pad0/pad3). Serial1/C3 and
  A4/A5 servo reservations untouched. Bounded USB writes, sample sequence and
  micros stamps, nonce clock replies and 1 Hz firmware/health counters. No
  production scheduler/USB mux changes.
- `ServoPort` interface only: pan requests expire; measured encoder angle,
  angular-error bound and observed settled interval must cover the acquisition
  interval. No PWM/motor control or simulated feedback in hardware paths.
- Wiring, protocol contracts, limitations and staged bench/calibration procedure
  in `lib/liblidartracking/README.md`; placeholder calibration is fail-closed.

Verification:

- Existing `l4t-ml-gpio:latest` container: 36 Python tests passed, including
  real OpenCV preprocessing/rectification with injected model predictions and
  a real POSIX tty path against a pseudo-terminal. C++ protocol/FSM tests pass
  with `-Wall -Wextra -Wpedantic -Werror`, including 100000 noise bytes,
  single-bit frame mutations and configuration mismatch rejection. GCC static
  `-fanalyzer` also passed.
- Host and container C++ AddressSanitizer + UBSan passed. Host non-camera tests pass; camera
  tests skip because the existing host cv2 / NumPy2 ABI mismatch persists.
  Do not change system Python packages to address this task; use the container.
- Arduino SAMD 1.8.14 compile-only, warnings all: 14892 bytes flash / 3096 bytes
  globals. No new warnings. Existing stereo suite: 30/30 passed in container.
- No physical LiDAR, encoder, actual ONNX model inference or moving-vehicle
  ranging tested. No ±1 m at 100 m validation. No hardware flash, service stop,
  camera ownership change, commit or push performed.

Important next steps / electrical blockers:

1. **Verify converter IC and levels before wiring.** Photo labels A/R+, B/R-,
   Y/T+, Z/T- show full duplex, but do not prove chip, level shifting or isolation.
   Bare MAX490 is 5 V; a vendor's 3.3 V variant must use an appropriate transceiver
   or level conversion. MKR inputs are NOT 5 V tolerant. Verify DI/RO rather than
   trusting ambiguous TXD/RXD silk/arrows. GRF is TTL UART: use a transceiver at
   **both** ends of the external RS-422 cable. Separate sensor/servo power.
2. **Do not flash this helper over production telemetry casually.** It replaces
   CAN/IMU/GPS. Native MKR USB is already the raw-CAN production path. A future
   integration must add LiDAR records to the existing single reader/mux, health
   and boot-epoch reporting, and scheduler/watchdog budget tests; no second tty
   reader. The C3/host protocol is unchanged.
3. Bench-verify delivered sensor firmware, cm units, raw bytes, sentinel, latency,
   power and serial recovery. Measure camera intrinsics/mount/pan geometry and
   encoder backlash. Sensor reception time is NOT acquisition time: unknown
   latency bounds invalidate association. Encoder must report an observed stable
   interval, not commanded position or a future promise.
4. Calibrate and validate against measured static and then controlled moving
   targets. Bounding boxes are not object masks; IoU is not guaranteed identity.
   A horizontal servo cannot fix pitch/vertical mismatch. Defaults reject bad
   alignment, target ambiguity, distinct first/last returns and stale data.
   Results expire; consumers must clear them on expiry or transport loss.

## 2026-10-07 (Orin): IMX296 v2 chart and offline calibration analyser

**Implemented and tested offline; no camera/ISP/service changes.** Original work
is at `/home/jetson/drive_logs/tools/isp_tuning/chart/v2/`. Reusable sources,
documentation, geometry and printable assets are now version-controlled under
`tools/isp_tuning/chart/v2/`, with the existing range-aware `csi_decode.py` copied
unchanged to `tools/isp_tuning/`. The repo wrapper additionally mounts the repo
tool directory read-only so it works outside `drive_logs`. See
`tools/isp_tuning/README.md` for local-vs-repo paths. Original v1 chart/tools are
untouched. Captures, generated analysis results and caches are not committed.

### Delivered

- **Three A4 landscape vector sheets:** colour/neutral balance (24 colour
  patches plus six repeated greys), noise/shadows (six large flat regions and
  ten shadow steps), and detail (four 5-degree slanted edges plus four texture
  periods). The PDF contains no embedded JPEGs; SVGs, lossless 300 dpi PNGs,
  preview, geometry and generator are included. IDs 10-21 distinguish v2 from
  legacy IDs 0-3. Print at 100% and verify the 100 mm ruler.
- `calibrate.py`: saved-image colour comparison against USB, regularized
  neutral-preserving matrix fitting with six held-out chromatic patches,
  native-pixel temporal noise (32-256 frames; recommend 64), separate spatial
  variation and lag-1/4/8 difference statistics, relative edge width/halo and
  texture diagnostics. Writes JSON/Markdown reports and annotated ROI/swatch
  previews to a new directory, refusing overwrite.
- Gates wrong/missing markers, poor geometry/coverage, motion, lighting drift,
  clipping, repeated-neutral inconsistency and duplicate frames. Retains the
  rig's limited-range nvJPEG correction and records the selected decoder.
- `run_calibration.sh`: offline Docker execution in `l4t-ml-gpio:latest`, no
  network/GPU/devices/Argus access; inputs read-only, only new results writable.
  Usage and method limitations: `chart/v2/CALIBRATION.md`.

### Verification and limitations

- **24 synthetic tests passed** in the existing container, including a rerun
  from the packaged repo copy: geometry and failure
  gates, known matrix recovery/holdout, independent/correlated noise, a full
  32-frame noise command, Gaussian-blur response, decoding and overwrite guards.
- Chart validation passed all 12 markers and 46 flat-patch RGB regions, native
  1456x1088 framing/perspective simulations and independent PDF rendering. The
  local colour/detail wrappers were exercised on the generated artwork; the
  repo-path detail wrapper also passed after packaging.
- Existing Oct 1 CSI/USB JPEGs selected the expected limited/full-range decoder
  paths. This was read-only verification, **not fresh live calibration**.
- The colour matrix acts on inverse-sRGB-linearized **processed RGB**, with USB
  as a comparative reference, not ground truth. It is **not an Argus ISP CCM**;
  do not paste it into or multiply it into `camera_overrides.isp`. No `.isp`
  output or automatic installation is provided. Home-printed RGB is not measured
  reflectance, and noise/detail figures include print/codec/ISP effects.
- Analysis cannot prove capture timestamps, sensor synchronization or control
  locks from still files. The old five-frame runtime sweep is insufficient for
  noise analysis; never combine different variants just to reach 32 frames.

### Next steps

1. Print/mount v2 matte at actual size, filling about 70-80% of native CSI width;
   verify all markers and ROI overlays in both cameras. Use controlled lighting
   and fixed exposure/gain/WB; a characterized target is needed for absolute colour.
2. Acquire new per-sheet captures and one continuous 64-frame noise burst per
   profile. **A dedicated v2 acquisition helper is not implemented**; the new
   script analyses saved files only. No live v2 print/camera validation yet.
3. Validate colour on held-out captures under another light, and assess noise
   jointly with detail retention and moving-scene ghosting. Any later ISP trial
   must be a separately reviewed base/candidate/base experiment, not direct use
   of the post-render matrix. Installed profile and recorder state were not changed
   or freshly certified by this work.

Local synthetic smoke-test reports remain in
`/home/jetson/drive_logs/tools/isp_tuning/chart/v2/results/`; these use chart
artwork, not real camera measurements. Turning EKF antenna measurement and
independent measured-drive validation below also remain outstanding.

## 2026-10-07 (Orin): GNSS + gyro + individual-wheel turning EKF

**Implemented in the read-only demo, not deployed into v0.4.** No firmware flash,
CAN transmission, camera change or service control. The earlier wheel-only demo
and C++ dead-reckoning library remain available and unchanged in behaviour.

- `peripherals/mkr_zero/tools/canstream/turning/fusion.py`: four-state EKF
  (heading, forward speed, yaw rate, gyro bias), individual front/rear wheel
  measurement models, GNSS antenna lever-arm correction, wrapped course residuals,
  analytic Jacobians, innovation gates and Joseph covariance updates. Produces
  equivalent bicycle front road-wheel angle, yaw and rear-centre turning radius;
  it is not a steering-angle sensor or an individual Ackermann tyre-angle estimate.
- `fusion_input.py`: bounded join of existing raw CAN and v0.4 telemetry CSV.
  Rejects bad sensor quality, repeated GNSS fixes, stale/late observations and
  inconsistent clocks. GNSS course is gated on movement; missing data is not zero.
  `--telemetry` plus `--fusion-config` enables fusion, including `--follow` for
  growing logs. No additional serial reader or CAN sender is started.
- Dashboard and CSV retain the wheel-only comparison and add fused estimates,
  model uncertainty, gyro bias, active sources and rejection/NIS diagnostics.
  The optional C++ trajectory remains wheel-only, not GNSS-corrected position.
- `synthetic-fusion` generates reproducible multi-sensor data with gyro bias,
  noise, a GNSS outage, isolated wheel slip and a gyro spike. Usage, equations,
  limitations and validation procedure: `turning/FUSION.md`.

**Rig inputs and required next step:** the user confirmed IMU **+Z up** and a
centred antenna at the bottom of the windscreen. Honda Vietnam's stock Brio RS
specification lists **front/rear track 1.475/1.459 m**, wheelbase **2.405 m**
(source linked in `FUSION.md`). The example config uses those track widths and
zero lateral antenna offset. **Measure the horizontal rear-axle-centre-to-antenna
distance forward in metres.** `fusion.brio_rs.example.json` deliberately leaves
`antenna_x_m` null and refuses to run until supplied; the synthetic 2 m value is
not a rig measurement. Verify gyro sign, tyres, geometry and sensor latency on
the private measured course before trusting results.

**Verification:** 27 fusion tests pass natively and in `l4t-ml-gpio:latest`,
including Jacobians, covariance positive definiteness, faults, file following,
CLI replay and HTTP endpoints. Original turning tests: 29 pass; unchanged C++
DR: 55 checks pass. Full canstream run: 134 tests, no failures, one intentional
skip; that run preceded two final tests, both covered by the 27-test reruns.

**Do not claim an accuracy improvement yet:** in the synthetic 60 s run (scored
after 10 s), yaw RMSE was 1.3721 deg/s raw wheels, **0.3777 smoothed wheels**, and
**0.4651 EKF**. Bias converged to 1.7661 deg/s versus injected 1.8; 52 observations
rejected, zero late. The existing smoother wins this clean-wheel, slow-changing
case. Independent held-out drives and noise/latency tuning remain necessary.
Evidence stays local at `/home/jetson/drive_logs/turning_ekf_check.NAQNaN/`.

**Timing limitation:** CSV logs host arrival time, not per-sensor acquisition
time, and lacks GNSS accuracy/fractional fix time. This join is approximate; its
uncertainty is not a calibrated error bound. The October 3 log check found a
boundary with 0.078 s host advance versus 5.211 s master advance, which fusion
correctly refuses to cross. Split/re-align such logs; the cause was not determined.
No live-drive accuracy validation was performed. Production integration should
first add acquisition timestamps/accuracy fields and validate clock mapping.

## 2026-10-06 (Orin): turning calibration demo and dead-reckoning library

**Implemented; not deployed to the production recorder.** No firmware flash, CAN
transmission, camera/profile change or service stop/restart was performed by this
work. The existing night-drive analysis edits below are preserved. This update
and all pending source/documentation changes are being committed together at the
user's request; raw captures and ignored build outputs stay local.

### Delivered

- `peripherals/mkr_zero/tools/canstream/turning/`: standard-library Python demo
  following the existing logger's candump file (no second serial reader), with a
  localhost-only passenger dashboard, replay, bounded history, CSV/provenance
  output, synthetic test data and candidate calibration fitting.
- Brio-specific wheel/gear decoding estimates left-positive yaw, rear-axle-centre
  turning radius and equivalent front road-wheel angle using a planar bicycle
  model. This is neither measured steering angle nor the individual inner/outer
  tyre angle; EPS assist torque is not used. Default map scales and the 2.405 m
  wheelbase remain provisional until independently verified.
- `turning/DRIVE_PROCEDURE.md`: measured 30 m straights and 10 m rear-centre
  circles, optional 15 m circles, left/right runs at 6–9 km/h, separate fit and
  held-out validation runs, independent gate/lap timing and pose-closure checks.
  The user confirmed a private measured area and a passenger are available.
  Driver watches the course, never the delayed display.
- Fit estimates tyre mismatch, wheel-speed scale and yaw scale from independent
  distance/lap references. It rejects overlapping or invalid windows and writes
  a **candidate only**, with baseline/validation errors; no automatic install.
- `lib/libdeadreckoning/`: standalone C++17 CPU library for the Orin, static and
  shared builds, narrow C API and Python binding used by the demo. Integrates
  rear-wheel speed/yaw into local rear-centre x/y/heading with constant-curvature
  arcs; supports forward/reverse, explicit reset and stale-state snapshots.
- Reuses MKR `rateLimit`, `saturate` and `SimpleMovingAverage` unchanged. Only
  Arduino/non-Arduino include guards changed in the two shared headers. A
  time-aware IIR smooths integration inputs; the original moving average is
  diagnostic only, avoiding its startup/lag effects on the pose.
- Invalid/stale/out-of-order samples, gaps and direction changes cannot bridge
  unknown travel. Continuity stays false until explicit re-anchoring. Default
  unanchored budgets are 120 s / 1000 m, not accuracy guarantees. Updates allocate
  no heap memory; the estimator is single-owner, with no IO or production wiring.
- Root `make deadreckoning` / `make deadreckoning_test` delegate to the standalone
  library without creating a timestamped app build or changing service selection.

### Verification and evidence

- C++: **55 checks pass** natively, in `l4t-ml-gpio:latest`, and under ASan/UBSan;
  GCC analyzer reported no diagnostics. Strict compiler warnings enabled.
- Python turning tests: **29 pass**, including the actual C++ binding. Full
  canstream suite: **109 tests, no failures, one intentional abstract-fixture
  skip**. Existing MKR host suites also passed; details in
  `lib/libdeadreckoning/VERIFICATION.md`.
- MKR compile-only in the existing Arduino container succeeded: **123236 B flash,
  23924 B global RAM**, unchanged. Vendor core/library warnings remain; no upload.
- October 3 baseline: all four wheel fields matched the existing C++ decoder on
  **13103 wheel frames** (333679 raw frames). This proves decoder parity, not
  steering or positioning accuracy.
- Synthetic calibration/replay exercised the real shared library. A single
  synthetic lap closed within **0.314 m / 1.81°**; this is software evidence only.
  HTTP page/state endpoints were checked; browser visual rendering was not.
- A **20 s hardware capture in the existing container**, with only MKR native USB
  passed through, received 20 FS / 21 console lines, zero malformed lines and
  **zero CAN frames**. Production DISCOVER firmware reported IMU/GNSS/C3 up and
  no I2C errors. The demo followed the growing log correctly with no data.
  **Live motion, angle/radius and position accuracy are still untested.**
- Evidence stays local at `/home/jetson/drive_logs/turning_demo_check.thjJHt/`.
  Do not publish raw vehicle logs or camera data.
- During that earlier test session, `dashcam-v04` went from active to a clean
  systemd stop at **11:39:52 local time**. This work issued no service-control
  command and left it inactive; recheck current service state before a drive.

### Remaining work / limits

1. Run the measured private-area course with a passenger. Check logger loss and
   clock-sync files, align independent gate timestamps, fit once, then evaluate
   held-out straights/circles and independently measured endpoint/heading errors.
2. Both rear wheel counts must clear the 300-count sensor cutoff; unavailable
   readings are not standstill. The demo is forward-only; the C++ API supports
   reverse and requires independently confirmed stationary input to accept zeros.
3. Candump display inherits about **2–3 s logger delay** and epoch-clock limits.
   Do not use it for steering guidance or physical gate timing. Production input
   should carry mapped monotonic acquisition timestamps, fresh gear and quality
   flags; small wall-clock steps cannot be repaired from candump alone.
4. No GNSS/IMU fusion, absolute anchoring, covariance or calibrated error bounds
   yet. Tyre mismatch, slip and unknown travel need independent validation and
   re-anchoring, not optimistic extrapolation. Integrate into production only
   after validation and explicit deployment approval.
5. For host-only library checks, prefer `make -C lib/libdeadreckoning check`:
   the root Makefile also probes unrelated app packages (`pugixml`, `spdlog`)
   absent on the host, producing warnings even though the standalone test passes.

## 2026-10-06 (Jetson): night drive 2026-10-03 analysed: camera findings (analysis only, nothing changed)

No code, config, device tree or firmware was changed. Proposals are at the end and are not done.
- **Outputs** (camera frames and vehicle data: local only, never publish), in
  `~/drive_logs/night_drive_20261003_analysis/`:
  - `REPORT.md`: the verified synthesis;
  - `synth/verification_status.csv`: every analyst claim with its status;
  - `synth/findings.json`;
  - scripts and data under `synth/`, `ugreen/`, `csi/`, `system/`.
- **Method:**
  1. three analysts (UGREEN, CSI, system);
  2. a synthesis pass that re-measured every high and medium finding by a different method;
  3. a main-session check of the key claims against images, logs and source.
- **Not finished:** a second check (CSI mount over the whole drive, MKR clock drift, coverage of earlier camera
  issues) was cut off by a power loss. Its partial files in `verify2/` are unverified.

**Correction first: the UGREEN autofocus did not fail.** In the car (22:08–22:16 section below) I reported it stuck
at 512 and blurred. That was wrong.
- **AF-on footage 22:08–22:14:50 is as sharp as fixed 690.** Laplacian variance, central band:
  - AF on: 733–874;
  - fixed 690: 754–849;
  - wrong manual positions: 141–573.

  Street-light halo was 12.9 px in both. From manual 1023, autofocus was sharp again within 0.7 s (3 times).
- **What misled me:**
  - `focus_absolute` does not follow the lens while autofocus is on. It keeps the last manual value (512, 1023, 650).
    So the 21:10 section's note "with autofocus on, `focus_absolute` is the live lens position" is wrong too.
  - The frame I judged came from `ffmpeg -sseof` on a growing MKV, which returns a stale frame.
- **Fixed 690 stays.** It is verified as sharp as autofocus and cannot hunt on wipers or rain. Autofocus while driving
  is untested.

**Other corrections to the sections below:**

| Earlier claim | Now |
|---|---|
| v0.4 run 2: 29 segments | **39** (304–342), 138,752 frames, 19.979 fps |
| CSI about 12 MB/s | **18.3 MB/s** average (7.4–26.5 per segment, about 25 on unlit roads); 12 was the first, parked segment |
| CSI rolled 5–10°, dashboard lower third to half | The mount **moved during the drive** (below) |
| Drive 22:17–23:38 | Car stationary until 22:20:04. After "drive done" (23:38) it kept driving 23:38–23:51 and 00:00–00:05 (CAN baseline parked 23:54–23:59) |
| Wall = mono + 1791040065.46 | **mono + 1791040066.389** (userspace journal, n=408, no drift). Kernel printk stamps drift up to +0.18 s, so use the journal's realtime column for kernel lines |
| "Night mode one-way" (analysts) | **Wrong as a bug.** The exit works: 3 exits on the 2026-10-02 bench (`log_20261002_153605.txt`). At night it should stay on: v0.4's 30 fps setting (32.3 ms, gain 15) gave luma 4.6–9.3 against 67.6 under the camera's AE |

**Camera findings, most important first** ([V] = re-measured or checked in source; [A] = analyst; [I] = inferred)

1. **The CSI mount moved at 22:20:13, the first braking after pulling away** (about 30 km/h, −0.66 m/s²). [V]
   - **Evidence:**
     - Dashboard match against the parked reference: NCC 0.88–0.92 until 22:20:08, 0.69 at :12, 0.06 at :14,
       never above 0.31 afterwards.
     - The dash vent row went from 15.8° to a median 5.8° tilt.
     - Roll against the level UGREEN went from about 13° (SIFT 10.3–11.5°) to about 4° (LSD median 3.9°, IQR
       2.9–6.0).
     - The view dropped about 230 px (about 13°). The car-fixed area went from about 25–30% to 44.5% of the frame.
     - Checked by eye on `synth/img/csi_move_222008_16.png`: the dash pattern visibly rotates between :12 and :14.
     - `csi_record.log` has one pipeline part, so this is not a mode or crop change.
   - **Earlier change:** a possible shift at about 22:10:30 while parked, just after the rig was connected (22:09:32;
     likely handled). [I]
   - **Open: the mount may have moved again later.** The roll estimate swings from about −8° (22:30–22:50) to +7…+8°
     (23:36–23:38). The 23:17 and 23:37 frames in `synth/img/csi_mount_montage.png` show yet another dash geometry.
     The synthesis compared against one parked reference only, so it could not see later moves. Treat the mount as
     loose until shown otherwise.
2. **The CSI is gain-limited all night by the device tree.** [V]
   - **Installed overlay** `tegra234-p3767-camera-p3768-imx296-cam1.dtbo`, mode0: `gain_factor` 16,
     `max_gain_val` 200, `max_exp_time` 1000, `exposure_factor` 1e6, `default_framerate` 60 fps. This matches the
     source overlay in `~/drive_logs/tools/imx296_driver/src/`.
   - **What the driver does with them** (`build_src/imx296.c`, identical to the built one):
     - Gain is written straight to the 0.1 dB register: 200 = 20 dB of the sensor's 48 dB (`IMX296_GAIN_MAX` 480).
     - Exposure is lines (`SHS1 = vmax − val`, 14.81 µs per line), so 1000 = 14.8 ms. Argus believes it is 1000 µs.
     - vmax at 30 fps is 2249 lines, so about 33 ms is physically possible.
   - **What happened on the drive:**
     - Sensor gain ≥ 199 for 99.91% of the drive (35,527 gain writes; one 2.1 s dip at 22:39:37).
     - AE held output brightness level (median luma 89 lit vs 86 unlit) [A], so the rest came from ISP digital gain,
       which shows up as noise.
3. **CSI noise on unlit roads.** [V]
   - Classed by UGREEN brightness, lit vs dark:
     - pixel noise 5.6 vs 29.4 DN;
     - temporal noise 11.0 vs 48.8;
     - row banding 2.4 vs 9.5 DN;
     - JPEG 383 vs 843 kB.
   - Row banding is full-width and present on lit roads too: row variance 30–44× white noise, columns about 1×.
   - Analyst [A]: clipped headlight pool 22.1% vs 2.7% of the frame, crushed blacks 20.8% vs 3.6%.
4. **The CSI bitrate is set by noise**: Spearman 0.99 between JPEG size and noise (n=183). [V]
   - 100.12 GB in 5,481 s.
   - Both cameras together are about 23 MB/s, about 83 GB/h. Budget at least 25 MB/s for the CSI at night.
5. **UGREEN turn blur is exposure, not focus.** [V]
   - At 20 fps the camera's AE exposure is between 33 and 50 ms: it drops the frame rate only when it needs more than
     1/30 s.
   - Streak measurements:
     - ≥ 32–38 ms (a lower bound);
     - 44 ms from the cross-camera comparison [A];
     - the analyst's 58.7 ms is longer than the frame, inflated by halos.
   - Smear is about 15 px at 10°/s and 37 px at 17°/s [A]. In 10/10 turns the CSI kept more detail (≤ 3 px) [A].
   - The UVC exposure/gain readback is stale under the camera's AE, so the real exposure is never logged.
6. **The UGREEN is light-starved on unlit roads, even at maximum exposure.** [V]
   - 44.9% of driving frames are below luma 35 (p5/50/95: 17.0/40.4/99.5).
   - The fps stayed 19.9–20.0 all drive [A]; the camera's AE never returned to 30 fps under street lighting.
7. **The CSI missed the last 13.3 min / 8.2 km of driving. The cause is operational.** [V]
   - I stopped the drive session (the CSI recorder) by hand at 23:39:35, a minute after "drive done", but the car
     drove on.
   - v0.4 recorded until 00:10:35.
8. **CSI glare discs** around street lights are 2.5–3× the UGREEN's angular size: 10.3/6.3° vs 3.6/2.6° for the same
   lamps. [V]
   - The cause (lens or windscreen veiling glare vs the brighter AE target) is not separated. [I]
9. **Smaller items:**
   - **UGREEN:** windscreen reflections of the in-car green and amber status LEDs (static, at 8 checked times). [A, V]
   - **CSI colour:** warmer than the UGREEN (R/G ×1.12, B/G ×0.91). [A]
   - **CSI hot pixels:** about 47 on the half-resolution grid at high gain. [V]
   - **CSI coverage:** no in-car reflections.
10. **imx296 debug flood:** 106,579 of the boot's 109,495 journal lines. [V]
    - Cause: `#define DEBUG 1` at line 19 of `~/drive_logs/tools/imx296_driver/build_src/imx296.c`.

**What worked** [A, V]
- **UGREEN:** 146,591 frames, 0 duplicates, 0 decode errors, seamless joins. One 196 ms gap at each start, and the
  planned 5.99 s restart gap.
- **CSI:** one pipeline part, 164,498 frames at 30.013 fps, 3 single-frame drops, 91/91 seamless boundaries.
- **System:**
  - CPU mean 12.5%, junction ≤ 54.3 °C;
  - 0 camera, USB, storage or thermal journal lines 22:17–23:38;
  - 0 v0.4 warnings while driving (the first is 23:54:44, the planned CANRawLog flash).
- **Timing:** video vs telemetry agree within about ±0.1 s. The `.ass` overlay values lag 0.10–0.20 s [A].
- **Telemetry, low priority:**
  - 5 C3→host stalls of 0.36–0.91 s lost 22 frames, matching the `host_tx_dropped` increments; root cause unknown.
  - The analysts report MKR `master_ms` drifting up to −1283 ppm against the host. That is implausible for a
    crystal-locked SAMD21 and **unverified**: the check was cut off. Use `host_ms` as the timebase.

**Proposals (not done; for the user to decide)**
1. **CSI mount:** re-mount it on a rigid bracket with no ball joint that can slip, level within 2°, car-fixed area
   ≤ 30%. Take dashboard reference frames before and after the next drive (`synth/scripts/mount_track.py`,
   `roll_track.py`).
2. **CSI device tree:**
   - raise `max_gain_val` (300–480) so AE uses analog gain before ISP digital gain;
   - raise `max_exp_time` toward about 1500 lines (about 22 ms; ≤ 2249 at 30 fps; more exposure means more blur);
   - review `default_framerate` 60 fps against the 30 fps recording.

   Then A/B on a night drive with `synth/scripts/csi_verify.py`.
3. **Driver:** drop `#define DEBUG 1` and rebuild with `build.sh`.
4. **Coverage:** keep the CSI recording as long as v0.4 runs, or stop the session only when the engine is off.
5. **UGREEN:** keep fixed 690. An exposure A/B (e.g. `exposure_dynamic_framerate=0`) would be darker; use the CSI
   for motion detail instead.
6. **In the car:** cover the status LEDs, add a dark dash mat, clean the CSI lens and its windscreen patch, consider a
   lens hood.
7. **Next drive's logging:**
   - Argus per-frame exposure and gain;
   - UGREEN frames stamped in CLOCK_MONOTONIC;
   - tegrastats at 1 s.
8. **Finish the cut-off check:** the mount over the whole drive, and the MKR clock (lower-envelope slope of
   `host_ms − master_ms`, against GPS time and the 2026-10-06 bench `can_sync.csv`). The script is at
   `~/.claude/projects/-home-jetson-Documents-github-repos-dashcam/9e99e416-de12-476d-85c1-eba06ce68c4d/workflows/scripts/night-drive-verify2-wf_527be524-0ac.js`.

## 2026-10-06 (Jetson): raw CAN over the MKR's USB — implemented, reviewed, bench-proven (committed `3a702a8`)

**Firmware** (`peripherals/mkr_zero`; workflows `wf_265592d8-b63` implement + `wf_1db52a87-ea3` adversarial review):
- **Capture:** the TC3 timer ISR drains the MCP2515 every 100 µs into a 1024 × 16 B SPSC ring (`lib/CANFrameRing.h`,
  `lib/CANStreamHw.*`, `canDrainIsr()`), stamping each frame with `micros()`.
- **Boot and modes:** boot into DISCOVER (listen-only, accept-all, no decode, never bus-active on its own). SNIFF only
  by host command, decoding from the ring through `canDecodeFrame()`.
- **USB stream:** `lib/CANRawStream.*` writes `F`/`FS` lines (contract in CANRawStream.h). It writes only when DTR is
  high and the bulk-IN bank is free (BK1RDY), and keeps the bank full with bounded waits (300 µs per packet, 1 ms per
  call).
- **Status line:** goes out in about 4 whole packets via `canStreamConsoleWrite()`, to avoid the core's `send()`
  TRCPT1 race, which the 10 kHz ISR made likely.
- **Telemetry:** in DISCOVER the CAN fields read unavailable (NaN, sentinels, validity bits clear, `canMode=1`). The
  `.ass` dashes them and shows `CAN discover`.
- **Self-test build** `build_and_upload.sh auto selftest[=N]`: numbered synthetic frames on id 0x7F0 for bench
  throughput. **Never flash it to the car.**
- **Tests:** 11 host suites (incl. can_ring/drain/telemetry/stream/selftest and sketch_policy_tests.py);
  `make mutations` catches **102/102**. Production build: 123,236 B flash, 23,924 B RAM (8,844 B free; stack + heap
  estimated ≤ 3.3 KB).

**Orin tools** (`peripherals/mkr_zero/tools/canstream`, 67 tests):
- `mkr_stream_log.py` writes `can_raw.log` (candump), `can_stats.csv`, `can_sync.csv` and `mkr_console.txt`. It uses
  a reader thread and HUPCL, maps the MKR clock with a windowed minimum, and does exact host-loss accounting.
- `canrawlog2candump.py` converts old CANRawLog logs.
- `can_decode` is the offline decoder, built from the unmodified firmware decode.
- `can_contract_emit` and the contract tests cover firmware formatter → logger → decoder.
- `drive_session.sh` now runs the new logger.
- **Real-data check:** the decoded 2026-10-03 baseline matches the user's actions.
- **Map findings:** hazards are not observable on this bus, and 0x294 is a steady stalk bit, not a blinking lamp.

**Bench results (2026-10-06, MKR + C3 on USB, no CAN bus):**

| Self-test rate | Result |
|---|---|
| 2400 fr/s (2× the Brio), 120 s | 288,103 frames, **0 sequence gaps**, ovf 0, ringdrop 0, host loss 0, ring peak 22/1024, timestamps monotonic; loop 5.4–7.4 k passes/s, max pass ≈ 17 ms; IMU/GNSS/C3 healthy |
| 4400 fr/s (a full 500 kbps bus of 8-byte frames), 60 s | **0 gaps**, ring peak 184/1024, host loss 0 |
| 10000 fr/s (physically impossible), 60 s | saturates at ≈ 5,850 fr/s; the excess is counted exactly as ringdrop; no reset; IMU/C3 fine; loop slows to ≈ 200 passes/s |

- **DTR/HUPCL:** with the port closed, frames counted as `nohost` and none as ringdrop.
- **Stalled host** (port open, nobody reading): only the ring's overflow is dropped, and counted.
- **Found and fixed on the bench:** every spell without a reader cost one IMU data gap. The new console writer still
  handed unread status-line text to the core, arming a packet nobody collects, so the next `send()` blocked for 70 ms.
  `canStreamConsoleWrite()` now **drops** text no host takes (break owed); tests and 3 mutants were added. Re-run:
  `gaps=` held at 1 through two closed-port spells and stalls of 4 s and 8 s. The old firmware had the same 70 ms
  behaviour.
- **The production build is on the MKR now**, checked on the bench: DISCOVER, FS once a second, no self-test traces.
- Logs: `~/drive_logs/bench_can_stream/`.

**Tools adversarial review (done):** 80 tests; 70 mutants (logger, converter, decoder), all caught.
- **Fixed:** a full or failing disk killed the logger for the rest of a session; it now keeps reading and resumes
  writing, losing whole lines only.
- **Fixed:** the stale first packet after a logger restart was used as a timing sample and as the start of the loss
  count.
- **Fixed:** a reader paused for more than 20 s split the timeline.
- **Fixed:** the wall-clock read was racy (clock-step false positives).
- **Format interop:** checked against python-can 4.6.1 and can-utils `log2asc`.
- **Open risk:** a disk that hangs without erroring grows the logger's queue by about 0.85 GB/h until it recovers.
- **Also open:** `drive_watch.sh` watches only `mkr_console.txt`'s age, not `can_raw.log` growth.

**Logger on real hardware (2026-10-06 01:28, self-test at 1200 fr/s):**
- **Restart against a running MKR:** one timeline, 0 malformed lines, 0 gaps.
- **DTR drop on logger exit:** the gap was counted as `nohost` (7,353), with 0 ringdrop.
- **Real MKR reset under the running logger** (production flashed mid-run):
  - port lost → reopened 4.9 s later → exactly one new timeline;
  - boot banner captured ("reset cause 0x40", "CAN: mode -> discover");
  - 53,730 frames before the reset with 0 gaps.

**Map facts from the baseline decode** (now in the `canmap.brio.txt` comments, `3a702a8`):
- 0x294 is a steady stalk-position bit, not a ~1.5 Hz blinking lamp, so SNIFF's 900 ms hold only delays "off".
- Hazards are not observable on this bus.

**Still to do:**
- The **in-car loss measurement** (parked, engine on, about 10 min) against the CANRawLog baseline (≥ 0.4 %).
- ~~Commit when the user asks.~~ Committed and pushed (`3a702a8`, `9b5c00e`).
- The drive crontab stays disarmed until the user re-arms it.

## 2026-10-04 00:25 (Jetson): raw CAN over the MKR's USB — decided, baseline captured, firmware NOT started yet

**What the user asked** (after the drive): "stream raw CAN frames to the orin nano without logging CAN related sidecar
info". Clarified by questions:
- transport = **the MKR's native USB**, direct to the Orin, so that link is no longer development-only;
- **all IDs**, accept-all;
- **no CAN decoding on the MKR**: decoding moves offline to the Orin, and telemetry's CAN fields read as unavailable.
- *Not* chosen: removing the CAN lines from the `.ass` (they will just show dashes), or removing CAN status lines.

**Design (fixed in the workflow script):**
- **Drain:** a hardware-timer ISR polls the MCP2515 (INT is not wired) into an SPSC RAM ring with a `micros()` stamp
  per frame. Every main-loop MCP2515 SPI access is masked.
- **Modes:** boot into `CanMode::DISCOVER` (listen-only, accept-all, raw stream, no decode); no probe and no
  automatic OBD2. SNIFF only by host command, decoding from the ring through a new public
  `canDecodeFrame(VehicleSignals&, YawEstimator&, uint32_t id, uint8_t dlc, const uint8_t*, uint32_t nowMs)`.
- **USB lines** (main loop, never blocking: `availableForWrite`, DTR gate):
  - `F tttttttt iii d hh..` (8-hex `micros()`, 3-hex standard / 8-hex extended id, `R d` for RTR);
  - once a second, `FS t drained streamed ovf ringdrop nohost`.
- **Unchanged:** the C3 and the wire protocol. DISCOVER stamps vehicle liveness, so the IMU low-power logic still
  sees a running car.
- **Orin tools:** in `peripherals/mkr_zero/tools/canstream/`:
  - `mkr_stream_log.py` writes candump-format `can_raw.log` (MKR clock → host epoch via a sliding-minimum offset),
    `can_stats.csv`, `can_sync.csv` and `mkr_console.txt`;
  - `can_decode` is a host C++ build of the unmodified firmware decode;
  - `drive_session.sh` switches to the new logger.

**Status:**
- Workflow `wf_e2433458-e09` (4 agents) was cut off when the session ended. Its first agent (firmware implementer)
  only researched (23:50–00:13): **no file in the repo was changed.**
- The script is saved at `~/drive_logs/tools/mkr_raw_can_stream_workflow.js`. Relaunch it with
  `Workflow({scriptPath})`: nothing is cached, so the agents start fresh.
- The agent's baseline helpers (flash/RAM and stack baseline of HEAD) are in
  `~/drive_logs/tools/session_scratch_20261003/` (`stack_*.txt`, `build_baseline.sh`, `test_baseline.sh`,
  `find_core.sh`).
- Before relaunching, add to its prompts: use the real baseline log below as the decoder's real-data input
  (read-only, outside the repo, never copied in), and the 0.4 % loss as the target.
- **Relaunched 2026-10-04 06:55 as run `wf_265592d8-b63`**, with those additions.
  - The script must sit under the working directory, so it is at `.git/claude_workflows/mkr_raw_can_stream.js`
    (inside `.git`, never committed).
  - Resume an interrupted run with
    `Workflow({scriptPath: ".git/claude_workflows/mkr_raw_can_stream.js", resumeFromRunId: "wf_265592d8-b63"})`:
    completed agents come back cached.

**Real all-IDs baseline, captured parked with the engine on (23:55):**
- The MKR was flashed with CANRawLog from a clean `git archive HEAD` copy, then production was flashed back at 00:00
  from the same copy: 121,684 bytes, identical to the 2026-09-27 production flash. The probe passed, sniffing
  resumed, and the C3 reported "master link up" at 00:00:10.
- **The MKR now runs production again.**
- `~/drive_logs/can_baseline_20261003/` (README there) holds `canrawlog_engine_on_20261003.log`: 333k frames,
  1229 fr/s, 39 IDs, ≥ 0.4 % loss. During the capture the user did, in order:
  - brake ×5;
  - left indicator, right indicator, hazards;
  - gear P-R-N-D and back;
  - throttle blips, and a hold at 2500 rpm;
  - steering full left and full right.
- Also in that dir: the restore script `flash_production.sh` and `journal_boot_final_0024.txt` (the full journal of
  the drive boot, taken at 00:24).

**v0.4 at the end of the night:**
- **Stop:** stopped with `systemctl stop` at 00:10:35, cleanly. The last segment `dashcam_000342` closed, and the
  service ends inactive but still enabled.
- **Session:** 138,752 frames from 22:14:49 = about 20 fps average, which is night mode's camera auto-exposure as
  documented.
- **Focus hand-back confirmed on the road (item X):** "focus: handed back to the camera … (autofocus on, stored lens
  position 650)". That is exactly the state it found at its 22:14 start, after the parked sweep.

**Housekeeping:**
- The drive-logging crontab line is **disarmed** (saved copy:
  `~/drive_logs/tools/drive_session/crontab_before_disarm_20261004.txt`). Re-arm with
  `(crontab -l; tail -2 ~/drive_logs/tools/drive_session/crontab_before_disarm_20261004.txt) | crontab -`.
- The uncommitted v0.4 telemetry work (11 files, including HANDOVER) has a patch backup:
  `~/drive_logs/tools/drive_session/telemetry_ass_csv_working_tree_20261004.patch`.
- The deployed config keeps `FocusMode=fixed` 690, `TelemetryCsv` on, and logs at 16 MB × 8.

## 2026-10-03 night drive: results (22:17–23:38, about 85 min, 83 km, max 106 km/h)

Everything recorded for the whole drive. The watchdog (`drive_watch.sh`, read-only, 30 s checks) never fired. The
session was stopped by hand at 23:39, and the CSI segment closed with EOS. The data is in:
- `/media/jetson/backup/drive_sessions/session_001/` (with `journal_final.txt` + `journal_final_monotonic.txt`, full
  snapshots taken at 23:39);
- `/media/jetson/backup/footage/dashcam_000301…`;
- `/media/jetson/backup/logs/{log,telemetry,bridge_status}_20261003_2{11445,21449}.*`.

The first file pair spans the clock step, so its wall-clock span reads 60 min; really it is about 6 min, at the same
~9.8 rows/s.

| Area | Result |
|---|---|
| v0.4 | 29 segments (**39**, corrected 2026-10-06), **0 warnings or errors during the drive** (the only WARN is the 22:14:44 port close at the planned restart); night mode the whole way |
| UGREEN focus | Held at 690 with autofocus off for 5031 s (`ugreen.csv`); auto exposure (night mode) throughout |
| CSI | **92 segments, one pipeline part (no restart)**, 100 GB, median 1.1 GB/min |
| Telemetry CSV | 50,187 rows over 85.4 min = 9.79/s; max gap 912 ms (2 gaps over 500 ms) |
| GNSS | Fix 100% of the drive, median 19 sats |
| Distance / speed | Integrated CAN speed 83.1 km; moving 90%; engine on 93% |
| C3 bridge | 0 CRC errors on the MKR hop; `host_tx_dropped` 4 → 66 (62 frames not sent to the Jetson in 85 min); telemetry age max 108 ms; C3 ≤ 35.7 °C |
| CAN (MKR console, 5078 engine-on seconds) | 401 fr/s received, `ovf` 62/s → **lower-bound loss 13.4%** (same as 2026-09-26); `turn=--` 7.4% of seconds; turn bits valid 85.7% and brake bits valid 93.0% of telemetry frames |
| IMU | `ioerr` 1/1 → 30/155; `hgrej` 1 → 155 (corrupt bursts, engine on); **High-G 0 → 64** (road shocks again, about 45/h); `i2cerr` 0 → 2; `i2cto` 0; `gaps` 1 → 3 |
| MKR | No reset, no USB drop, no quarantine for the whole session |
| Jetson | `tj` max 54.3 °C (mean 52.1 °C); disk 464 GB free after the drive |

**Paired snapshots (2026-10-04), 4 moments, both cameras:** in `~/drive_logs/night_drive_20261003_snapshots/`,
outside the repo (camera captures).
- **Method:**
  - dashcam: frames with the `.ass` burned in;
  - CSI: raw JPEGs decoded with `csi_decode.read_bgr_u8`;
  - moments located through the telemetry CSV; CSI segments mapped through `mono=` (the recorder started before
    the +3208 s clock step).
- **UGREEN at fixed 690:** sharp when steady. Heavy motion blur in turns and on passing objects, because night-mode
  auto-exposure runs a slow shutter (about 20 fps). The overlay and detail block render as designed.
- **CSI (c8_sh15):**
  - **On lit roads:** clean, sharp, natural colour, and much less motion blur than the UGREEN in the same turns
    (global shutter, exposure ≤ 15 ms).
  - **On unlit roads** (the 106 km/h highway frame): very noisy, with strong horizontal row banding and clipped
    headlight pools. This is the driver's exposure cap (1001 lines ≈ 15 ms) forcing gain to maximum; lifting the cap
    to the 33 ms a 30 fps frame allows would roughly halve the gain.
  - **Mount:** the dashboard fills the lower third to half of every CSI frame, and it is rolled about 5–10°. Re-aim it
    before the next drive. (2026-10-06: the mount moved at 22:20:13, about 13° → 4° roll, and possibly again later;
    see the night-drive analysis section.)
- **Yaw sign checked over the whole drive:**
  - it is opposite to the GNSS course rate in 92 % of turns over 5 °/s (correlation −0.56, as left-positive
    requires);
  - left indicator with |yaw| > 8 °/s: yaw positive in 213 of 219 samples; right indicator: negative in 192 of 197.
  - So the sign convention is right. The rare mismatches are real driving (signalling during an opposite curve).

Still to analyse: the High-G events against the footage, and CAN speed against GNSS speed. The cameras were
analysed on 2026-10-06 (see that section; it corrects several numbers here).

## 2026-10-03 21:10 (Jetson): rig prepared for the second drive, a night drive (code committed later in `7293abe`)

The K–Z work is committed and pushed (`2de0578`). Then, for the drive:

**v0.4: every telemetry field into the `.ass`, every bridge frame into CSV (uncommitted, 11 files)**
- **`.ass` detail block.**
  - `OverlayData::detailText`, drawn middle-left in a new `DET` style at 0.6× the corner font.
  - The application composes the text: `formatBridgeDetail()` / `applyBridgeDetail()` in `src/bridge_overlay.h`. The
    lines are:
    - powertrain + source;
    - brake / turn / hazard / steer torque / yaw;
    - the four wheels;
    - GNSS;
    - IMU raw;
    - IMU fusion + peaks;
    - High-G count/age, calibration, IMU flags;
    - the OBD-II PIDs;
    - CAN mode, map id and filter origin, switches, MKR uptime, flags.
  - Same rule as the corners: a field whose source is not live is a dash. A bit is printed only when its validity bit
    is set. A stale bridge gives `TLM -- no live bridge sample`.
  - librecord dashes the whole block (`TLM --`) once `detailTimestampMs` is older than `StaleTimeoutMs`. It replaces
    commas, braces and backslashes, which are ASS syntax.
  - Switch: `<Overlay><VehicleDetail>`, default true.
- **Telemetry CSV.** `src/telemetry_log.h` (`TelemetryCsvLog`) writes `telemetry_<start>.csv`, one row per bridge
  telemetry frame (about 10 Hz), and `bridge_status_<start>.csv` (1 Hz) to the log directory.
  - Columns: host epoch ms, every field, and the decoded bits (empty when not measured, never 0).
  - Threading: the RX thread only queues (bounded at 6000 frames, oldest dropped and counted). A writer thread
    flushes every 1 s and runs fdatasync every 5 s.
  - Switch: `<Log><TelemetryCsv>`, default false. **The deployed `dashcam.xml` has it on**, plus log rotation
    16 MB × 8; backup at `configs/dashcam.xml.bak-20261003-before-night-drive`.
- **Tests:**
  - `bridge_overlay_test` has 51 checks (detail lines; sentinels never leak; CSV row width, values and empty cells).
  - `record_test` is PASS: DET rendering, sanitising, staleness, and the header style; style count 5 → 6, with
    Part B on the UGREEN.
  - `config_test` 173 and `dashcam_v0_4 --self-test` PASS.
  - Plain `make` builds `bin/build_20261003_135326` (only the two old warnings). The service runs the newest build,
    so this is what boots in the car.
- Patch backup: `~/drive_logs/tools/drive_session/telemetry_ass_csv_working_tree_20261003.patch`.

**Drive logging outside v0.4: `~/drive_logs/tools/drive_session/` (README there), armed with a crontab `@reboot`
line (no sudo)**
- Each boot writes `/media/jetson/backup/drive_sessions/session_NNN/`:
  - the whole journal, followed live (the journal is volatile here);
  - IMX296 footage: nvarguscamerasrc with no ISP properties, so the installed c8_sh15 is what you see; 1456x1088 at
    30 fps, nvjpegenc, 60 s MKV segments, EOS on stop;
  - the MKR USB console;
  - UGREEN focus, exposure and gain every 1 s (corrected 2026-10-06: with autofocus on, `focus_absolute` is NOT the
    lens position, it keeps the last manual value; exposure and gain read stale under the camera's AE);
  - tegrastats;
  - a health snapshot every minute.
- Every line carries `mono=`, because the clock is wrong at boot in the car until v0.4 sets it from GPS.
- Disarm: `crontab -l | grep -v drive_session.sh | crontab -`.
- **Dry run at 21:01 (indoors), every output checked:**
  - the `.ass` block is live (IMU values, CAN sniff probing accept-all, GNSS present with no fix);
  - the CSV gives 9.7 rows/s;
  - CSI segments are finalised on stop;
  - `ugreen.csv` follows v0.4's exposure loop step for step;
  - stop takes 1 s;
  - a run under a bare cron environment works.

**Not done, by decision:** raw CAN frames. The MKR's single MCP2515 cannot stream all ~1100 fr/s and run production
at once, and the Orin's `can0` (Waveshare SN65HVD230) is not wired yet. The decoded parameters are logged (telemetry
CSV, `.ass`, MKR console).

**Found:** the IMX296's V4L2 `exposure` control maxes at 1001 lines, about 15 ms. Indoors it already read 1000/1001,
with gain 127/201. At night the CSI is gain-limited at about half the 33 ms a 30 fps stream allows. This is the
driver's range and the DT `max_exp_time`, not the ISP file. Look at it in the imx296 driver rework after the drive.

**In the car, 22:08–22:16 (parked, engine idling), before driving off:**
- **Startup:** everything started by itself (session_001). NTP set the clock within a second of v0.4 starting
  (+3208 s step). The detail block is live with car data: rpm 856, gear P, brake, steering torque, wheels, a 3D GNSS
  fix with 8 sats, map 0x0B filtered. The telemetry CSV gives 9.5 rows/s.
- **CAN loss:** the MKR console shows `ovf=` rising about 70/s against `rx=` about 420/s, roughly the 13% loss
  of 2026-09-26.
- **IMU:** corrupt bursts with the engine on are back (`hgrej=8`, `ioerr=1/8`).
- **Footage rates:** CSI night segments are about 12 MB/s (parked; the drive averaged 18.3 MB/s, see the 2026-10-06
  analysis) (3× indoors; noise inflates the JPEGs) and the UGREEN
  about 7 MB/s, so about 68 GB/h. With 577 GB free that is about 8 h before v0.4's 20 GB floor.
- **CSI at night:** bright and sharp, no halos. But the mount is rolled about 10° and the lower third of the frame is
  dashboard.
- **WRONG, corrected 2026-10-06 (see the night-drive analysis section): autofocus worked; `focus_absolute` is
  stale while it is on, and the judged frame was stale.** As first reported:
  **UGREEN autofocus fails at night.** After power-up it sat at the default `focus_absolute` 512 with
  `focus_automatic_continuous=1`, never moved, and the footage was badly blurred (street lights as large discs).
  After the sweep it also stayed at whatever value was written (1023, then 650) with autofocus back on.
- **Parked manual sweep**, via the standard control while recording: frames were cut from the growing MKV's tail,
  where frames are whole JPEGs, and scored by Laplacian variance.

  | `focus_absolute` | 200 | 450 | 575 | 625 | 650 | 675 | 725 | 800 | 950 | 1023 |
  |---|---|---|---|---|---|---|---|---|---|---|
  | Laplacian variance | 144 | 152 | 172 | 203 | 230–248 | **370** | 322 | 196 | 160 | 154 |

  Road focus is about 690, close to the 665–694 that autofocus chose indoors in daylight.
- **User chose fixed focus.** The deployed config now has `<FocusMode>fixed</FocusMode>` and
  `<FocusAbsolute>690</FocusAbsolute>`; backup at `configs/dashcam.xml.bak-20261003-2215-before-fixed-focus`.
- **v0.4 restarted without sudo:** `docker kill -s TERM dashcam_v04` makes v0.4 finalise and exit 0, and the
  unit's `Restart=always` brings it back. The footage gap was about 6 s (22:14:44.7 → 22:14:50.9). The log reads
  "focus: fixed at 690 … (autofocus off; range 0..1023)". The live frame scored Laplacian variance 486, the
  sharpest of the night; `ugreen.csv` reads 0/690.
- **The config change persists for later boots.** Revert to `camera` once a daylight check says autofocus is fine
  by day, or keep fixed for windscreen use.

**After the drive:**
- Disarm the crontab line.
- Pull `drive_sessions/session_NNN/`, `footage/` and `logs/` (`log_*`, `telemetry_*`, `bridge_status_*`).
- Judge the CSI night tuning from `csi/` (decode with `csi_decode.read_bgr`: limited range).
- Judge autofocus from `ugreen.csv` plus the footage.
- ~~Then decide whether to commit the telemetry work.~~ Committed (`7293abe`).

## Done 2026-10-03 (Jetson): items K–Z resolved

All sixteen items from "Next: items from the Jetson run and a second review" are fixed in the working tree.
- Four agents implemented them, one per area, on file-disjoint changes.
- An adversarial reviewer then checked each area: it read the diff, re-ran the tests, and reverted each fix in a
  private copy to confirm a test fails.
- A fixer applied every finding it could reproduce.

The full agent reports are in `~/drive_logs/tools/kz_agent_reports_20261003.json`. Committed after the boot check below.

**Build and host tests**
- Plain `make -j4` in l4t-ml-gpio builds `bin/build_20261003_124421`, rc 0. The only warnings are the two old
  `-Wunused-result` ones in `libcan.cpp` and `libgpio.cpp`, files nobody touched.

| Suite | Result |
|---|---|
| `camera_gst_test` | 178/178 (was 120); also clean under ASan and TSan builds |
| `focus_test` | 43/43 (was 22) |
| `usb_test --self-test` | new, PASS |
| `commlink_sim_test` | 88/88 (was 73) |
| `test_csi_driver.sh` | 123 checks (was 31); every reviewer mutant (38) is caught |
| `dashcam_v0_4 --self-test`, `config_test`, `bridge_overlay_test` | PASS |

**What changed**
- **CSI loader (K–P)**, `docker_dev/csi_driver.sh`, `csi-driver.service`, `test_csi_driver.sh`:
  - **K:** the default module is `imx296`, and both headers now give the real root cause.
  - **L:** the unit is ordered `After=nvargus-daemon`, not `Before=`. Under systemd (`$INVOCATION_ID`) Argus is
    started with `--no-block` and then polled. The unit gets `TimeoutStartSec=180`.
  - **M:** a new `reload` mode does the vendor unit's job: run every check first, wait `CSI_RELOAD_DELAY`, stop
    Argus, `modprobe -r` + `modprobe`, start Argus. The unit runs it, so it can replace `imx296-reload.service`.
    A failed unload is re-checked, and a still-enabled vendor unit is noted.
  - **N:** the judged file is `modinfo -k <kernel> -F filename imx296`. Files depmod does not know are detected,
    and "built-in" is handled.
  - **O:** a failed Argus restart exits 5.
  - **P:** `test_csi_driver.sh` is executable (mode 100755).
- **Camera_GST (Q–T)**, `libcamera_gst.{h,cpp}`:
  - **Q:** whenever the pipeline's EOS does not complete, the branch flush runs on a helper thread under the state
    timer. A sink stuck in `write()` (tested with a filesink on a FIFO nobody reads) no longer holds teardown past
    the bound.
  - **R:** after an early ERROR, the send gets until the deadline. A WARN appears only if the send is really
    stuck, and it gives the real waited time.
  - **S:** a `tearingDown_` flag, which `stop()`, `close()` and `start()` wait on. `close()` of a running camera
    is one teardown, and the false comment is fixed.
  - **T:** "auto controls first" is qualified in `camera_attributes.xml`, `libcamera.h` and `libcamera_gst`.
- **libcommlink (U–W)**, `libuart`, `libcommlink`:
  - **U, the close:** `Uart::close()` lets output that is still moving drain. It gives stalled output
    `closeDrainMs` (250 ms) plus the line time of the queued bytes, then runs `tcflush`. If bytes remain, it sets
    `closing_wait` to NONE (L4T's cdc-acm has no flush_buffer; needs CAP_SYS_ADMIN, which the privileged
    container has). The Jetson's DMA ttyTHS UARTs still send their last write.
  - **U, discovery:** the probe cursor resets when the candidate list changes, by-id matches come first, and
    `commlink_test --by-id` gives by-id-only discovery.
  - **V:** no log call is made under `m_statsMtx`.
  - **W:** `enumerate()` sorts the by-id names before resolving them.
- **Focus and usb_test (X–Z):**
  - **X:** a camera found with autofocus on gets its stored lens position back, written while autofocus is still
    off.
  - **Y:** a failed rollback sets `leftChanged()`. v0.4 then warns "could not hand the focus back" every time
    instead of "left to the camera". A later open retries the owed autofocus.
  - **Z:** usb_test prints the restore command at start. SIGINT, SIGTERM and SIGHUP stop it between tests and
    inside capture loops, and the controls are restored exactly once. A duplicate signal (from `timeout`, or an
    SSH drop) is absorbed, and SIGPIPE is ignored. The exit codes are 130, 143 and 129.
  - **v0.4:** only the focus log lines changed. The SIGINT/SIGTERM → finalise + exit 0 contract is untouched,
    and the rc=0 below confirms it.

**Run on the rig 2026-10-03**
- **Z, usb_test on the UGREEN:**
  - The full run is 8/8 PASS, and the `--list-ctrls` diff is empty.
  - Interrupted with `timeout --preserve-status` at 4, 5, 6 and 7 s, which lands during Tests 2, 3, 4 and 4b:
    every run restored the controls (empty diff) and exited 130 for SIGINT or 143 for SIGTERM.
- **X, v0.4 for 25 s with `FocusMode=fixed`, `FocusAbsolute=300`:**
  - This used a temporary config mounted at `/user/output/configs/dashcam.xml`, the path v0.4 reads; the deployed
    file was not touched. Footage went to a scratch dir.
  - Focus went from autofocus on, lens 694, to off at 300, then back on at 694 after SIGINT. The log reads
    "handed back to the camera … (autofocus on, stored lens position 694)", and rc=0.
- **U, the C3 without a replug:**
  - `commlink_test --by-id 20`: all [ OK ], rc 0.
  - `commlink_test "" 20`: the C3's by-id match comes first and is chosen directly. All [ OK ], prompt close,
    rc 0.
- **After the CSI ribbon was reseated (power off, boot 20:08):** the IMX296 probes again (`found IMX296LQ`, bound to
  tegra-capture-vi, `/dev/video0`).
  - **R:** `csi_test 0` 5/5 and `csi_rtp_test` PASS on this build, with no `EOS not delivered` or other WARN.
  - **M, O (stage A, user's sudo):** `sudo docker_dev/csi_driver.sh reload` gave rc 0, with "stopping
    nvargus-daemon.service to reload imx296", "reloaded …/5.15.199-tegra/kernel/drivers/media/i2c/imx296.ko",
    "nvargus-daemon.service started again", and the vendor-unit note. The sensor re-probed at 20:13:27, and
    `csi_test 0` gave 5/5 afterwards. Outside the unit the reload delay is 0; the unit sets 5.
  - **K, N refusal:** `CSI_MODULE=nv_imx296 … reload` gave rc 2 ("no nv_imx296 under /lib/modules … left as they
    were"), and Argus kept its MainPID (3720 before and after).
- **Stage B (user's sudo): `csi-driver.service` installed and enabled at 20:21, replacing `imx296-reload.service`,
  which is disabled.** The script is installed as `/usr/local/sbin/dashcam-csi-driver`.
  - **L:** `systemctl start csi-driver` took 5.6 s and `systemctl restart csi-driver` 5.5 s, with Argus active
    both times, where both used to hang with Argus down.
  - The journal shows "waiting 5 s before the reload", "stopping nvargus-daemon.service to reload imx296",
    "reloaded …imx296.ko" and "nvargus-daemon.service started again".
  - The unit ends with `Result=success`, and `csi_test 0` gives 5/5 afterwards.
- **Step 5, the first boot with `csi-driver.service` (20:25):** Argus started at 13.67 s (monotonic); the unit started
  right after it, waited 5 s, stopped Argus at 18.90 s, reloaded `imx296.ko`, and Argus was back at 19.27 s. The unit
  finished at 19.28 s, active (exited), `Result=success`, exit 0; Argus `NRestarts=0`. The sensor re-probed (`found
  IMX296LQ`, bound), `imx296-reload.service` is disabled, nothing is failed, and `csi_test 0` gave 5/5.
- **K, N:** `docker_dev/csi_driver.sh check`, with no `CSI_MODULE`, exits 0. It judges
  `kernel/drivers/media/i2c/imx296.ko`, vermagic 5.15.199-tegra, and notes that `imx296-reload.service` is still
  enabled.

**Not run yet**
- **Hardware fault, RESOLVED by reseating the ribbon (see above): the IMX296 did not probe at the first boot of 2026-10-03.** The kernel log shows `imx296 9-001a: 8-bit write to
  0x3000 failed: -121` and `probe of 9-001a failed with error -121`: no I2C acknowledge from the sensor.
  - There is no CSI device. The UGREEN took `/dev/video0` and `/dev/video1`.
  - So `csi_test` gets "No cameras available" from Argus, and the 2026-09-28 build's `csi_test` fails the same
    way: this is not the code.
  - Power the rig off, reseat the CSI ribbon at both ends, and boot. The loader can't help: the module loads,
    the sensor doesn't answer.
- ~~R on Argus~~: done, see above.
- **The CSI loader on real systemd (needs the user's sudo; the sensor must probe first):**
  Steps 1–5 are done (above). Only the optional step 6 is left.
  1. `sudo docker_dev/csi_driver.sh reload; echo rc=$?`
     - Pass: rc 0, with "stopping nvargus-daemon.service to reload imx296", "reloaded …imx296.ko" and
       "nvargus-daemon.service started again" in the output.
     - Then `csi_test 0` 5/5.
  2. `A=$(systemctl show -p MainPID --value nvargus-daemon); sudo CSI_MODULE=nv_imx296 docker_dev/csi_driver.sh reload; echo rc=$?`
     - Pass: rc 2, the same Argus MainPID, imx296 still loaded.
  3. Install, replacing the vendor unit:
     ```bash
     sudo systemctl disable imx296-reload.service
     sudo install -m 755 docker_dev/csi_driver.sh /usr/local/sbin/dashcam-csi-driver
     sudo cp docker_dev/csi-driver.service /etc/systemd/system/
     sudo systemctl daemon-reload
     sudo systemctl enable csi-driver.service
     ```
     Rollback: `sudo systemctl disable csi-driver.service; sudo systemctl enable imx296-reload.service`.
  4. Item L: run `time sudo systemctl start csi-driver.service`, then `time sudo systemctl restart csi-driver.service`.
     - Pass: each returns in about 6–15 s, both units end active, and the journal shows the four reload lines.
     - Before the fix, both hung with Argus down.
  5. Reboot.
     - Pass: `csi-driver` is active (exited) and started after Argus (`systemd-analyze critical-chain csi-driver.service`),
       and `csi_test 0` gives 5/5.
  6. Optional, item O.
     - On this rig `systemctl mask --runtime nvargus-daemon` is shadowed by `/etc/systemd/system`. Use a runtime
       drop-in instead: `/run/systemd/system/nvargus-daemon.service.d/zz-fail.conf` with `[Service]` and
       `ExecStartPre=/bin/false`, then `daemon-reload`.
     - `sudo docker_dev/csi_driver.sh reload` must then exit 5, with "Argus is down".
     - Remove the drop-in, `daemon-reload`, and start Argus.
- **U with a replug (needs sudo to de-authorise the C3: `echo 0`, 6 s, `echo 1` to `/sys/bus/usb/devices/1-1/authorized`):**
  - `commlink_test --by-id 50`: expect a hangup, a reopen about 1 s after the replug, `reconnects=1`, all [ OK ].
  - `commlink_test "" 50`: expect one "did not identify itself" line for the MKR console, a close of about 360 ms
    (it was 30.9 s), the C3 first after the replug, and a prompt `stop()`.
- **Y on hardware:** it needs two control writes to fail with the camera attached, so it is covered only by
  `focus_test`.
- **The deployed v0.4 already runs this build.** The launcher starts the newest `bin/build_*/dashcam_v0_4`, and the
  20:25 boot started `bin/build_20261003_124421` (journal: "starting bin/build_20261003_124421/dashcam_v0_4").

## 2026-09-28 → 10-03: items K–Z from the Jetson run and a second review (all fixed 2026-10-03, see the section above)

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

## 2026-10-01 evening → 10-02 (Jetson): CSI/USB chart tuning, c8_sh15 installed

**Current state, not a deployment:** the user ran the privileged c7 A/B trial successfully. It restored
`/var/nvidia/nvcam/settings/camera_overrides.isp` to **c5_rpi100T.isp** afterwards. The restore was verified by
hash; `nvargus-daemon` is active and `dashcam-v04` remains **stopped**. The later c7 install command was offered
but has **not** been run. No production application configuration or MCU firmware was changed by this tuning.

| Profile | SHA-256 |
|---|---|
| Installed c5 | `20b4c94c5fa4663aaef762192d0ecbd9b5d68066a32fc5e78d2950ab36456d61` |
| Tested c7 candidate | `bd5c2cacf7696d71ddd63896dddf94ac09dbed619fa8278ba59d322af799d9c0` |

The recovery copy from this trial is `/var/backups/csi-trial.c4MsRq/camera_overrides.isp`.
The candidate is `/home/jetson/drive_logs/tools/isp_tuning/session_20261001_2146/c7_usb_half.isp`.

**Platform catch-up:** the existing log `/home/jetson/drive_logs/platform_checks/20261001-161803/summary.txt`
records the completed R36.5.2 upgrade: 24 checks passed, none failed, one skipped. It includes the loaded
5.15.199 IMX296 module, `csi_test` 5/5, CSI RTP, Camera_GST, discovery of both cameras, CUDA, and TensorRT
(mean GPU compute 14.008 ms). `usb_test` was skipped because v0.4 owned the UGREEN at that time. This is a
reviewed earlier log, not a fresh rerun of the platform suite. The evening chart sessions independently
confirm live CSI and USB capture after the upgrade. Earlier rollback/pre-upgrade sections below are historical.

### Measured A/B result

Both cameras viewed the same printed chart under room lighting. Captures used **host GStreamer/Argus**;
offline OpenCV analysis ran in an isolated `l4t-ml-gpio` container with no camera access. These tuning captures
were **not** taken inside the production container. The USB camera was a relative reference, not a calibrated
colour standard; ordinary printed patches and USB processing cannot establish absolute colour accuracy.

Evidence directories (outside this repo, on the Jetson):

- `/home/jetson/drive_logs/tools/isp_tuning/trial_20261001_215416_base/`
- `/home/jetson/drive_logs/tools/isp_tuning/trial_20261001_215416_candidate/`

Each contains `session.json`, `metrics.json`, JPEG pairs and the installed-profile snapshot. Both sessions
completed with 50 pairs each and the expected distinct profile hashes. Analysis accepted **97/100 pairs**
with all four chart markers detected in both images. Excluded: base `ee_default_1`, candidate `ee_default_4`
and `base_end_1`. All five pairs at the main comparison setting were valid in each session.

At **`ee-mode=0`, `tnr-mode=1`, `saturation=1.0`**, means over five pairs:

| Measurement | c5 baseline | c7 candidate |
|---|---:|---:|
| Colour difference to USB, mean ΔE76 over 18 colour patches (lower is better) | 8.56 | 4.79 |
| Grey difference to USB, mean ΔE76 over patches 19–22 | 5.36 | 4.03 |
| CSI paper lightness L* | 71.15 | 70.23 |
| USB paper lightness L* | 72.41 | 72.56 |

**Correction (late evening review, below):** the CSI JPEGs hold limited-range BT.601 YCbCr with no JFIF marker,
but every script decoded them as full range. With the correct decode, colour ΔE to USB is **11.71 (c5) vs 6.69
(c7)**, and CSI paper L* is 74.9 vs USB 72.6. The ranking stands, but the saturation conclusion in the next
paragraph does not: see "Review of the c7 work".

The ~44% reduction in colour-matching error persisted at the end of the sweeps (8.60 versus 4.75).
USB controls were not written; before/after dumps matched. USB automatic processing remained enabled,
but measured reference drift between sessions was small: mean patch ΔE 0.35, maximum 0.76. Frames were
paired, not hardware-synchronised. c7 is about 0.9 L* darker at the same sensor exposure and retains a grey
tint (mean a*=+3.88, b*=-9.12); the USB reference itself renders blue greys.

**Provisional c7 runtime choice (withdrawn, see the review below):** `ee-mode=0 tnr-mode=1 saturation=1.0`.
Saturation 0.9 and 1.1 worsened c7's colour score to 5.59 and 5.68 respectively; the earlier 0.9 recommendation
applied to c5 only. *With the correct decode, c7 scores 5.24 / 6.71 / 9.27 at 0.9 / 1.0 / 1.1, so 0.9 is
better, and an explicit 1.0 is about 1.3× the chroma the app gets today with saturation unset.*
Default sharpening produced strong low-contrast bright halos (~63% versus ~5% with enhancement off,
representative frames). Spatial luma standard deviation was 2.39 versus 0.99; this includes print texture
and JPEG artefacts, so it is not a sensor-noise measurement. These are indoor observations, not proof of
better lane detection or all-light image quality. Earlier workspace experiments report green output with
`tnr-mode=2`; that mode was not revalidated in this c7 A/B and should not be selected on this evidence.

### Tools, pitfalls and next steps

Workspace: `/home/jetson/drive_logs/tools/isp_tuning/`. `README.md` has detailed results at the top;
its older c1/c2 and "Next: the printed chart" notes are historical, not the current state.

- `tune_session.py` uses continuous CSI/USB sessions, fresh output directories, bounded frame waits,
  settling, five pairs per setting, profile/control metadata and pipeline cleanup. It refuses an active recorder.
- `analyse_session.py` measures valid four-marker pairs; `--no-fit` analyses an A/B without deriving another
  candidate. c7 came from a half-strength, brightness-normalised USB colour fit to the stable
  `session_20261001_2146` captures. The earlier `session_20261001_2142` lock-only run drifted in brightness
  and was excluded from the final fit. Offline cross-validation numbers are not the live A/B scores above.
- `trial_c7.sh` checks the baseline/candidate hashes, backs up the installed profile, captures both profiles,
  and restores the original on normal exit/error/interrupt. It never starts the recorder. Power loss or
  SIGKILL can bypass the restoration trap; retain the recovery copy.
- The older `pair.sh`/`cycle.sh` were **not repaired or used**: capture failures can permit stale shot reuse,
  and failed installs/restarts or interruption can leave an experimental profile selected.
- Fixed bench settings are vendor-driver-specific: Argus `exposuretimerange="675000 675000"` and
  `gainrange="11.5 11.5"`, digital gain 1, read back as exposure **675 lines (~10 ms)** and gain **184
  (18.4 dB)** throughout the trial. Do not interpret these as 675 microseconds or 11.5× gain, or copy the
  manual settings/AE-AWB locks into the road application.
- `c7_runtime.json` only records the suggested runtime preset; it is **not loaded by the application**.
  Installing an ISP file alone does not disable runtime edge enhancement.

Next: select c7 only if the user chooses to proceed, then repeat paired comparisons in daylight and with
automatic exposure/white balance, check moving scenes for blur/temporal artefacts, and verify the actual
application's runtime properties before making a production default. Keep c5 and the backup available for
rollback. No additional profile installation, service start, fixes or firmware changes were made while
recording this handover.

### Review of the c7 work, and what the lead tuning session found (2026-10-01, 23:45)

This is the session that ran the upgrade and the c1–c6 rounds (`cycle.sh`). At the end it ran five agents:
- one researched the ISP keys;
- one measured edges and noise;
- three reviewed: the sharpening result, the c7 trial, and the decode/saturation findings.

Their full outputs are in `isp_tuning/reviews_20261001/`. Nothing was installed after 20:20 except by
`trial_c7.sh`. The repo's code and config are unchanged, and no ISP file, service or camera setting was touched.

**Platform: a lesson from the upgrade.** The first `--apply` stopped at its final hold check:
`apt-get install --allow-change-held-packages` had **cleared all 65 holds**. Both `l4t_upgrade_36_5_2.sh` and
`l4t_rollback_36_5_0.sh` now run `apt-mark hold` on the 65 after apt. The re-run passed, and after the reboot
`verify_platform.sh` gave 24 pass, 0 fail, 1 skip. A backgrounded `sudo` that is waiting for a password shows
state `T` and looks hung: run `sudo -v` first.

**What is installed, and why.** `camera_overrides.isp` = `c5_rpi100T.isp` (sha256 `20b4c94c…`), installed since
20:20. *Superseded 2026-10-02 06:53 by `c8_sh15.isp`, which is c5 plus the sharpness table (see below).* It differs from the vendor file in two ways:
- **Black level 50, not 60.** The driver programs BLKLEVEL 0x032 for colour; the vendor ISP subtracted 60,
  which crushed the shadows green. (c1)
- **The colour matrix, untransposed.** `colorCorrection.srgbMatrix[i]` is the i-th **column** of the operator:
  out = fileᵀ · in. Two things prove it:
  - a probe file, rows [[1,0,0],[0,1,0],[0.5,0,0.5]], turned the greys orange-yellow (b* +13.7), as the
    column reading predicts;
  - the c7 trial matches its fitted matrix to 0.6 ΔE under this convention, against 27.8 ΔE under the other.

  The InnoMaker file (v1.1) stores I + 0.5·(Raspberry Pi IMX296 5600 K CCM − I) row by row, i.e. transposed for
  NVIDIA. That is the magenta cast. c5 is the RPi CCM at full strength, written column by column. It preserves
  neutrals (column sums 1).
- The vendor file is not an IMX296 tuning at all: it shares 403 of its 404 keys with NVIDIA's own IMX477 v.03
  tuning. The NLM noise reduction (v8) and `sharpness.v5.tab` were deleted, and the CUDA DCT denoiser (v6),
  which NVIDIA disables on Orin, was switched on.

**Sharpening.**
- `ee-mode=0` is the only setting measured that removes the halos. Same-session values, ee-mode=0 vs the ISP
  default:
  - bright halo on low-contrast edges: 5 % vs 62 %;
  - luma noise in the grey patches: ×0.42–0.49;
  - pixel-level grain: about ×1/3.
- `ee-strength` 0.05 → 0.20 sits in between: rim and MTF are linear in strength, and 0 equals ee-mode=0.
- A dark rim survives even ee-mode=0: 14–18 % on low-contrast edges, 22–25 % on high-contrast ones.
- **`sharpness.v2` values are set indices: LOWER = STRONGER.** 0 is the strongest sharpening, and MaxValue is the
  weakest (that end is inferred, not yet captured). The evidence is libnvscf's disassembly, NVIDIA's embedded
  tunings and the measurements.
  - So `c6_sh1` and `c6_sh0` sharpened **more** than the vendor table. The rim went 84 → 118 → 135 %, z +5.9 and
    +9.0 against the measured spread.
  - Their header comments now say so. Don't use them.
- **File-only candidates, untested:** `c8_sh15.isp` (the auto table all 15) and `c8_v5off.isp`
  (`sharpness.v5.enable = FALSE`, NVIDIA staff's advice for Xavier). Updated 2026-10-02, see below; `c8_v5tab15`
  was dropped.
  - Aim: give every app ee-mode=0-like output without setting properties.
  - Pass, in a fixed-exposure A/B with the ISP default EE: low-contrast rim ≤ 20 %, halo ≤ 8 %, MTF(0.25) ≤ 1.15,
    noise within 10 % of ee-mode=0, colour unchanged (±0.5 ΔE), clean override parse.

**Analysis bug: the CSI stills.**
- `nvjpegenc` writes limited-range BT.601 YCbCr (Y 16–235) into a JPEG with no JFIF marker. The UGREEN's are
  JFIF full range. cv2 decodes both as full range, which lifts the CSI blacks and shrinks its chroma by 0.878.
- Every CSI number before 22:55 is off by this, including the README's "EV +0.5 matches brightness": corrected,
  EV +0.5 overshoots, at paper L* 95 vs 73.
- The decode recipe is in `reviews_20261001/workflow_run2_verify.json` (`decode`). `chartcmp.py`, `chartnoise.py`,
  `chartsharp.py` and `analyse_session.py` still use the wrong decode.
- **Production:** none for v0.4, which never opens the CSI camera.
  - v0.3's inference path (nvvidconv → BGRx) is correct.
  - v0.3's RTP H.264 carries BT.601 samples but a VUI saying bt709. nvarguscamerasrc caps carry no colorimetry,
    and GStreamer assumes bt709 above 576 lines.
  - One-line fix, verified by emulation and **not applied**: `video/x-raw,format=(string)I420,colorimetry=(string)bt601`
    after nvvidconv in `libnetwork_rtp.cpp`.

**Saturation and the other properties.**
- nvarguscamerasrc applies `saturation`, `ee-mode`, `ee-strength` and `tnr-*` **only when the property is set**
  (`saturationPropSet` etc.).
- `setColorSaturation` replaces the ISP's value; it does not multiply it (Argus `Settings.h`).
- Unset, the effective saturation measures **about 0.75** of an explicit 1.0. The plausible mechanism is the
  file's `defaults.saturation` 0.85 × `ae.saturation` 85; it has no same-session A/B yet.
- The app sets **no** CSI properties today. The csi0 entry in `config/dashcam.xml` lists V4L2 metadata only, and
  `libcamera_csi.cpp` builds a bare `nvarguscamerasrc`. So v0.3 would render the ISP default edge enhancement and
  about 0.75 saturation.
- Bench settings reach the app only as csi0 `<Capabilities>`, or baked into the ISP file.

**c7: closer to the UGREEN, not shown to be better.**
- **In-sample.** The fitted matrix predicts the trial to 0.1–0.2 ΔE, so the trial confirms the fit, not that it
  generalises. The fit and the trial share the chart, the lamp, the fixed exposure and gain, an explicit
  saturation of 1.0 and the target.
- **Out of sample.** On 11 earlier auto-exposure captures, c7 is still closer to the UGREEN: 12.1 → 10.1, and
  every capture improves.
- **Against the chart's nominal values: about zero.** At the default saturation, 22.48 → 22.20 (exposure-
  normalised) and 17.60 → 17.75 (white-balanced). Hue error is unchanged, 6.83 → 6.75. c7 rotates hues toward
  the UGREEN, which is further from nominal (|dH| 8.1) than c5 is (6.9).
- **The tint.** c7's columns sum to 0.891 / 0.958 / 0.950. That is a lamp-specific red cut (a* −1.6 on a neutral
  that c5 renders neutral; −2.6 on white) plus 4 % darkening. On its own the tint is slightly harmful against
  nominal.
- **Grey cast.** Only a* improves; the blue b* ≈ −9 is untouched, because the UGREEN shares it.
- **Option: `c7n`,** c7's neutral-preserving part on c5 (rows in the review). Untested.
- **Settling it needs:** a reference with known values (a real ColorChecker, or the print measured with a
  spectrophotometer), and a daylight + lamp A/B with auto AE/AWB, no locks, and saturation unset, 0.9 and 1.0.

**`trial_c7.sh`: safe for the file, weak in details.** A simulation drove Ctrl-C, SIGTERM, SIGHUP, a hung child,
a failed child and a failed restart. Every ordinary path ended with c5 installed, as it did for real at 21:54.
Weak points:
- Ctrl-C waits for the running capture, up to 120 s: `timeout` moves the child into its own process group.
- The candidate hash is checked at the start but the file is installed 35 s later, from a jetson-writable path.
- A second Ctrl-C can interrupt the restore.
- "RESTORE FAILED" is printed when only the daemon restart failed.
- The recorder check sees only the systemd unit.
- There is no HUP trap.

**`pair.sh` / `cycle.sh`: the other session's critique is fair.**
- `cycle.sh` has no trap, so an interrupt can leave a test profile installed.
- `pair.sh` can leave a stale image when a capture fails.
- One frame per candidate leaves no measure of run-to-run spread.
- The `[exposure: gain:]` readout is the **CSI's** (`/dev/video0`), not the UGREEN's, and it goes only to stdout.
- Use `tune_session.py`-style sessions for A/B work.
- Separately, the earlier lock-only session's frame-to-frame swing is probably 100 Hz lamp flicker at 14.8 ms.
  675 lines ≈ 10 ms is exactly one flicker period.

**Next, in order (2026-10-02: steps 1 and 2 done; c8_sh15 installed and verified; see below):**
1. ~~Switch the analysis scripts to the correct CSI decode.~~ Done.
2. ~~A/B the c8 sharpening files.~~ Done: c8_sh15 passed, and it has been installed since 2026-10-02 06:53.
3. Pick the CSI properties (ee-mode=0 or c8; saturation about 0.9–1.0) and put them where the app reads them.
4. Daylight + auto AE/AWB comparison of c5 / c7n before any production default.
5. Fix the grey cast at its source, the AWB tables inherited from IMX477, not in the colour matrix.

Files (`/home/jetson/drive_logs/tools/isp_tuning/`):
- `chartcmp.py`, `chartnoise.py`, `chartsharp.py`, `c5gen.py`, `convtest.py`;
- `chart/` (the printed chart and its geometry);
- `c1_black50` … `c8_*` candidates;
- `reviews_20261001/`.

### 2026-10-02 early: the decode fixed, and an A/B kit for the c8 files (not run yet)

Nothing was installed or restarted, and no camera was opened; the installed profile is still c5 (`20b4c94c…`).
Three more agents reviewed this tooling adversarially, and their findings are fixed.

**Decode.**
- `csi_decode.py` (`is_nvjpeg`, `read_bgr`) recognises an nvjpegenc still from its header and expands the limited
  range: no APPn marker, SOF component ids 0,1,2. It also caught 20 CSI files in `noise_study/` whose names lack
  `.csi`.
- `chartcmp`, `chartnoise`, `chartsharp`, `fitccm`, `match`, `regions` and `analyse_session` all use it now.
- Checked:
  - on all 485 JPEGs it matches an independent decode to 6e-5;
  - JFIF (UGREEN) results are bit-identical to before;
  - corrupt files return None, as cv2 did;
  - `chartsharp` measures the JPEG's own (expanded) Y and keeps its clip column.
- Paper spot 5, which sat inside sharpening halos, is gone.
- `analyse_session.py` refuses to overwrite `metrics.json` / `c7_usb_half.isp` without `--force`.
- Re-score: c5 11.70 vs c7 6.65 to the UGREEN. Pre-fix copies are in `reviews_20261001/pre_decode_fix/`.

**What NVIDIA's parser does with the c8 files.** A reviewer ran the installed libnvscf parser in-process on each
file; no camera was opened.
- `sharpness.v2.{Preview,Still,Video}[r]` and `sharpness.v5.tab.*` are **one table**, and the last line wins.
- `sharpness.v2.MaxValue` is parsed and **discarded**. Writing it as `15.0` would make the whole override fail
  to load.
- An unknown key skips only its line, and the loader logs it. A bad value or syntax error makes the whole load
  fail: there is no silent fallback to vendor defaults.
- The vendor table {3,3,5,…} replaces the stock ISP6 table {3,4,5,7,8,10,11}, so **the vendor file sharpens more
  than NVIDIA's own default.**
- The files now:
  - `c8_sh15.isp` (sha256 `43ec0c81…`): the table all 15, no MaxValue line. At the ISP default it should select
    exactly what ee-mode=0 selects (sharpen set 15, gain 0), and leave manual ee-strength and colour unchanged.
  - `c8_v5off.isp` (`4d40517a…`): also kills manual ee-strength sharpening. Medium confidence.
  - `c8_v5tab15` parsed identically to c8_sh15, so it moved to `reviews_20261001/superseded/`.

**The kit.**
- **`trial_ab.sh [--auto] <candidate>…`** (sudo) runs base, cand1, base, cand2, base with
  `tune_session.py --manual --with-unset`. It restores the entry profile on every exit. Over `trial_c7.sh` it adds:
  - signals act within about a second (Ctrl-C, Ctrl-\, TERM, a closed terminal);
  - installs only from root-owned copies, by rename;
  - a signal that lands as the restore starts cannot cut it short;
  - file restore and daemon restart are reported apart;
  - refuses if anything but nvargus holds a camera;
  - saves each step's nvargus journal and warns on loader lines the base did not log.

  It passed the reviewer's simulation: real pty signals, hangs and failed restarts, with the settings file
  polled every 0.5 ms.
- **`ab_report.py <trial dir> --json`** scores each candidate against its neighbouring base sessions:
  - checks: the sharpening criteria above, colour unchanged, and a clean override parse from the saved journals;
  - informational: manual ee-strength 0.2, which tells c8_v5off from c8_sh15.
- **`tune_session.py`** has two new opt-in flags, and its defaults are unchanged:
  - `--with-unset`: frames captured before any saturation property is set;
  - `--auto`: no fixed exposure and no AE/AWB lock, for daylight.

**To run (about 5 min; chart in view, recorder stopped):**
```bash
sudo bash ~/drive_logs/tools/isp_tuning/trial_ab.sh c8_sh15 c8_v5off
docker run --rm --user 1000:1000 -v ~/drive_logs/tools/isp_tuning:/w -w /w l4t-ml-gpio:latest python3 ab_report.py trial_<stamp> --json
```
Daylight, later: `sudo bash …/trial_ab.sh --auto <candidates>` with the rig at a window.

### 2026-10-02 05:45: the c8 A/B result. Both pass; `c8_sh15` is the one to use

The user ran `trial_ab.sh c8_sh15 c8_v5off` (output in `isp_tuning/trial_20261002_054515/`, scored in `report.json`).
- All 5 sessions completed with the expected profile hashes.
- No loader lines appeared in any nvargus journal.
- The entry profile c5 was restored at the end of the trial.

ISP default edge enhancement (the path an app that sets no ee property gets), 5 frames each:

| | c5 neighbours | **c8_sh15** | c8_v5off | ee-mode=0 (same sessions) |
|---|---|---|---|---|
| low-contrast dark rim | 77–92 % | **14.5 %** | 13.3 % | 13–16 % |
| bright halo | 62–73 % | **7.2 %** | 7.5 % | 7–8 % |
| MTF at 0.25 cy/px | 3.2 | **1.01** | 1.03 | 1.03–1.06 |
| luma noise vs ee-off | ≈ 2.3× | **1.04×** | 0.94× | 1× |
| manual ee-strength 0.2, halo | 22–25 % | **23 %** (kept) | 7.8 % (gone) | — |
| colour ΔE vs neighbours | — | −0.25 | +0.09 | — |

- Both behave as the parser review predicted.
- `c8_sh15` is the better default:
  - every app gets ee-mode=0-like output with no property set, which includes the dashcam app, since it sets none;
  - `ee-strength` still works for anyone who wants mild sharpening.
- `c8_v5off` disables the sharpen block outright.
- Crops (`isp_tuning/reviews_20261001/c8_crops.png`): the c5 default draws pen-like outlines and grain;
  `c8_sh15` at the default looks like ee-mode=0.
- Side note: with saturation unset, the CSI is closer to the UGREEN than with an explicit 1.0 (ΔE 9.9 vs 12.7).

**Marker detection, changed for this run.**
- In the softer frames the CSI missed marker 0, which is only about 19 px wide (cells about 3 px). That dropped
  half the frames, including all of c8_sh15's key frames.
- A three-marker homography is not good enough: patch positions moved 3.5 px median, 6.6 px max.
- `chartcmp.find_markers` now retries the markers it misses on a zero-phase unsharp-masked copy.
  - Frames that found all four markers before are byte-identical, and the c5/c7 re-score is unchanged.
  - The retry recovered 117 of 119 frames.
  - Its corners differ from plain detection by median 0.31 / p95 1.24 px. That equals the detector's own spread
    between two upscale factors on the same image (median 0.25 / p95 1.24).

**c8_sh15 is the default since 2026-10-02 06:53.** The user installed it:
```bash
sudo install -m 0644 ~/drive_logs/tools/isp_tuning/c8_sh15.isp /var/nvidia/nvcam/settings/camera_overrides.isp && sudo systemctl restart nvargus-daemon
```
Back to c5: the same command with `c5_rpi100T.isp`.

**Verified at 06:56.**
- The installed file's sha256 is `43ec0c81…` (= c8_sh15), and nvargus-daemon restarted in the same second.
- Two later camera sessions loaded the override with no config-loader lines in the journal.
- A capture with **no properties at all** (what the dashcam app does) measures like an ee-mode=0 capture taken a
  minute later. One frame each:

| | rise (px) | dark rim | bright halo | MTF at 0.25 | JPEG |
|---|---|---|---|---|---|
| no properties | 1.09 | 19.5 % | 8.5 % | 1.07 | 263 KB |
| ee-mode=0 | 1.07 | 21.2 % | 5.6 % | 1.10 | 266 KB |
| c5, no properties (2026-10-01) | 0.38 | 83 % | 61 % | 3.29 | 469 KB |

## Done 2026-10-01 (Jetson coder): the IMX296 driver rebuilt from source, and tools to move between R36.5.0 and R36.5.2

**Historical pre-upgrade notes:** the upgrade and hardware checks have since completed; see the evening
update above. Statements below saying "not yet run" or "not applied" describe the state when these notes
were originally written, not the current rig.

**Why.** On R36.5.0 everything works, but the platform is held one release back, and the QSPI bootloader is already
36.5.2. UEFI refused the 36.5.0 capsule: `fwupdmgr` reports `Current version 2360578`, `Minimum Version 2360578`,
which is 36.5.2. So the question was whether the IMX296 can run on R36.5.2 (kernel 5.15.199). Very likely: the driver
now rebuilds for any kernel from the vendor's own source, and the 5.15.199 build matches all 41 of 36.5.2's symbol
CRCs. Loading it on a 5.15.199 kernel is still untested. **Nothing below has been applied yet.** The machine is still on
R36.5.0 as of 2026-09-30, and the upgrade is the user's call.

**The driver source.**
- InnoMaker (`github.com/INNO-MAKER/cam-imx296raw-trigger`, cloned at `~/Downloads/IMX296_driver/cam-imx296raw-trigger`)
  publishes only prebuilt modules. At HEAD (`69a6afc`, 2026-07-05), the 5.15.185 folder holds only a file named
  `contact sales@inno-maker.com for the driver.txt`. There is nothing for 5.15.199.
- Their source package `imx296_source_working_20260615.tar.gz` was added in `523d689` (2026-06-16 09:37 +0800) and
  deleted in `92143cf` four hours later. It is recoverable from the clone's history (`git show 523d689:…`).
- It lacks `imx296_mode_tbls.h`. That header comes from FRC team 971's open IMX296 tegracam driver
  (`github.com/frc971/jetson-orin-kernel-builder`, `patches/`), which the vendor's code derives from (per the
  research; the vendor pointed to it in their issue #1).
- One small patch, `v1.1.patch` (two hunks), turns the 06-15 source into the vendor's v1.1 binary.
  - One hunk sets the mono black level to `0x03c` (it was `0x070`).
  - The other changes the probe line to `found IMX%uL%c` (`L` for mono, `Q` for colour, as before).
  - Neither touches the colour path.
- A second patch, `build-fix.patch`, drops an include of `camera_gpio.h`, which nothing uses.

**Verified on the Jetson.**

| Check | Result |
|---|---|
| Built for 5.15.185, against the vendor's working v1.1 module | `.text`, `.rodata`, `.rodata.str1.8`, `.data`, `.modinfo` and symbol sizes **byte-identical**; 41/41 symbol CRCs identical |
| Built for 5.15.199, from the 36.5.2 `nvidia-l4t-kernel-headers` and `-oot-headers` (downloaded, not installed) | vermagic `5.15.199-tegra`; **41/41 CRCs match** the 36.5.2 kernel and nvidia-oot `Module.symvers`; code and data identical to the vendor module |
| nvidia-oot `include/` (tegracam, camera_common), 36.5.0 against 36.5.2 | byte-identical |
| Base DTB `tegra234-p3768-0000+p3767-0005-nv-super.dtb`, 36.5.0 against 36.5.2 | byte-identical; `imx296-cam1.dtbo` applies to the 36.5.2 one (`fdtoverlay`) |
| The vendor's 5.15.185 prebuilt, forced onto 5.15.199 | not possible: besides the vermagic, 32 of its 40 imported symbols changed CRC, including every tegracam_* and camera_common_* (research) |

**Not yet run:** loading the 5.15.199 module on a 5.15.199 kernel. That is step 4 of the procedure below.

**The tools.** They live outside the repo, in `~/drive_logs/tools/` on the Jetson. A cloud session cannot see them.

| Tool | What it does |
|---|---|
| `imx296_driver/` | The rebuild. `src/` holds the recovered vendor tarball (sha256 `e4e86894…`), the frc971 header (`d27ae376…`) and the vendor's 5.15.185 prebuilt as the reference (`7061c58f…`). `build_src/` holds the patched `imx296.c`, `v1.1.patch`, `build-fix.patch` and the Makefile. `README.md` records the provenance and the checks. |
| `imx296_driver/build.sh [kver]` | Builds `out-<kver>/imx296.ko`, for the running kernel or from extracted header debs in `headers-<kver>/`. It **fails on any symbol-CRC mismatch**, then compares `.text`, `.rodata`, `.rodata.str1.8` and `.data` with the vendor's prebuilt. It only builds; it installs nothing. No package owns `imx296.ko`, so at every kernel change, build it, then install it as root: `install -D -m 0644 out-<kver>/imx296.ko /lib/modules/<kver>/kernel/drivers/media/i2c/imx296.ko && depmod -a <kver>`. The upgrade script does this for 5.15.199. |
| `l4t_upgrade_36_5_2.sh` | R36.5.0 → R36.5.2. Modes: `--list`, `--check`, `--simulate`, `--apply` (root). Details below. |
| `l4t_rollback_36_5_0.sh` | R36.5.2 → R36.5.0, the state verified on 2026-09-30. Same modes and the same checks. Rewritten on 2026-10-01: the 09-30 version could no longer run, because the apt history had rotated and the packages are held. |
| `l4t_36_5_2.pairs`, `l4t_36_5_0.pairs` | The 65 `package=version` pairs for each direction, frozen on 2026-10-01, because `/var/log/apt/history.log` rotates. The 36.5.2 list equals the `dpkg -l` that the rollback backed up before running. The 36.5.0 list equals what is installed now. |
| `verify_platform.sh [--quick]` | The test after any platform change. It changes nothing and stops no service. |
| `l4t_checks.sh` | The boot-file checks that the upgrade, the rollback and `verify_platform.sh` share: `extlinux_default_check`, `extlinux_entry_check` and `initrd_check`. It is sourced, not run, and every script that uses it stops if it is missing, so copy it along with them. It reads `extlinux.conf` the way NVIDIA's UEFI launcher does, and the initrd the way the kernel does (below). |
| `test_l4t_checks.sh` | Its regression test: 141 checks against good and broken boot entries, initrds and package lists built in a temporary directory, plus this machine's own `/boot` and packages, read-only. No root needed; about 15 s. Every rule is load-bearing: removing or weakening any one of them (56 mutants) makes it fail. |

What `l4t_upgrade_36_5_2.sh --apply` does:
1. Installs the 65 packages with `--allow-change-held-packages`, which **clears their holds** (seen on 2026-10-01:
   the first `--apply` stopped at the hold check), then holds them again with `apt-mark hold`.
2. Puts the 5.15.199 `imx296.ko` in place *before* apt, then runs `depmod`.
3. Restores `DEFAULT JetsonIO`, which the kernel package resets to `primary`.

Before it says "All checks passed", it checks:
- that `/boot/Image` is 5.15.199;
- that the uvcvideo, cdc-acm, imx296 and tegra-camera module files exist for 5.15.199, that imx296 is in
  `modules.dep`, and that `modprobe -n` resolves it;
- that `/boot/initrd` is whole (one gzip stream, one cpio archive, nothing hidden after either); that, unpacked the
  way the kernel does it, every entry is created; that it then carries nvme, nvme-core, pcie-tegra194 and phy-p2u
  for 5.15.199 where `modprobe` looks, byte-identical to the installed modules (the root file system is on NVMe,
  and these are modules); and that `/init` and everything it needs to reach the root are there;
- that `dpkg --audit` is clean;
- that `DEFAULT JetsonIO` is the only DEFAULT, and that the JetsonIO entry boots `/boot/Image` and `/boot/initrd`,
  with the running root's `root=PARTUUID=` and the cam1 overlay as its only camera overlay, every file present,
  and the overlay applying to its DTB (`fdtoverlay`);
- that each of the 65 target packages, by name, is held and installed at its target version.

Any failure ends with "do NOT reboot". Re-running it is safe.

`--check` reports `ready`, and `--simulate` shows 65 Inst, 65 Conf and 0 Remv.

What `verify_platform.sh` covers:
- **Platform:** kernel, release and UEFI; that each of the release's 65 packages is held and at its version (by
  name, from its pairs file); the boot entry and overlay;
  imx296 built for the running kernel, loaded and probed, with no I2C or capture errors; Argus and the reload unit;
  both cameras.
- **Tests, in the dev container:** `csi_test`, `csi_rtp_test`, `camera_gst_test` (with the CSI dictionary against
  live Argus), `scan_cameras`, `usb_test` with a control diff (skipped while `dashcam-v04` runs), CUDA and TensorRT.
- **Output:** logs to `~/drive_logs/platform_checks/<timestamp>/`.
- **Baseline on R36.5.0 (2026-10-01):** 23 passed, 0 failed (`platform_checks/20261001-081029`). TensorRT 14.1 ms
  mean.

**Review.** A workflow reviewed the upgrade script adversarially against the real 36.5.2 maintainer scripts. Every
finding went to a skeptic. Fixed:
- a failure mid-apt could leave an initrd without NVMe, and the old script did not say "do not reboot";
- a re-run after a partial run (an SSH drop, a dpkg error) silently skipped the driver and boot-entry steps;
- the rollback could not run.

Documented:
- the first boot re-applies the same-version 36.5.2 capsule, which takes a few minutes plus one extra automatic
  restart;
- after the upgrade there is no 5.15.185 kernel to fall back to.

Checked fine:
- hold marks survive (**wrong**, found on the real run: `--allow-change-held-packages` clears them; the
  scripts now re-hold after apt);
- no conffile prompts;
- nothing ships its own imx296 that would override this one;
- `nv-update-extlinux` keeps the JetsonIO entry and its FDT/OVERLAYS lines;
- the A_kernel fallback holds a bootable stock 5.15.199 (without the camera overlay).

**Second review, 2026-10-01 afternoon: the pre-reboot checks.** These checks decide whether the user may reboot,
so they were reviewed again, for what they let through.
- **The user's reviewer found two holes; both were reproduced.**
  - The initrd check threw away decompression errors (`zcat … || true`): an initrd missing its last 25% still listed
    all four NVMe modules, and passed.
  - The overlay check (`awk '/^LABEL JetsonIO/{f=1} f && /OVERLAYS/'`) never ended the entry. It accepted the overlay
    under a later entry, a commented-out `OVERLAYS` line and a nonexistent file. It also accepted a `.dtbo.bak` name
    and a `LABEL JetsonIO-old`.
  - The rollback script had the same two checks, and `verify_platform.sh` the overlay one.
- **A first fix moved both checks into `l4t_checks.sh`.** A workflow then attacked it: three attackers worked against
  NVIDIA's own sources (edk2-nvidia `L4TLauncher.c`, identical at r36.5, r36.5.1 and r36.5-updates; the r36.5
  kernel's `init/initramfs.c` and `lib/decompress_inflate.c`), and a skeptic re-ran every finding.
  - 26 findings: 25 confirmed and 1 plausible. One of the confirmed is a false alarm on initrd layouts that
    `nv-update-initrd` never writes, and needs no change.
  - All low severity: NVIDIA's tools (`jetson-io`, `nv-update-extlinux`, `nv-update-initrd`) never write such files.
    Only a hand edit or a broken writer would get through.
  - The confirmed ones that mattered:
    - the launcher matches keywords case-sensitively by prefix (`FDTDIR` is read as `FDT`), cuts every line at the
      first `#`, drops CRs, and reads only the first 10 entries;
    - the check never looked at `APPEND` (without it the launcher dereferences NULL, and the kernel has no `root=`);
    - a zero-filled or non-DTB overlay or FDT passed: the launcher then falls back to the kernel partition, without
      the camera;
    - the kernel unpacks everything after the first cpio trailer, and a second gzip member, which `cpio -t` never
      shows, and keeps the last copy of a name.
- **The rewrite parses both files the way their readers do,** in Python, called from `l4t_checks.sh`. It refuses
  anything it cannot read the same way as the boot path:
  - non-ASCII bytes;
  - keyword-like lines that are not plain `KEYWORD value`;
  - a JetsonIO entry that is not exactly one of the first 10, or that lacks any of LINUX, INITRD, FDT, OVERLAYS or
    APPEND, or has two of one;
  - an `APPEND` without the running root's `root=PARTUUID=`;
  - an overlay list with spaces, empty items, `.`/`..`/`//`, a file outside the root file system, or a second camera
    overlay;
  - overlays that `fdtoverlay` cannot apply to the FDT;
  - in the initrd: gzip header flags the kernel cannot skip, a cut-off stream, data after the stream or the cpio
    trailer, a bad cpio header, a duplicate name (after `lib -> usr/lib`), a hard link, modules not where the
    kernel's `/lib/modules` will be, or bytes that differ from the installed module.
- **Also fixed.**
  - The rollback now checks the boot entry in `--check` and before it changes anything (it used to find a broken
    entry only after the downgrade).
  - Boot-entry problems after apt now print as `WARNING`, which the grep in step 2 matches.
  - A re-run stopped by a bad entry says not to reboot if an earlier `--apply` ran.
  - An initrd failure names the repair: `nv-update-initrd` cannot fix a damaged initrd, so the message gives
    `sudo apt-get install --reinstall --allow-change-held-packages nvidia-l4t-initrd`.
  - The scripts find `l4t_checks.sh` when run through a symlink.
- **Verified on the Jetson.**
  - `test_l4t_checks.sh`: 109/109 checks, and all 37 mutants caught. The reviewer's faults, and the attackers' data
    after the cpio trailer or in a second gzip member, are each shown to have passed the old check. The attackers
    showed their other cases passing the first fix.
  - The live `/boot` passes.
  - So does an initrd built the way 36.5.2 will build it: the attacker ran the 36.5.2 `nv-update-initrd` (identical
    to 36.5.0's) under fakeroot on the 36.5.2 base initrd and kernel modules. So does `extlinux.conf` as the
    attacker's run of the 36.5.2 kernel package's `nv-update-extlinux` left it.
  - Upgrade and rollback `--check` report `ready`.
  - A stubbed `--apply` (apt, install, depmod and `sed -i` replaced) passes on the live R36.5.0 state.
  - The real upgrade's checks, run before its packages, give `do NOT reboot` and exit 3.
  - `verify_platform.sh --quick`: 13/13.
- **Known false alarm.** An initrd layout that `nv-update-initrd` never writes (hard links, the crc cpio format)
  fails with a clear message, although the kernel could boot it. This machine does not make such initrds.

**Third review, 2026-10-01 evening: two more gaps, both reproduced and fixed.**
- **The initrd check read the archive, not what the kernel would unpack.** A copy of the live initrd without the
  NVMe modules' parent directories passed, and so did one without `/init`.
  - The kernel creates a file with `filp_open(O_CREAT)`. When the parent directory does not exist (yet), that fails
    and the entry is skipped, silently.
  - Without an executable `/init`, the kernel tries to mount `root=` itself, and nvme is a module.
- **`initrd_check` now unpacks the archive into a model of the kernel's rootfs** (`init/initramfs.c`: `do_name`,
  `do_symlink`, `clean_path`), starting from the built-in initramfs (`/dev`, `/dev/console`, `/root`).
  - Every entry must be created. A file written twice, or a directory entry with data, fails.
  - The modules are looked up through symlinks at `/lib/modules/<kver>/`, where `modprobe` looks.
  - It requires an executable `/init`, and everything NVIDIA's R36.5 `/init` needs on its way to an NVMe root:
    - bash, its `#!` interpreter;
    - `mount`, `cat`, `grep`, `sed`, `tail`, `ln`, `kmod`, `modprobe`, `sleep`, `expr` and `chroot`, on bash's
      default PATH (the kernel gives `/init` none);
    - for each of them, its ELF loader and every `DT_NEEDED` library, in the loader's default directories (the
      initrd has no `ld.so.cache`).

    The command list comes from the R36.5 `/init`. If NVIDIA changes that script, update `INIT_COMMANDS` in
    `l4t_checks.sh`.
- **The hold check counted; it did not compare.** With `nvidia-l4t-kernel` unheld and an unrelated `nvidia-*`
  package held, the count was still 65, and it passed.
  - `holds_check` now compares the held names with the 65 in the pairs file, and names any that is missing.
  - Added: `versions_check`. Each of the 65 must be installed (dpkg state `i`, no error flag; held packages show
    `hi`) at exactly its target version.
  - The upgrade and the rollback run both before the reboot. `verify_platform.sh` runs both against its release's
    pairs file.
- **Verified on the Jetson.**
  - `test_l4t_checks.sh`: 141/141. The new cases:
    - missing parent directories (also shown to have passed the old check), directories after their files, a file
      replacing a directory;
    - no `/init`, or one that is not executable; a missing interpreter, loader, library or command; a cut-off ELF;
    - the kernel unheld behind another hold; wrong versions; dpkg error flags.
  - 56 mutants, each removing or weakening one rule: all caught.
  - The live `/boot/initrd` passes, and so does the simulated 36.5.2 initrd.
  - Both reported scenarios, end to end through the rollback's real check section (an `apt-mark` shim; the edited
    initrds in place of `/boot/initrd`): `do NOT reboot`, exit 3.
  - Upgrade and rollback `--check` report `ready`, and the stubbed rollback `--apply` passes.
  - `verify_platform.sh --quick`: 14/14. The new version check is the extra one.
- **Note.** A stubbed upgrade `--apply` can no longer pass. With apt stubbed the packages do not move, and the
  version check says so; that is what it is for.

**Upgrade procedure** (the user, on the bench, with HDMI and a keyboard or the serial console attached):
1. `bash ~/drive_logs/tools/l4t_upgrade_36_5_2.sh --check`, then `--simulate`.
2. `sudo systemctl stop dashcam-v04`. Then run the apply at the console, or with nohup over SSH, because tmux is not
   installed:
   - `sudo -v`, on its own line: it asks for the password in the foreground.
   - `sudo nohup bash ~/drive_logs/tools/l4t_upgrade_36_5_2.sh --apply > ~/l4t_upgrade.log 2>&1 &`
   - `tail -f ~/l4t_upgrade.log`

   The log must show `All checks passed.` (the script's next steps follow it), and no `WARNING`, `WRONG`, `FAILED`
   or `do NOT reboot` line: `grep -E 'All checks passed|WARNING|WRONG|FAILED|do NOT reboot|upgrade:' ~/l4t_upgrade.log`.
3. `sudo reboot` on bench power. The first boot writes the capsule and restarts once by itself; do not cut power. If
   it hangs at the camera driver, choose "primary kernel" in the boot menu, which boots without the overlay.
4. `bash ~/drive_logs/tools/verify_platform.sh`. Expect kernel 5.15.199, R36.5.2, UEFI 36.5.2, `Detected IMX296LQ`,
   `csi_test` 5/5, both cameras, CUDA and TensorRT.
5. If the camera fails:
   - `bash ~/drive_logs/tools/l4t_rollback_36_5_0.sh --check`, then `--simulate`.
   - `sudo systemctl stop dashcam-v04`: it starts at boot, and `--apply` refuses while it runs.
   - `sudo bash ~/drive_logs/tools/l4t_rollback_36_5_0.sh --apply`, at the console or with nohup as in step 2.
   - Reboot, then `verify_platform.sh`.

   The rollback restores `DEFAULT JetsonIO` itself. The bootloader stays 36.5.2 either way.

**Licence.** The driver is GPL v2 (file header and `MODULE_LICENSE`). The frc971 repository is MIT. Keeping
`imx296_driver/` in this repo would let a cloud session see it, and turn future kernel updates into a routine rebuild.
That is the user's call.

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

## 2026-09-27 → 10-03: status by commit (historical; the current state is under Start here)

| Item | Commit | Status | Still needs |
|---|---|---|---|
| Platform | — | **Upgraded to L4T R36.5.2 / JetPack 6.2.3+b81 / kernel 5.15.199-tegra on 2026-10-01; UEFI also 36.5.2.** Existing 16:18 verification log: 24 PASS, 0 FAIL, 1 SKIP; all 65 target packages at target versions and held. See the 2026-10-01 evening section. | `usb_test` was skipped while the recorder owned the USB camera; evening paired USB captures succeeded, but do not substitute for that full test. |
| IMX296 driver for any kernel | — | Rebuilt from the vendor's source plus FRC 971's mode table; 41/41 CRCs match 36.5.2. **Now loaded and hardware-tested on 5.15.199:** platform log has `csi_test` 5/5, CSI RTP and Camera_GST PASS; evening tuning captured from the IMX296 through Argus. Build/upgrade/rollback tools remain outside the repo in `/home/jetson/drive_logs/tools/`. | The 2026-10-03 probe failure (`-121`, no I2C ACK) was a loose ribbon, fixed by a reseat. Road-use validation is still open. |
| CSI image tuning | — | **c8_sh15 is installed (2026-10-02 06:53, verified live)**: c5_rpi100T's colour (black level 50 + the IMX296 colour matrix the vendor file had transposed) plus the sharpness table set to its weakest index, so the ISP's default output matches ee-mode=0. The c7 A/B brings it closer to the USB camera (ΔE 11.7 → 6.7 once the CSI JPEGs are decoded correctly; first reported as 8.56 → 4.79). A review found that gain is mostly in-sample and about zero against the chart's own values, so **do not install c7 as it is**. The app sets no CSI properties, and with c8_sh15 that default is now clean (no halos). Details in "Review of the c7 work" and the c8 sections below. | A daylight A/B (`--auto`) before calling it production; the CSI saturation in the app's config. |
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
| libcommlink: fixes | `68fcceb` | CommLink logged its port open/close lines with the port mutex held, so a log callback that asked the link anything deadlocked (latent; v0.4's does not). The bridge tty and the wake pipe are now close-on-exec (children such as `nmcli` inherited them). `typeName()` now names `CMD_SET_IMU_MODE` in both `HostProtocol.h` copies, which stay byte-identical; the wire is unchanged. Discovery keeps partial results when a node vanishes mid-scan. `Uart::read(0)` no longer makes two `fcntl()` calls per read. New `logPrintf()` in liblog replaces three private log helpers. | **Run on the Jetson 2026-09-28:** `commlink_test` recovers from a cable pull on the C3's by-id path (what v0.4 uses), and v0.4's bridge line is as before. Auto-discover mode fails on this rig, which predates the pass (item U). Items U, V fixed 2026-10-03; C3 runs pass, the replug check needs sudo. |
| libbus | `5be7a5b` | `ibus.h` (the base of libuart, libspi and libi2c) moved from `lib/libgpio` to `lib/libbus`. There is no code change. | — |
| CSI driver loader | `aa9deb1` | The Jetson's own IMX296 unit (`imx296-reload.service`) stopped nvargus-daemon, then failed at `modprobe -r imx296`, because 5.15.199-tegra has no imx296 module at all (only 5.15.185-tegra has one), and left Argus stopped (found by the 2026-09-28 review; corrected by the Jetson run). `docker_dev/csi_driver.sh` checks the module against the running kernel before touching Argus, and restarts Argus on every exit path; `csi-driver.service` (in the repo, not installed) would load it before Argus at boot. The script does not fix a kernel/module mismatch by itself: on 5.15.199 the module would have had to be rebuilt, and the 2026-09-30 rollback to 5.15.185 removed the mismatch instead. Host test: `test_csi_driver.sh`, 31 checks. | Not needed on R36.5.0: there, the vendor's `imx296-reload.service` works as intended (active since the rollback). **Items K–P fixed 2026-10-03.** `csi-driver.service` (the `reload` mode) is installed and enabled in place of the vendor unit; start/restart and `csi_test` 5/5 are checked. The first boot with it passed (2026-10-03 20:25: ordered after Argus, `Result=success`, `csi_test` 5/5). |
| libcamera: review fixes | `f03a13f` | Shutdown is bounded when a branch stops consuming (it hung, reproduced on the Jetson) or the source's thread is stuck. A stream that ends reports ERROR (it stayed RUNNING). Branches that start disabled start (item C). frameCount restarts at `start()`. Items B, D, E, F (docs), G, H, J and both older races fixed. `camera_gst_test` 72 → 120 checks. | **Run on the Jetson 2026-09-28:** `camera_gst_test` PASS 14/14; the reviewer's shapes redone on the UGREEN. **2026-09-30 on CSI:** `csi_test` 5/5, `csi_rtp_test` PASS, and Camera_CSI stops no slower than before, with no WARN (below). Items Q–T fixed 2026-10-03; R confirmed on Argus (`csi_test` 5/5, `csi_rtp_test`, no WARN). |
| usb_test | `782bdfd` | Puts the controls it writes back as found, checks that, and reads back what Tests 3 and 4 set (items A, B). | **Run on the Jetson 2026-09-28:** PASS, the before/after control diff is empty; again on 2026-09-30 (R36.5.0). Item Z fixed 2026-10-03, confirmed on the UGREEN. |
| Focus hold | `3e3dc4f` | `UvcFocusControl` refused nothing when it could not read what to hand back: autofocus came back on for a camera found in manual, or the manual lens position was not restored (the review's mocked read failures). It now refuses before writing anything. New `focus_test`: 22 checks against a simulated camera. | **Run on the Jetson 2026-09-28:** v0.4 with `FocusMode=fixed` holds the lens and hands autofocus back. Items X, Y fixed 2026-10-03; X confirmed on the UGREEN. |
| commlink_sim_test | `0f548d7` | CommLink against a simulated C3 on a pseudo-terminal, with no hardware: 73 checks in about 6 s (handshake, every frame type, commands, watchdog, hot-plug, discovery, libuart). Passed 36 of 36 runs, 16 of them overloaded; ASan, UBSan and TSan are clean. | **Run on the Jetson 2026-09-28:** PASS 16/16, 5 of them with every core busy. Item W fixed 2026-10-03. |

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

The procedure (Reference: "Rig procedure: the bus-error proof") is kept for reruns after a Wire change.

## 2026-09-27: after flashing, one drive answers the rest (the 2026-10-03 drive answered most of it)

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
