# Handover — branch `camera`, 2026-10-03 (night drive prepared; see the 21:10 section)

## Where things stand

| Item | Commit | Status | Still needs |
|---|---|---|---|
| Platform | — | **Upgraded to L4T R36.5.2 / JetPack 6.2.3+b81 / kernel 5.15.199-tegra on 2026-10-01; UEFI also 36.5.2.** Existing 16:18 verification log: 24 PASS, 0 FAIL, 1 SKIP; all 65 target packages at target versions and held. See the evening update below. | `usb_test` was skipped while the recorder owned the USB camera; evening paired USB captures succeeded, but do not substitute for that full test. |
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
| v0.4 | 29 segments, **0 warnings or errors during the drive** (the only WARN is the 22:14:44 port close at the planned restart); night mode the whole way |
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
    before the next drive.
- **Yaw sign checked over the whole drive:**
  - it is opposite to the GNSS course rate in 92 % of turns over 5 °/s (correlation −0.56, as left-positive
    requires);
  - left indicator with |yaw| > 8 °/s: yaw positive in 213 of 219 samples; right indicator: negative in 192 of 197.
  - So the sign convention is right. The rare mismatches are real driving (signalling during an opposite curve).

Still to analyse: the High-G events against the footage, and CAN speed against GNSS speed.

## 2026-10-03 21:10 (Jetson): rig prepared for the second drive, a night drive (code uncommitted)

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
  - UGREEN focus, exposure and gain every 1 s (with autofocus on, `focus_absolute` is the live lens position);
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
- **Footage rates:** CSI night segments are about 12 MB/s (3× indoors; noise inflates the JPEGs) and the UGREEN
  about 7 MB/s, so about 68 GB/h. With 577 GB free that is about 8 h before v0.4's 20 GB floor.
- **CSI at night:** bright and sharp, no halos. But the mount is rolled about 10° and the lower third of the frame is
  dashboard.
- **UGREEN autofocus fails at night.** After power-up it sat at the default `focus_absolute` 512 with
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
- Then decide whether to commit the telemetry work.

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

## 2026-10-01 evening (Jetson): CSI/USB chart tuning and current rig state

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

## Items from the Jetson run and a second review (K–Z): all fixed 2026-10-03, see "Done 2026-10-03"

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
$B/usb_test --self-test                              # usb_test's stop-and-restore path on a simulated camera
docker_dev/test_csi_driver.sh                        # csi_driver.sh against stubs (123 checks)
docker_dev/csi_driver.sh check                       # on the Jetson, read-only: the IMX296 module vs the running kernel
$B/commlink_sim_test                                 # CommLink + libuart against a simulated C3 on a pty: no hardware
$B/commlink_test --by-id 30                          # the real C3 bridge, by-id only as v0.4 (stop dashcam-v04 first)
$B/commlink_test "" 30                               # auto-discover with the ttyACM fallback
$B/record_test                                       # Part A needs GStreamer base/good/bad/ugly plugins; Part B a camera
$B/usb_test                                          # a USB camera (stop dashcam-v04 first); puts its controls back
$B/scan_cameras                                      # lists the cameras, their controls and formats
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
- **Jetson only:** CUDA targets, `record_test` Part B, `usb_test`, `csi_test`, `scan_cameras` (with a camera),
  `commlink_test` (with the C3), `csi_driver.sh`, and everything on the rig or the car.
