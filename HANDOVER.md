# Project handover

**27 November 2026 · branch `camera` · reviewed through `6fdfd48`.** The recorder and LAN viewing have run on the bench. The next release task is to deploy the eight recordings fixes and prove a cloud clip export from the running recorder to a phone and a browser. The deployment state below comes from the last recorded checks on 26–27 November. [Project overview and setup](README.md)

## What is running

| Component | Last confirmed state | What remains |
| --- | --- | --- |
| Bench rig, 27 Nov | Wired Jetson at `192.168.0.138`, Orin Nano, L4T R36.5.2 / JetPack 6.2.3. IMX296 in CAM1 at `/dev/video0` passed a 60-frame Argus capture at 1456×1088/60. The UGREEN exposed `video1` and `video2`; use the by-id `video-index0` capture link. MKR Zero, C3 and ES3C28P were detached. | Reconnect peripherals for vehicle telemetry and display tests. With the deployed preview enabled and display absent, v0.4 logged a DEBUG retry every second. |
| Recorder and LAN monitor | `dashcam-v04` was recording 180 s UGREEN segments with build `bin/build_20261126_082828` from `aa99454`. The Jetson checkout and running `dashcam-monitor` reached `eaa8c6e`. mDNS advertised the wired address with `LAN_ADDRESS=auto`. | Pull the recordings fixes and restart the monitor. No recorder C++ changed in `6fdfd48`. LAN discovery follows an address change; browser TLS still needs the new address in the certificate. |
| Uplink and cloud | The enrolled uplink reconnected after its 27 Nov restart, but the installed `/opt/dashcam-uplink/uplink.py` predates cloud recordings. The last recorded relay revision was `dashcam-relay-staging-00002-w8j` on 22 Nov. | Check the active revision, IAM and bucket. Stage relay and web before updating the uplink or clients. A service restart alone reloads the installed copy. |
| Android and browser | The S25 release passed LAN list, fullscreen playback with captions and downloads after the Android fixes in `5e840b6`. Both downloaded files matched their Jetson originals. The S25 also showed LAN SD at 720×404/10 fps, LAN HD at 1920×1080/30 fps and cloud SD in fullscreen. | Install the release with `6fdfd48`. Phone storage, interrupted playback and cloud clip delivery still need physical checks on that version. |

### 2026-11-21 Dashcam Connect cloud pilot

Vehicle A completed S25 mobile-data viewing, simultaneous S25/A54 sessions, A54 fallback to LAN and a ten-minute idle check. These trials covered telemetry and live video. The signed APK hashes, relay revision and timing measurements are in the [original pilot log](https://github.com/chad-Ant/dashcam/blob/6fdfd4895308d2cccae158430c8ccedf44c8627f/HANDOVER.md#2026-11-21-dashcam-connect-cloud-pilot). [Connector validation record](connector/VALIDATION.md)

## Recordings fixes awaiting deployment

All eight findings were fixed in `6fdfd48`. The recorded checks passed 74 agent tests, 17 cloud tests and 60 Android JVM tests. Android lint passed and the browser reconnect scenarios ran against a scripted relay. On real footage the new agent identified five interrupted segments out of 831. The phone download and retry paths still need verification on the new APK.

| Priority | Failure on the previous version | Fix to verify |
| --- | --- | --- |
| High | An interrupted Android re-download could erase a previously saved video or captions. | Stage both files, check their sizes, then publish the pair. Interrupt each transfer on the S25 and compare the earlier pair's hashes. |
| High | A signing error could disconnect a viewer or the vehicle, interrupting other viewers' telemetry. | Handle upload and download signing failures per request. A second viewer should stay live and the next export should succeed. |
| High | After a vehicle reconnected, an export on the old socket could stay pending. | Bind requests to their vehicle socket, fail them when it closes or is replaced, and ignore late replies. Silent list requests expire at 60 s and clip requests at 10 min. |
| Medium | Hiding a browser tab during a request could leave Refresh or Export disabled. | Fail handlers on socket close. Reload interrupted lists after reconnect and offer **Export again** for interrupted exports. |
| Medium | A clip starting past the final frame could produce a small MP4 with no video. | Reject known out-of-range starts with HTTP 416 and abort empty ffmpeg output when duration is unknown. Verify that a clip starting inside the segment still plays through its end. |
| Medium | A power-interrupted MKV could appear playable, then fail on open. | Check the segment index and expose `playable`. Both LAN clients should disable Play for a cut-short segment and still allow downloading it. |
| Medium | Refreshing the cloud list could discard export progress or a signed link. | Keep state per vehicle and segment across refreshes. The same export should finish once, retain its links and offer another export after expiry. |
| Medium | Android told the user to retry after losing Wi-Fi but disabled Play. | Treat a transport failure as recoverable. Restore Wi-Fi and resume from the paused position. Invalid media and a refused segment remain terminal. |

The four earlier live-view findings were fixed in `aa99454`: the HD sender backs off on a full queue, the agent falls back to SD after five seconds of HD silence, Android checks the HD capability, and a late browser callback cannot reopen fullscreen after exit. The recorder build reached the bench. Sustained HD load, a real stalled receiver, mid-session fallback and physical browser fullscreen remain field checks. [Detailed live-view review and regression results](https://github.com/chad-Ant/dashcam/blob/6fdfd4895308d2cccae158430c8ccedf44c8627f/HANDOVER.md#2026-11-26-live-view-review-fixes-not-deployed)

## Next deployment

Use the recordings-fix revision or a descendant for the relay, web assets, Jetson monitor, uplink and APK. The pilot project is `jetson-dashcam-handler` in `asia-southeast1`. [Cloud setup commands](connector/cloud/README.md) · [Detailed pilot checklist and rollback commands](https://github.com/chad-Ant/dashcam/blob/6fdfd4895308d2cccae158430c8ccedf44c8627f/HANDOVER.md#2026-11-27-cloud-recordings-configuration-partly-deployed)

1. **Cloud operator:** Verify that the relay account can sign URLs through IAM Credentials. Check or create the private `jetson-dashcam-handler-clips` bucket with public access prevention, relay object create/view access and a one-day deletion lifecycle.
2. **Relay and web:** Record the revision currently serving traffic. Compare the live default-database Firestore rules with the repository copy because `connector/cloud/stage.sh` republishes the entire ruleset. Stage with the clip bucket configured and verify sign-in on the new revision. Keep the previous revision for rollback. An older relay closes a viewer socket on a recordings request.
3. **Jetson:** Fast-forward the checkout, restart `dashcam-monitor`, and check the authenticated identity for recording and clip capabilities. Back up the installed `uplink.py`, install the reviewed copy from `connector/cloud/` to `/opt/dashcam-uplink/`, restart `dashcam-uplink` and compare the installed file with the checkout. Keep the existing enrollment. Check both journals while exporting through the systemd service.
4. **Android:** Build against the pilot relay origin and install with the existing release signing identity on the S25 to preserve pairings. The A54 uses its debug signing identity. Record the APK SHA-256, agent commit, installed uplink and serving relay revision at rollout.

### Acceptance on the new release

- **Cloud clips:** From a browser and the S25, list recordings, export a 30 s Small clip and a three-minute 720p clip, play and download. Exercise owner and viewer access. Confirm the burned-in clock at the chosen offset, fresh telemetry for a second viewer, and uninterrupted recorder segments. Check that links expire after one hour and the bucket lifecycle removes the object.
- **LAN recordings:** Interrupt a replacement download during both video and captions on the S25. Verify that the previous pair survives, a retry yields one correctly named pair, and playback can resume after a Wi-Fi drop. Open a cut-short segment in the phone and desktop lists. Check external-player caption pickup, A54 playback rate and a visible browser.
- **Live view and connectivity:** Repeat SD and HD on car Wi-Fi with a phone and laptop. Record delivered fps, frame age, recorder fps, CPU, phone heat and HD drops. Test fullscreen rotation, loss of HD with SD fallback, LAN reconnection after router or Jetson restart, Android token-refresh retry, A54 mobile data, tablet/browser CA setup and vehicle map tiles. [Field validation notes](connector/VALIDATION.md)

## Vehicle and bench backlog

| Workstream | Next action and evidence needed |
| --- | --- |
| CSI and lane evaluation | Rigidly remount and level the IMX296, verify the stock driver and calibrate the crop and mount reference while parked. The gain floor helped, but a hot module still reset in direct sun. Capture daytime and night runs and record CSI gaps. The lane harness needs exclusive camera access; run it separately from the recorder. [Drive procedure](tools/lane_eval/DRIVE_PROCEDURE.md) |
| Camera image quality | Capture the three CSI calibration sheets, then compare night settings, unlit-road plate exposure and moving autofocus. UGREEN fixed focus 690 was verified on the rig. [Chart capture and analysis](tools/isp_tuning/chart/v2/CALIBRATION.md) |
| Vehicle telemetry | Parked with engine on, compare ten minutes of raw CAN loss with the earlier roughly 0.4% baseline. Investigate BNO055 corrupt bursts, measure MKR clock lock and C3 transmit stalls, then check CAN/GNSS loss and recovery on the clients. [CAN capture tools](peripherals/mkr_zero/tools/canstream/README.md) |
| ESP32-S3 display | ES3C28P touch, audio and USB preview were bench-tested at 10 and 15 fps. Reconnect it to measure USB throughput and JPEG redraw, then verify map panning, page changes, stale GNSS, reconnect and whole-frame fallback before enabling navigation on the rig. [Display plan](peripherals/es3c28p/NAVIGATION_PLAN.md) |
| Calibration and owner decisions | Measure rear-axle-to-antenna offset and run the turning EKF on a measured course. Decide the high-G incident threshold, CAN speed scale, treatment of stale fusion rows and the lane inference rate after collecting drive data. [Turning calibration](peripherals/mkr_zero/tools/canstream/turning/README.md) |
| Later prototypes | GRF-250 ranging still needs converter-level verification and a hardware bench test. The encoder-supervised pan servo needs hardware, mount and calibration before integration. [Ranging library](lib/liblidartracking/README.md) · [Pan controller](peripherals/mkr_zero/libraries/EncoderPan/README.md) |

Earlier driver recovery procedures, test evidence, APK hashes, drive logs and deferred code items remain in the [full handover at the cleanup baseline](https://github.com/chad-Ant/dashcam/blob/6fdfd4895308d2cccae158430c8ccedf44c8627f/HANDOVER.md). To read it offline, run `git show 6fdfd48:HANDOVER.md`.
