# Jetson dashcam

A headless recorder for a Jetson Orin Nano in a Honda Brio. The active recorder, `dashcam_v0_4`, saves the UGREEN USB camera's compressed video directly to disk. Dashcam Connect lets an Android phone or web browser view the camera, read vehicle telemetry and browse recordings. For the installed versions and next bench or road tests, see the [project handover](HANDOVER.md).

## What runs where

| Component | Role |
| --- | --- |
| Jetson recorder | The UGREEN Camera 4K supplies 1920×1080 MJPEG at 30 fps in the rig configuration. v0.4 writes three-minute MKV segments with matching ASS clock and telemetry captions, rotates old footage against storage limits, and can record with the telemetry bridge disconnected. |
| Vehicle peripherals | The Arduino MKR Zero collects CAN, GNSS and IMU data. The ESP32-C3 bridges telemetry to the Jetson over USB. The optional ES3C28P ESP32-S3 display has a camera preview and a map page. [Bridge firmware](peripherals/esp32-c3/README.md) · [Display firmware](peripherals/es3c28p/README.md) |
| Jetson monitor | The host's `dashcam-monitor` service reads recorder updates over local sockets, samples host health independently and serves authenticated Android and browser clients. An enrolled `dashcam-uplink` service connects it to the optional cloud relay. |
| Camera and inference development | The IMX296 colour CSI camera in CAM1 is used for lane evaluation at 1456×1088. The lane harness and earlier v0.2/v0.3 applications are separate targets. v0.4 records one USB camera. [Lane evaluation procedure](tools/lane_eval/DRIVE_PROCEDURE.md) |

## Viewing live and recorded video

| In the Android app or browser | Direct LAN | Cloud pilot |
| --- | --- | --- |
| Live view with fullscreen | SD is 720×404 at up to 10 fps for the UGREEN. HD sends its original JPEG frames at camera rate, up to 1080p30 on this rig. | SD with fullscreen. The uplink sends the reduced preview only. |
| Telemetry | Paired clients receive recorder state, vehicle readings when available, and Jetson health. | Authorized owners and viewers can monitor the enrolled vehicle over mobile data or Wi-Fi. |
| Recordings | List finished segments, play and seek with captions, or download the original MKV and ASS. | The branch implements export of up to three minutes from one segment as an H.264 MP4, with optional burned-in captions and one-hour signed links. Pilot deployment and end-to-end acceptance are pending. |

## Run the recorder on the Jetson

The rig uses a Jetson Linux host with Docker and the NVIDIA container runtime. The launcher expects the checkout at `~/dashcam` and media under `/media/$USER/backup` by default. The media directory holds `configs/`, `footage/`, `logs/` and `navcache/`. The recorder reads the mounted `configs/dashcam.xml`. The [repository example](config/dashcam.xml) identifies the recording camera by its stable USB capture link.

Footage goes to the SD card, and the M.2 system SSD is the fallback. The card is mounted on the host at `/mnt/sdcard` by an `/etc/fstab` entry with `nofail` and check pass `2`. With `exfatprogs` installed, the card is checked and safely auto-repaired at boot before it is mounted, because exFAT has no journal and a slot failure or power cut can leave it marked dirty. The launcher shares host `/mnt` into the container as `/user/output/mnt`, so mounts made later also reach the running recorder.

To record to the card, edit `<System>` in `$MEDIA_DIR/configs/dashcam.xml`, which is the file the recorder reads; the [repository example](config/dashcam.xml) already has these values. Set `<FootagePath>` to `/user/output/mnt/sdcard/footage` and `<FootageMountPoint>` to `/user/output/mnt/sdcard`. Without them, the recorder keeps writing to `$MEDIA_DIR/footage` on the SSD.

The recorder uses the card only while it is mounted and takes writes. If the card is missing at boot, pulled, failing or read-only, recording continues in `<FallbackFootagePath>`, which defaults to `<repo>/footage_fallback` on the SSD. Recording moves back to the card within about a minute of the card being mounted and working again.

The [systemd unit](docker_dev/dashcam-v04.service) waits for `mnt-sdcard.mount` at boot. It deliberately does not require that mount, so the recorder still starts without a card. After updating the unit, copy it into `/etc/systemd/system/` again and run `sudo systemctl daemon-reload`.

The [udev rules](docker_dev/99-dashcam-sdcard.rules) mount a card inserted after boot and detach a pulled card's dead mount. Without them, run `sudo umount -l /mnt/sdcard; sudo mount /mnt/sdcard` after inserting a card.

To remove the card cleanly, stop everything that uses it, then unmount it: `sudo systemctl stop dashcam-v04 smbd dashcam-monitor`, then `sudo umount /mnt/sdcard`. Check that `findmnt /mnt/sdcard` prints nothing before pulling the card. If `umount` reports "target is busy", `sudo fuser -vm /mnt/sdcard` shows what still holds the mount. Start the services again after inserting a card. Point the LAN connector's `FOOTAGE_PATH` and the Samba share at `/mnt/sdcard/footage`. Only v0.4 checks `<FootageMountPoint>`. With the card missing, v0.2 and v0.3 write into the empty `/mnt/sdcard` directory on the system disk.

On a new rig, verify the media mount and camera, build the image, then launch in the foreground. The launcher reuses the newest recorder binary or builds one if needed. Use `--rebuild` to pick up C++ changes. On a rig already recording under systemd, use the service checks below before starting another instance.

```bash
cd ~/dashcam
findmnt -T "/media/$USER/backup"
findmnt /mnt/sdcard
ls -l /dev/v4l/by-id/
bash docker_dev/build.sh
bash docker_dev/launchcode_v0_4.sh --rebuild
```

For boot-time recording, follow the install instructions in the [systemd unit](docker_dev/dashcam-v04.service). On an installed rig, check `systemctl is-active dashcam-v04` and `journalctl -u dashcam-v04 -n 40 --no-pager`. The footage directory should gain a finished MKV and matching ASS after each segment closes.

## Set up Dashcam Connect

1. Install the LAN monitor on the Jetson host, point it at the recorder's footage directory, and enable the monitoring and video settings in the deployed recorder configuration. [Jetson setup](connector/README.md#jetson-setup) · [Camera video](connector/README.md#camera-video)
2. Pair Android from the monitor's pairing link. For a LAN browser, trust the local CA and use an address listed in the certificate. The monitor serves the web app itself. [Browser setup and pairing](connector/README.md#browser-access)
3. For remote access, configure the pilot relay, enroll the Jetson and build the Android client for the deployed relay origin. [Cloud deployment and enrollment](connector/cloud/README.md)

For Android development on Windows, run the checked-in wrapper from `android/` with JDK 17+ and Android SDK 37:

```powershell
.\gradlew.bat :app:assembleDebug :app:testDebugUnitTest :app:lintDebug
```

For development without the rig, the agent simulator serves synthetic telemetry and a camera test card. For results on the S25 Ultra, A54 and desktop browser, see the validation record. [Simulator and tests](connector/README.md#simulator-and-tests) · [Device validation](connector/VALIDATION.md)
