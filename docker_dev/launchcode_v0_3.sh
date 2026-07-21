#!/bin/bash
#
# launchcode_v0_3.sh — launch the dashcam v0.3 software immediately.
#
# Same container/mount setup as launchcode_dev.sh, but instead of dropping into
# an interactive shell it starts the newest built dashcam_v0_3 binary right
# away (building it first if none exists).  The binary runs as the container's
# foreground process, so Ctrl+C (SIGINT) or `docker stop` (SIGTERM) reaches it
# directly and triggers the graceful shutdown path: recording valves closed,
# EOS flushed, MKV finalised, lane + driver inference threads joined.
#
# v0.3 = v0.2 (UVC passthrough recording + UFLD v2 lanes on the IMX296) plus
# DRIVER DROWSINESS MONITORING on a driver-facing UVC camera (libdriverstate:
# Viola-Jones face crop + binary ResNet18 TRT classifier, 2 Hz).  Alerts are
# logged at WARN after P(drowsy) holds >= threshold for 2 s.
#
# Camera roles: librecord opens its USB camera exclusively, so recording and
# driver monitoring need SEPARATE UVC devices.  Auto mode gives the first USB
# camera to recording and the next YUYV-capable one to the driver monitor; to
# pin a specific device as the cabin camera add to <Cameras> in dashcam.xml:
#   <Camera name="cabin" type="USB"><Device>/dev/video2</Device></Camera>
# (pinning the ONLY camera runs driver monitoring without recording).
# Engine paths and driver knobs live in the <Detection> config section.
#
# Storage: /user/output/{configs,footage,logs} are bind mounts onto the backup
# media; if the media is missing the software falls back to build-local
# directories inside /user/dashcam/bin/build_<ts>/ automatically.

IMAGE_NAME="l4t-ml-gpio"
INIT_DIR="/user/dashcam"
SRC_DIR="/home/$USER/dashcam"        # repo/source + build tree (working dir)
MEDIA_DIR="/media/$USER/backup"      # backup drive holding footage/logs/configs
HOST_TZ="$(cat /etc/timezone 2>/dev/null || echo UTC)"   # overlay clock + filenames

echo "Booting Dashcam v0.3: $IMAGE_NAME... (TZ=$HOST_TZ)"

docker run -it --rm --network=host --privileged --ipc=host \
    --runtime nvidia \
    -w $INIT_DIR \
    -e TZ="$HOST_TZ" \
    -v /dev:/dev \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v $SRC_DIR:/user/dashcam \
    -v $MEDIA_DIR/configs:/user/output/configs \
    -v $MEDIA_DIR/footage:/user/output/footage \
    -v $MEDIA_DIR/logs:/user/output/logs \
    -v /etc/localtime:/etc/localtime:ro \
    -v /etc/timezone:/etc/timezone:ro \
    $IMAGE_NAME bash -c '
        cd /user/dashcam || exit 1
        BIN=$(ls -td bin/build_*/dashcam_v0_3 2>/dev/null | head -1)
        if [ -z "$BIN" ]; then
            echo "dashcam_v0_3 not built yet — building..."
            make -j6 || exit 1
            BIN=$(ls -td bin/build_*/dashcam_v0_3 2>/dev/null | head -1)
        fi
        [ -n "$BIN" ] || { echo "build produced no dashcam_v0_3 binary"; exit 1; }
        echo "starting $BIN"
        exec "$BIN"
    '
