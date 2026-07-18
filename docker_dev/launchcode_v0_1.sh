#!/bin/bash
#
# launchcode_v0_1.sh — launch the dashcam v0.1 software immediately.
#
# Same container/mount setup as launchcode_dev.sh, but instead of dropping into
# an interactive shell it starts the newest built dashcam_v0_1 binary right
# away (building it first if none exists).  The binary runs as the container's
# foreground process, so Ctrl+C (SIGINT) or `docker stop` (SIGTERM) reaches it
# directly and triggers the graceful shutdown path: recording valves closed,
# EOS flushed, both MKV files finalised.
#
# Storage: /user/output/{configs,footage,logs} are bind mounts onto the backup
# media; if the media is missing the software falls back to build-local
# directories inside /user/dashcam/bin/build_<ts>/ automatically.

IMAGE_NAME="l4t-ml-gpio"
INIT_DIR="/user/dashcam"
SRC_DIR="/home/$USER/dashcam"        # repo/source + build tree (working dir)
MEDIA_DIR="/media/$USER/backup"      # backup drive holding footage/logs/configs
HOST_TZ="$(cat /etc/timezone 2>/dev/null || echo UTC)"   # overlay clock + filenames

echo "Booting Dashcam v0.1: $IMAGE_NAME... (TZ=$HOST_TZ)"

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
        BIN=$(ls -td bin/build_*/dashcam_v0_1 2>/dev/null | head -1)
        if [ -z "$BIN" ]; then
            echo "dashcam_v0_1 not built yet — building..."
            make -j6 || exit 1
            BIN=$(ls -td bin/build_*/dashcam_v0_1 2>/dev/null | head -1)
        fi
        [ -n "$BIN" ] || { echo "build produced no dashcam_v0_1 binary"; exit 1; }
        echo "starting $BIN"
        exec "$BIN"
    '
