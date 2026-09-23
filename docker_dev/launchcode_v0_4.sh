#!/bin/bash
#
# launchcode_v0_4.sh — launch dashcam v0.4 (recording only).
#
# v0.4 records ONE USB UVC camera's own compressed stream (MJPEG/H.264, no
# re-encode) into gapless 3-minute MKV segments with an .ass clock sidecar,
# starting as soon as the program starts.  Loop overwrite keeps its own
# footage under <Recording><MaxFootageGB> and free space above <MinFreeGB>.
# No lane detection, no driver monitoring, no network.
#
# Startup is fast: the newest existing bin/build_*/dashcam_v0_4 is reused and
# only compiled when none exists (or with --rebuild).  Inside the container a
# restart loop brings the program back if it crashes or exits non-zero (e.g.
# its teardown watchdog fired); a clean exit (SIGINT/SIGTERM) ends the loop.
# No terminal is needed, so systemd can run this script (see below).
#
# Storage: /user/output/{configs,footage,logs} are bind mounts onto
# $MEDIA_DIR; if the footage mount is unusable the program records into
# <repo>/footage_fallback/ (same loop-overwrite rules) and moves back when the
# mount works again.
#
# Autostart at boot (needs sudo once):
#   sudo cp /home/$USER/dashcam/docker_dev/dashcam-v04.service /etc/systemd/system/
#   sudo systemctl daemon-reload
#   sudo systemctl enable --now dashcam-v04.service
# Logs: journalctl -u dashcam-v04 -f      Stop: sudo systemctl stop dashcam-v04
#
# Usage: [SRC_DIR=...] [MEDIA_DIR=...] launchcode_v0_4.sh [--rebuild]

IMAGE_NAME="l4t-ml-gpio"
CONTAINER_NAME="dashcam_v04"
INIT_DIR="/user/dashcam"
SRC_DIR="${SRC_DIR:-/home/$USER/dashcam}"      # repo/source + build tree (working dir)
MEDIA_DIR="${MEDIA_DIR:-/media/$USER/backup}"  # backup drive holding footage/logs/configs
HOST_TZ="$(cat /etc/timezone 2>/dev/null || echo UTC)"   # sidecar clock + filenames

REBUILD=0
[ "$1" = "--rebuild" ] && REBUILD=1

# -t only with a real terminal: under systemd there is none.
TTY_FLAGS="-i"
[ -t 0 ] && TTY_FLAGS="-it"

echo "Booting Dashcam v0.4: $IMAGE_NAME... (TZ=$HOST_TZ)"

# A container left over from an unclean shutdown would block the name.
docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1

exec docker run $TTY_FLAGS --rm --name "$CONTAINER_NAME" --stop-timeout 20 \
    --network=host --privileged \
    -w $INIT_DIR \
    -e TZ="$HOST_TZ" \
    -e REBUILD="$REBUILD" \
    -v /dev:/dev \
    -v $SRC_DIR:/user/dashcam \
    -v $MEDIA_DIR/configs:/user/output/configs \
    -v $MEDIA_DIR/footage:/user/output/footage \
    -v $MEDIA_DIR/logs:/user/output/logs \
    -v /etc/localtime:/etc/localtime:ro \
    -v /etc/timezone:/etc/timezone:ro \
    $IMAGE_NAME bash -c '
        cd /user/dashcam || exit 1
        BIN=$(ls -td bin/build_*/dashcam_v0_4 2>/dev/null | head -1)
        if [ "$REBUILD" = 1 ] || [ -z "$BIN" ]; then
            echo "building dashcam_v0_4 from the mounted source..."
            make -j6 dashcam_v0_4 || exit 1
            BIN=$(ls -td bin/build_*/dashcam_v0_4 2>/dev/null | head -1)
        fi
        [ -n "$BIN" ] || { echo "no dashcam_v0_4 binary"; exit 1; }

        # Restart loop.  Signals are forwarded to the running binary; its exit
        # status then decides: 0 = graceful stop (leave), otherwise restart.
        STOP=0
        CHILD=0
        trap "STOP=1; [ \$CHILD -ne 0 ] && kill -TERM \$CHILD" TERM INT
        while :; do
            echo "starting $BIN"
            "$BIN" &
            CHILD=$!
            wait $CHILD; RC=$?
            # wait returns early when a trapped signal arrives; reap the child.
            while kill -0 $CHILD 2>/dev/null; do wait $CHILD; RC=$?; done
            CHILD=0
            [ $STOP -eq 1 ] && exit 0
            [ $RC -eq 0 ] && exit 0
            echo "dashcam_v0_4 exited with status $RC — restarting in 2 s"
            sleep 2
        done
    '
