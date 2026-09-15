#!/bin/bash
#
# launchcode_v0_4.sh — launch the dashcam v0.4 software, boot-safe.
#
# Same container/mount setup as launchcode_v0_3.sh, but written to be run by
# systemd at boot (deploy/install_dashcam_service.sh) as well as by hand.  Four
# differences from the v0.3 launcher, each of them a thing that breaks or hurts
# when nobody is at a terminal:
#
#   1. No `-it`.  `docker run -it` fails outright under systemd with "the input
#      device is not a TTY" — there is no terminal to attach to.
#   2. `--name` so `docker stop` has something to target.  With `--rm` and no
#      name there is no handle for an ExecStop, and SIGTERM never reaches the
#      binary, so the recording is never finalised.
#   3. FAIL FAST when the binary is missing — it never builds.  The v0.3 script
#      runs `make -j6 dashcam_v0_3` on every launch; in a boot path that is a
#      multi-minute compile before the first frame is recorded, with no signal
#      handler installed while it runs.  Build deliberately over SSH instead.
#   4. Paths come from the environment, not `$USER`.  A systemd system unit has
#      no login environment, so `$USER` is empty and `/home/$USER/dashcam`
#      silently becomes `/home//dashcam`.
#
# Override any of these before invoking:
#   DASHCAM_USER   host user owning the repo and media    (default: $USER, else root)
#   DASHCAM_SRC    repo/source + build tree               (default: /home/$DASHCAM_USER/dashcam)
#   DASHCAM_MEDIA  backup drive root                      (default: /media/$DASHCAM_USER/backup)
#   DASHCAM_NAME   container name                         (default: dashcam)
#
# The binary runs as the container's foreground process (exec), so `docker stop`
# (SIGTERM) reaches it directly and triggers the graceful shutdown path:
# recording stopped, EOS flushed, the final MKV segment and its .ass sidecar
# finalised.
set -u

IMAGE_NAME="${DASHCAM_IMAGE:-l4t-ml-gpio}"
INIT_DIR="/user/dashcam"
HOST_USER="${DASHCAM_USER:-${USER:-root}}"
SRC_DIR="${DASHCAM_SRC:-/home/$HOST_USER/dashcam}"
MEDIA_DIR="${DASHCAM_MEDIA:-/media/$HOST_USER/backup}"
CONTAINER="${DASHCAM_NAME:-dashcam}"
HOST_TZ="$(cat /etc/timezone 2>/dev/null || echo UTC)"

echo "Booting Dashcam v0.4: $IMAGE_NAME  (user=$HOST_USER TZ=$HOST_TZ)"

if [ ! -d "$SRC_DIR" ]; then
    echo "FATAL: source tree '$SRC_DIR' does not exist (set DASHCAM_SRC)" >&2
    exit 1
fi

# Fail fast rather than compiling inside the boot path.  Resolve by NAME order
# rather than mtime: the build directories are timestamped, and `ls -t` would
# prefer a freshly-touched old build over the newest one.
BIN_REL=$(cd "$SRC_DIR" && ls -1d bin/build_*/dashcam_v0_4 2>/dev/null | sort | tail -1)
if [ -z "$BIN_REL" ]; then
    echo "FATAL: no dashcam_v0_4 binary under $SRC_DIR/bin/build_*/" >&2
    echo "       Build it first:  docker_dev/launchcode_dev.sh  then  make -j6 dashcam_v0_4" >&2
    exit 1
fi
echo "binary: $BIN_REL"

# Storage banner.  Docker CREATES a missing bind-mount source as root, so if the
# backup drive is not mounted the three directories below are silently created
# on the internal disk, they are writable, resolveStorageDir()'s probe passes,
# and footage lands on the NVMe with no warning anywhere.  Recording anyway is
# the deliberate choice (better than missing a drive) — but say so loudly, since
# this banner is the only thing that distinguishes the two cases afterwards.
if mountpoint -q "$MEDIA_DIR" 2>/dev/null; then
    echo "storage: $MEDIA_DIR is mounted — footage/logs/configs go to the backup drive"
else
    echo "############################################################"
    echo "WARNING: $MEDIA_DIR is NOT a mountpoint."
    echo "         Recording will continue, but footage, logs and config"
    echo "         will be written to the INTERNAL DISK under that path,"
    echo "         not to the backup drive. Check the drive is plugged in"
    echo "         and mounted, then: systemctl restart dashcam"
    echo "############################################################"
fi

# Only bind /tmp/argus_socket when the daemon has actually created it.  v0.4
# never touches Argus unless <Startup><LaneDetection> is on, and mounting a
# non-existent path makes Docker create a DIRECTORY there, which then breaks a
# real CSI run in the dev container.
ARGUS_MOUNT=()
if [ -S /tmp/argus_socket ]; then
    ARGUS_MOUNT=(-v /tmp/argus_socket:/tmp/argus_socket)
else
    echo "note: /tmp/argus_socket absent — not mounted (CSI/lane detection unavailable)"
fi

# Clear a stale container left by an unclean stop so --name can be reused.
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

exec docker run --rm --name "$CONTAINER" \
    --network=host --privileged --ipc=host \
    --runtime nvidia \
    -w $INIT_DIR \
    -e TZ="$HOST_TZ" \
    -e DBUS_SYSTEM_BUS_ADDRESS=unix:path=/run/dbus/system_bus_socket \
    -v /dev:/dev \
    "${ARGUS_MOUNT[@]}" \
    -v /run/dbus:/run/dbus \
    -v /run/NetworkManager:/run/NetworkManager \
    -v "$SRC_DIR":/user/dashcam \
    -v "$MEDIA_DIR/configs":/user/output/configs \
    -v "$MEDIA_DIR/footage":/user/output/footage \
    -v "$MEDIA_DIR/logs":/user/output/logs \
    -v /etc/localtime:/etc/localtime:ro \
    -v /etc/timezone:/etc/timezone:ro \
    "$IMAGE_NAME" \
    /user/dashcam/"$BIN_REL"
