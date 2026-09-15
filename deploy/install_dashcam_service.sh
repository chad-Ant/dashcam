#!/bin/bash
#
# install_dashcam_service.sh — install/remove the systemd unit that starts the
# dashcam at boot on a Jetson Orin Nano.
#
# The unit lives on the HOST, not in the container: udev rules, systemd and log
# rotation belong host-side while the application, models and GStreamer pipelines
# stay containerised.  It runs docker_dev/launchcode_v0_4.sh in the foreground so
# the container's PID 1 is the dashcam binary itself, which is what lets
# `systemctl stop` deliver SIGTERM to it and finalise the recording.
#
# Usage:
#   sudo deploy/install_dashcam_service.sh [--now] [--user U] [--repo DIR]
#                                          [--media DIR] [--name N]
#   sudo deploy/install_dashcam_service.sh --uninstall
#
#   --now        start the service immediately as well as enabling it at boot
#   --user U     host user owning the repo and media (default: the invoking
#                user via SUDO_USER, else the repo directory's owner)
#   --repo DIR   repo/build tree   (default: /home/<user>/dashcam)
#   --media DIR  backup drive root (default: /media/<user>/backup)
#   --name N     systemd unit name (default: dashcam)
set -euo pipefail

UNIT_NAME="dashcam"
START_NOW=0
UNINSTALL=0
HOST_USER=""
REPO_DIR=""
MEDIA_DIR=""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

while [ $# -gt 0 ]; do
    case "$1" in
        --now)       START_NOW=1; shift ;;
        --uninstall) UNINSTALL=1; shift ;;
        --user)      HOST_USER="${2:?--user needs a value}"; shift 2 ;;
        --repo)      REPO_DIR="${2:?--repo needs a value}"; shift 2 ;;
        --media)     MEDIA_DIR="${2:?--media needs a value}"; shift 2 ;;
        --name)      UNIT_NAME="${2:?--name needs a value}"; shift 2 ;;
        -h|--help)   sed -n '2,25p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1 (try --help)" >&2; exit 2 ;;
    esac
done

UNIT_PATH="/etc/systemd/system/${UNIT_NAME}.service"

if [ "$(id -u)" -ne 0 ]; then
    echo "FATAL: must run as root (use sudo)" >&2
    exit 1
fi

if [ "$UNINSTALL" -eq 1 ]; then
    systemctl stop    "$UNIT_NAME" 2>/dev/null || true
    systemctl disable "$UNIT_NAME" 2>/dev/null || true
    rm -f "$UNIT_PATH"
    systemctl daemon-reload
    docker rm -f "$UNIT_NAME" >/dev/null 2>&1 || true
    echo "removed $UNIT_PATH"
    exit 0
fi

# ── resolve identity and paths ────────────────────────────────────────────────
# Never hardcode a username: the skill docs assume 'jetson' but the rig is the
# operator's.  Prefer whoever invoked sudo, then fall back to whoever owns the
# repository checkout.
if [ -z "$HOST_USER" ]; then
    HOST_USER="${SUDO_USER:-}"
fi
if [ -z "$HOST_USER" ]; then
    HOST_USER="$(stat -c '%U' "$DEFAULT_REPO")"
fi
REPO_DIR="${REPO_DIR:-$DEFAULT_REPO}"
MEDIA_DIR="${MEDIA_DIR:-/media/$HOST_USER/backup}"
LAUNCHER="$REPO_DIR/docker_dev/launchcode_v0_4.sh"

echo "unit      : $UNIT_PATH"
echo "user      : $HOST_USER"
echo "repo      : $REPO_DIR"
echo "media     : $MEDIA_DIR"
echo "launcher  : $LAUNCHER"
echo

# ── pre-flight ────────────────────────────────────────────────────────────────
fail=0
note() { echo "  $1"; }

command -v docker >/dev/null 2>&1 || { note "MISSING: docker is not installed"; fail=1; }
[ -x "$LAUNCHER" ] || [ -f "$LAUNCHER" ] || { note "MISSING: $LAUNCHER"; fail=1; }

if command -v docker >/dev/null 2>&1; then
    docker info 2>/dev/null | grep -q 'Runtimes:.*nvidia' \
        || note "WARNING: docker reports no 'nvidia' runtime — --runtime nvidia will fail"
    docker image inspect "${DASHCAM_IMAGE:-l4t-ml-gpio}:latest" >/dev/null 2>&1 \
        || note "WARNING: image ${DASHCAM_IMAGE:-l4t-ml-gpio}:latest not found — run docker_dev/build.sh"
fi

# The service fails fast rather than building at boot, so a missing binary means
# it will not start.  Warn now, while someone is looking at a terminal.
if ! ls -1d "$REPO_DIR"/bin/build_*/dashcam_v0_4 >/dev/null 2>&1; then
    note "WARNING: no dashcam_v0_4 binary built yet — the service will fail to start."
    note "         Build it:  docker_dev/launchcode_dev.sh  then  make -j6 dashcam_v0_4"
fi

mountpoint -q "$MEDIA_DIR" 2>/dev/null \
    || note "WARNING: $MEDIA_DIR is not mounted — footage would go to the internal disk"

[ "$fail" -eq 0 ] || { echo; echo "pre-flight failed; nothing installed." >&2; exit 1; }

chmod +x "$LAUNCHER" 2>/dev/null || true

# ── write the unit ────────────────────────────────────────────────────────────
# Ordering notes:
#   * No nvargus-daemon dependency: v0.4 records from USB and only touches Argus
#     if <Startup><LaneDetection> is switched on.  That also avoids the classic
#     "Argus wedged after an unclean shutdown" restart loop.
#   * ExecStartPre clears a stale container so --name can be reused.
#   * `docker stop -t 30` gives the binary 30 s to flush EOS and finalise the
#     last segment and sidecar; TimeoutStopSec stays above it so systemd never
#     SIGKILLs mid-finalise.
cat > "$UNIT_PATH" <<EOF
[Unit]
Description=Dashcam v0.4 (USB recording)
Documentation=https://github.com/chad-Ant/dashcam
Requires=docker.service
After=docker.service local-fs.target

[Service]
Type=simple
Environment=DASHCAM_USER=$HOST_USER
Environment=DASHCAM_SRC=$REPO_DIR
Environment=DASHCAM_MEDIA=$MEDIA_DIR
Environment=DASHCAM_NAME=$UNIT_NAME
ExecStartPre=-/usr/bin/docker rm -f $UNIT_NAME
ExecStart=$LAUNCHER
ExecStop=/usr/bin/docker stop -t 30 $UNIT_NAME
Restart=always
RestartSec=10
TimeoutStopSec=45
KillSignal=SIGTERM
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "$UNIT_NAME" >/dev/null
echo "installed and enabled at boot."

if [ "$START_NOW" -eq 1 ]; then
    systemctl restart "$UNIT_NAME"
    echo "started."
fi

echo
echo "  watch:    journalctl -u $UNIT_NAME -f"
echo "  status:   systemctl status $UNIT_NAME"
echo "  stop:     sudo systemctl stop $UNIT_NAME      # finalises the last segment"
echo "  remove:   sudo $0 --uninstall"
