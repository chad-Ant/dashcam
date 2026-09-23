#!/bin/bash
#
# remove_autostart_v0_4.sh — take dashcam v0.4 out of boot autostart.
#
# Undoes the install steps in dashcam-v04.service: stops the service (the
# running recording is finalised by the graceful docker stop), disables it,
# deletes the unit file and reloads systemd.  Footage, logs, configs and
# builds are left untouched.  Safe to run more than once.
#
# Usage: sudo ./remove_autostart_v0_4.sh
# (re-runs itself with sudo when started as a normal user)

SERVICE="dashcam-v04.service"
UNIT_FILE="/etc/systemd/system/$SERVICE"
CONTAINER_NAME="dashcam_v04"

if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi

echo "Removing $SERVICE from autostart..."

# Stop first so the current segment is finalised (ExecStop = docker stop -t 20).
if systemctl is-active --quiet "$SERVICE"; then
    echo "stopping $SERVICE (finalising the current recording)..."
    systemctl stop "$SERVICE"
fi

if systemctl is-enabled --quiet "$SERVICE" 2>/dev/null; then
    systemctl disable "$SERVICE"
fi

# A container started by the service outside systemd's view (or left over
# from an unclean stop) would keep recording; stop it gracefully too.
if docker ps -q --filter "name=^${CONTAINER_NAME}$" | grep -q .; then
    echo "stopping leftover container $CONTAINER_NAME..."
    docker stop -t 20 "$CONTAINER_NAME" >/dev/null
fi

if [ -f "$UNIT_FILE" ]; then
    rm -f "$UNIT_FILE"
    echo "deleted $UNIT_FILE"
fi

systemctl daemon-reload
systemctl reset-failed "$SERVICE" 2>/dev/null

if systemctl list-unit-files "$SERVICE" 2>/dev/null | grep -q "$SERVICE"; then
    echo "WARNING: $SERVICE is still known to systemd (installed somewhere other than $UNIT_FILE?)"
    exit 1
fi
echo "done: dashcam v0.4 will no longer start at boot."
echo "(run docker_dev/launchcode_v0_4.sh to start it manually)"
