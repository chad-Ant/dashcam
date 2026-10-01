#!/bin/bash
# Run as root from a terminal. Tests c7 against c5, restoring the entry profile
# even on ordinary error/interrupt. No recorder restart and no permanent selection.
set -euo pipefail
HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
SET=/var/nvidia/nvcam/settings/camera_overrides.isp
CAND="$HERE/session_20261001_2146/c7_usb_half.isp"
BASE="$HERE/c5_rpi100T.isp"
[ "$(id -u)" -eq 0 ] || { echo 'Run with sudo bash trial_c7.sh'; exit 1; }
if systemctl is-active --quiet dashcam-v04; then echo 'Recorder active; stop it first.'; exit 1; fi
cmp -s "$SET" "$BASE" || { echo 'Installed profile changed since this experiment. Refusing to overwrite it.'; exit 1; }
[ -s "$CAND" ] || { echo 'Candidate missing'; exit 1; }
[ "$(sha256sum "$CAND" | cut -d' ' -f1)" = bd5c2cacf7696d71ddd63896dddf94ac09dbed619fa8278ba59d322af799d9c0 ] || {
    echo 'Candidate changed since the offline evaluation; review it first.'; exit 1;
}
STAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP="$(mktemp -d /var/backups/csi-trial.XXXXXX)"
cp -a "$SET" "$BACKUP/camera_overrides.isp"
echo "Backup: $BACKUP/camera_overrides.isp"
restore() {
    rc=$?
    trap - EXIT INT TERM
    if install -m 0644 "$BACKUP/camera_overrides.isp" "$SET" && systemctl restart nvargus-daemon; then
        echo 'Original CSI profile restored; recorder remains stopped.'
    else
        echo "RESTORE FAILED: restore $BACKUP/camera_overrides.isp to $SET and restart nvargus-daemon" >&2
        rc=1
    fi
    exit "$rc"
}
trap restore EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for kind in base candidate; do
    profile="$BASE"; [ "$kind" != candidate ] || profile="$CAND"
    install -m 0644 "$profile" "$SET"
    systemctl restart nvargus-daemon
    sleep 3
    out="$HERE/trial_${STAMP}_${kind}"
    echo "Capturing $kind -> $out"
    timeout -k 10s 110s sudo -u jetson /usr/bin/python3 "$HERE/tune_session.py" "$out" --manual
done
echo 'A/B captures complete. Ask for analysis of the two new trial directories.'
