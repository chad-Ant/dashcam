#!/bin/bash
# cycle.sh [--keep <cand>] [--props "<nvarguscamerasrc props>"] <candidate>...   (run with sudo)
#   --props: capture properties (default "ee-mode=0"; "" = the ISP's own sharpening)
#   --keep:  the candidate installed at the end (default c1_black50)
# For each candidate (a .isp file next to this script, name without .isp): install it as
# the Argus ISP override, restart nvargus-daemon, then capture the IMX296 and the UGREEN
# as the user jetson (pair.sh). At the end, install KEEP (default c1_black50) again.
# Changes nothing else; the vendor original is vendor_v1.1.isp next to this script.
set -u
[ "$(id -u)" -eq 0 ] || { echo "run it with sudo"; exit 1; }
HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
SET=/var/nvidia/nvcam/settings/camera_overrides.isp
SHOTS_OWNER=jetson
KEEP="${KEEP:-c1_black50}"
PROPS="ee-mode=0"
while [ $# -gt 0 ]; do
    case "$1" in
        --keep)  KEEP="$2"; shift 2 ;;
        --props) PROPS="$2"; shift 2 ;;
        *) break ;;
    esac
done
for c in "$@"; do [ -r "$HERE/$c.isp" ] || { echo "no $HERE/$c.isp"; exit 1; }; done
[ -r "$HERE/$KEEP.isp" ] || { echo "no $HERE/$KEEP.isp"; exit 1; }
for c in "$@"; do
    install -m 0644 "$HERE/$c.isp" "$SET" && systemctl restart nvargus-daemon && sleep 3
    tag="${c}"; [ "$PROPS" = "ee-mode=0" ] || tag="${c}_p$(tr -c 'A-Za-z0-9' '_' <<< "${PROPS:-isp}" | sed 's/_*$//')"
    # shellcheck disable=SC2086
    sudo -u "$SHOTS_OWNER" bash "$HERE/pair.sh" "${tag}_ec0" $PROPS
    # shellcheck disable=SC2086
    sudo -u "$SHOTS_OWNER" bash "$HERE/pair.sh" "${tag}_ec0.5" $PROPS exposurecompensation=0.5
done
install -m 0644 "$HERE/$KEEP.isp" "$SET" && systemctl restart nvargus-daemon
echo "== done; installed again: $KEEP ($(sha256sum "$SET" | cut -c1-12))"
