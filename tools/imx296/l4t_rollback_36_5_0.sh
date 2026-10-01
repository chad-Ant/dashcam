#!/bin/bash
#
# l4t_rollback_36_5_0.sh — put this Jetson back on L4T R36.5.0 (JetPack 6.2.2,
# kernel 5.15.185-tegra), the state that was verified on 2026-09-30/10-01 with
# the IMX296 in CAM1.
#
# It installs exactly the 65 NVIDIA packages listed in l4t_36_5_0.pairs (next to
# this script), each at the version that runs and was tested.  That file was
# frozen on 2026-10-01 from the apt history and checked against dpkg; the apt
# history itself rotates.  The packages are on apt-mark hold; apt clears the hold
# of each package it installs, so --apply holds them again afterwards.
#
# Why the whole BSP and not just the kernel: the kernel's out-of-tree modules
# (nvgpu, the display driver, the camera stack Argus talks to) are built for the
# userspace of their own release.
#
# The bootloader stays 36.5.2: UEFI refuses the 36.5.0 capsule (its lowest
# supported version is 36.5.2).  That combination was tested and works.
#
# Usage (read-only modes first; only --apply changes anything):
#   l4t_rollback_36_5_0.sh --list        the pkg=version pairs it would install
#   l4t_rollback_36_5_0.sh --check       is every version in the apt indexes?
#   l4t_rollback_36_5_0.sh --simulate    apt-get -s: what apt would do (nothing is changed)
#   sudo bash l4t_rollback_36_5_0.sh --apply   the downgrade, then DEFAULT JetsonIO and checks
#
# Run --apply inside tmux (or with nohup).  Re-running it is safe.  --check and
# --apply first check the JetsonIO boot entry, so a problem that is already
# there stops it before anything is downgraded.
#
# History: first written 2026-09-28; ran 2026-09-30 (a pre-reboot check bug
# fixed then); 2026-10-01: frozen package list, held packages allowed, the
# boot entry restored by the script, initrd/audit checks, failure banner;
# later that day the initrd and boot-entry checks moved to l4t_checks.sh
# (a truncated initrd and a misplaced/commented/missing overlay now fail).

set -euo pipefail

HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"   # also when run through a symlink
PAIRS_FILE="$HERE/l4t_36_5_0.pairs"
KOLD="5.15.185-tegra"
EXTLINUX="/boot/extlinux/extlinux.conf"
OVERLAY="tegra234-p3767-camera-p3768-imx296-cam1.dtbo"
BACKUP_DIR="/var/backups/l4t-rollback-$(date +%Y%m%d-%H%M%S)"

die() { echo "rollback: $*" >&2; exit 1; }

# extlinux_default_check, extlinux_entry_check, initrd_check
[ -r "$HERE/l4t_checks.sh" ] || die "missing $HERE/l4t_checks.sh"
# shellcheck source=l4t_checks.sh
. "$HERE/l4t_checks.sh"

PAIRS=""
load_pairs() {
    [ -r "$PAIRS_FILE" ] || die "missing $PAIRS_FILE"
    PAIRS="$(grep -E '^[a-z0-9.+-]+=[^ ]+$' "$PAIRS_FILE")" || die "no pairs in $PAIRS_FILE"
    [ "$(wc -l <<< "$PAIRS")" -eq 65 ] || die "$PAIRS_FILE does not hold 65 packages"
}

# The JetsonIO entry must carry the CAM1 overlay, and every file it names must
# exist; checked before anything is changed, and again before the reboot.
entry_ok() {
    local out
    if out="$(extlinux_entry_check "$EXTLINUX" JetsonIO "$OVERLAY")"; then
        echo "ok       $out"
    else
        sed 's/^/WRONG    /' <<< "$out"; return 1
    fi
}

cmd_list() { load_pairs; echo "$PAIRS"; echo "($(wc -l <<< "$PAIRS") packages)" >&2; }

cmd_check() {
    load_pairs
    local missing=0 p pkg ver
    while IFS= read -r p; do
        pkg="${p%%=*}"; ver="${p#*=}"
        if apt-cache madison "$pkg" 2>/dev/null | awk -F'|' '{gsub(/ /,"",$2); print $2}' | grep -qxF "$ver"; then
            :
        else
            echo "MISSING  $pkg $ver (not in the apt indexes; try 'sudo apt update' first)"; missing=$((missing + 1))
        fi
    done <<< "$PAIRS"
    echo "packages: $(wc -l <<< "$PAIRS"), unavailable: $missing"
    [ -f "/lib/modules/$KOLD/kernel/drivers/media/i2c/imx296.ko" ] \
        && echo "ok       imx296.ko for $KOLD in place" \
        || { echo "MISSING  imx296.ko for $KOLD (build it: imx296_driver/build.sh $KOLD)"; missing=$((missing + 1)); }
    entry_ok || missing=$((missing + 1))
    [ "$missing" -eq 0 ] || die "$missing problem(s); do not --apply"
    echo "ready"
}

# apt-get simulation, run the way --apply runs (held packages allowed, -y); fails
# if apt would REMOVE anything.  Here-strings, so grep -q cannot SIGPIPE a writer.
simulate() {
    [ -n "$PAIRS" ] || load_pairs
    local out
    # shellcheck disable=SC2086
    out="$(apt-get install -s -y --allow-downgrades --allow-change-held-packages --no-install-recommends $PAIRS 2>&1)" || {
        echo "$out"; die "apt refuses this set (above)"; }
    { grep -E '^(Inst|Remv|Conf)' <<< "$out" || true; } | awk '{print $1}' | sort | uniq -c
    if grep -q '^Remv' <<< "$out"; then
        grep '^Remv' <<< "$out"
        die "apt would REMOVE the packages above; do not --apply"
    fi
    grep -E '^Inst nvidia-l4t-(kernel|bootloader|initrd|display-kernel) ' <<< "$out" || true
    grep -qE '^Inst ' <<< "$out" || echo "(every package is already at the target version)"
    echo "simulation: nothing removed"
}

cmd_apply() {
    [ "$(id -u)" -eq 0 ] || die "--apply needs root (sudo bash $0 --apply)"
    if systemctl is-active --quiet dashcam-v04; then die "stop dashcam-v04 first: sudo systemctl stop dashcam-v04"; fi
    entry_ok || die "the JetsonIO boot entry is not right (above); fix it before rolling back. Nothing was changed by this run; but if an earlier --apply already ran, do NOT reboot until it is fixed"
    load_pairs
    echo "== simulation"
    simulate
    echo "== backups in $BACKUP_DIR"
    mkdir -p "$BACKUP_DIR"
    cp -a "$EXTLINUX" "$BACKUP_DIR/"
    dpkg -l > "$BACKUP_DIR/dpkg-l.before.txt"
    echo "$PAIRS" > "$BACKUP_DIR/pairs.txt"
    trap 'rc=$?; [ "$rc" -eq 0 ] || echo "== FAILED (rc=$rc): do NOT reboot or power off. Fix the error above, run sudo dpkg --configure -a, then rerun --apply. Backups: $BACKUP_DIR" >&2' EXIT
    echo "== downgrading"
    # shellcheck disable=SC2086
    apt-get install -y --allow-downgrades --allow-change-held-packages --no-install-recommends $PAIRS
    # apt clears the hold of every held package it installs (seen 2026-10-01: all 65
    # went from "hi" to "ii"), so hold them again; holds_check below verifies it
    # shellcheck disable=SC2046
    apt-mark hold $(cut -d= -f1 <<< "$PAIRS") > /dev/null
    echo "== held again: $(wc -l <<< "$PAIRS") packages"
    depmod -a "$KOLD"
    echo "== boot entry"
    sed -i 's/^DEFAULT primary$/DEFAULT JetsonIO/' "$EXTLINUX"
    echo "== checks before any reboot"
    local kver ok=1 out
    kver="$(grep -aoE -m1 'Linux version [^ ]+' /boot/Image | awk '{print $3}' || true)"
    echo "/boot/Image: ${kver:-unknown}"
    [ "$kver" = "$KOLD" ] || { echo "WARNING: /boot/Image is not $KOLD — do not reboot"; ok=0; }
    for m in kernel/drivers/media/usb/uvc/uvcvideo.ko kernel/drivers/usb/class/cdc-acm.ko \
             kernel/drivers/media/i2c/imx296.ko updates/drivers/media/platform/tegra/camera/tegra-camera.ko; do
        if [ -f "/lib/modules/$KOLD/$m" ]; then echo "ok  $m"; else echo "WARNING: missing /lib/modules/$KOLD/$m — do not reboot"; ok=0; fi
    done
    if modprobe -n -q -S "$KOLD" imx296; then echo "ok  imx296 and its dependencies resolve for $KOLD"; else echo "WARNING: modprobe cannot resolve imx296 for $KOLD"; ok=0; fi
    # The root file system is on NVMe, and nvme/pcie are modules: the initrd must
    # carry them, and must itself be whole (a cut-off archive can still list them).
    if out="$(initrd_check /boot/initrd "$KOLD" "/lib/modules/$KOLD/kernel/drivers" \
            nvme/host/nvme.ko nvme/host/nvme-core.ko pci/controller/dwc/pcie-tegra194.ko phy/tegra/phy-tegra194-p2u.ko)"; then
        echo "ok  $out"
    else
        sed 's/^/WARNING: /; s/$/ — do NOT reboot/' <<< "$out"; ok=0
        echo "         (nv-update-initrd cannot repair a damaged initrd; rebuild it with: sudo apt-get install --reinstall"
        echo "          --allow-change-held-packages nvidia-l4t-initrd, then rerun --apply)"
    fi
    if [ -z "$(dpkg --audit 2>&1)" ]; then echo "ok  dpkg --audit clean"; else echo "WARNING: dpkg --audit reports unfinished packages"; ok=0; fi
    if out="$(extlinux_default_check "$EXTLINUX" JetsonIO)"; then echo "ok  $out"; else sed 's/^/WARNING: /' <<< "$out"; ok=0; fi
    if out="$(extlinux_entry_check "$EXTLINUX" JetsonIO "$OVERLAY")"; then echo "ok  $out"; else sed 's/^/WARNING: /' <<< "$out"; ok=0; fi
    # the target packages by name, not a count: each one held, and installed at its target version
    if out="$(holds_check <(printf '%s\n' "$PAIRS"))"; then echo "ok  $out"; else sed 's/^/WARNING: /' <<< "$out"; ok=0; fi
    if out="$(versions_check <(printf '%s\n' "$PAIRS"))"; then echo "ok  $out"; else sed 's/^/WARNING: /' <<< "$out"; ok=0; fi
    [ "$ok" -eq 1 ] || { echo "== something above is wrong: do NOT reboot; the backups are in $BACKUP_DIR"; trap - EXIT; exit 3; }
    trap - EXIT
    cat <<'EOT'

All checks passed.  Next:
  1. sudo reboot, on bench power (HDMI + keyboard or the serial console attached).
     The bootloader stays 36.5.2 (UEFI refuses the 36.5.0 capsule; that is fine).
  2. Then:  bash ~/drive_logs/tools/verify_platform.sh
EOT
}

case "${1:-}" in
    --list)     cmd_list ;;
    --check)    cmd_check ;;
    --simulate) simulate ;;
    --apply)    cmd_apply ;;
    *) sed -n '2,24p' "$0"; exit 1 ;;
esac
