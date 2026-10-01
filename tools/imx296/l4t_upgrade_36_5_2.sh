#!/bin/bash
#
# l4t_upgrade_36_5_2.sh — move this Jetson from L4T R36.5.0 (kernel 5.15.185) to
# R36.5.2 (JetPack 6.2.3, kernel 5.15.199) and keep the IMX296 CSI camera, by
# installing the imx296.ko rebuilt for 5.15.199 from the vendor's own source
# (imx296_driver/, see its README.md).
#
# It re-applies exactly the NVIDIA packages of the 2026-09-23 upgrade, at the
# versions that upgrade installed: l4t_36_5_2.pairs (frozen 2026-10-01 from
# /var/log/apt/history.log*, which rotates) — the reverse of
# l4t_rollback_36_5_0.sh.  The packages are on apt-mark hold; apt clears the hold
# of each package it installs, so --apply holds them again afterwards.
# The QSPI bootloader is already 36.5.2 (UEFI refused the 09-30 downgrade:
# its lowest supported version is 36.5.2).  The 36.5.2 bootloader package
# re-stages the same-version capsule anyway; the first boot writes it into the
# other QSPI slot and restarts once more by itself.  Afterwards firmware and
# rootfs agree again.
#
# Run --apply inside tmux (or with nohup): if the SSH session drops, apt finishes
# but the steps after it would not.  Re-running --apply is safe: the package step
# is then a no-op and the driver/boot-entry steps and checks run again.  The one
# exception: if the JetsonIO entry itself is wrong, a re-run stops before the
# package step, so fix the entry first (the --apply log names the problem).
#
# Usage (read-only modes first; only --apply changes anything):
#   l4t_upgrade_36_5_2.sh --list        the pkg=version pairs it would install
#   l4t_upgrade_36_5_2.sh --check       every version in the apt indexes? module ready?
#   l4t_upgrade_36_5_2.sh --simulate    apt-get -s: what apt would do (nothing changes)
#   sudo bash l4t_upgrade_36_5_2.sh --apply   upgrade, install imx296.ko, depmod, DEFAULT JetsonIO
#
# Written 2026-10-01 by Claude on the Jetson; NOT run yet.  Read what --check and
# --simulate print before --apply.  The initrd and boot-entry checks live in
# l4t_checks.sh (next to this script; test_l4t_checks.sh tests them).

set -euo pipefail

HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"   # also when run through a symlink
HIST="${HIST:-/var/log/apt/history.log}"   # and its rotated .1, .2.gz, ... siblings
UPGRADE_START="Start-Date: 2026-09-23  10:28:57"
NEW_RE='36\.5\.2-20260716114719|6\.2\.3\+b81'
KNEW="5.15.199-tegra"
MODULE="$HERE/imx296_driver/out-$KNEW/imx296.ko"
MODDEST="/lib/modules/$KNEW/kernel/drivers/media/i2c/imx296.ko"
EXTLINUX="/boot/extlinux/extlinux.conf"
OVERLAY="tegra234-p3767-camera-p3768-imx296-cam1.dtbo"
BACKUP_DIR="/var/backups/l4t-upgrade-$(date +%Y%m%d-%H%M%S)"

die() { echo "upgrade: $*" >&2; exit 1; }

# extlinux_default_check, extlinux_entry_check, initrd_check
[ -r "$HERE/l4t_checks.sh" ] || die "missing $HERE/l4t_checks.sh"
# shellcheck source=l4t_checks.sh
. "$HERE/l4t_checks.sh"

# pkg=newversion for every NVIDIA package the 2026-09-23 upgrade moved to R36.5.2.
pairs() {
    python3 - "$HIST" "$UPGRADE_START" "$NEW_RE" <<'EOF'
import glob, gzip, re, sys
hist, start, new_re = sys.argv[1], sys.argv[2], re.compile(sys.argv[3])
text = ""
# logrotate moves the entry on: history.log, history.log.1, history.log.2.gz ...
for f in sorted(glob.glob(hist + "*")):
    opener = gzip.open if f.endswith(".gz") else open
    try:
        with opener(f, "rt", encoding="utf-8", errors="replace") as fh:
            t = fh.read()
    except OSError:
        continue
    if start in t:
        text = t
        break
i = text.find(start)
if i < 0:
    sys.exit("upgrade: the 2026-09-23 upgrade is not in " + hist + "*")
block = text[i:text.find("End-Date:", i)]
m = re.search(r"^Upgrade: (.*)$", block, re.M)
if not m:
    sys.exit("upgrade: no Upgrade: line in that block")
out = [f"{pkg.split(':')[0]}={new}"
       for pkg, old, new in re.findall(r"([^\s,()]+) \(([^,()]+), ([^,()]+)\)", m.group(1))
       if new_re.search(new)]
if not out:
    sys.exit("upgrade: no NVIDIA packages found in that upgrade")
print("\n".join(sorted(out)))
EOF
}

module_ok() {
    [ -f "$MODULE" ] || { echo "MISSING  $MODULE (run imx296_driver/build.sh $KNEW)"; return 1; }
    local vm
    vm="$(modinfo -F vermagic "$MODULE" 2>/dev/null | awk '{print $1}')"
    [ "$vm" = "$KNEW" ] || { echo "WRONG    $MODULE is built for '$vm', not $KNEW"; return 1; }
    echo "ok       $MODULE ($KNEW)"
}

overlay_ok() {
    # The JetsonIO entry must still carry the CAM1 overlay, and every file it
    # names must exist; the kernel package resets DEFAULT to primary, but has
    # kept the JetsonIO entry so far.
    local out
    if out="$(extlinux_entry_check "$EXTLINUX" JetsonIO "$OVERLAY")"; then
        echo "ok       $out"
    else
        sed 's/^/WRONG    /' <<< "$out"; return 1
    fi
}

PAIRS=""
PAIRS_FILE="$HERE/l4t_36_5_2.pairs"   # frozen 2026-10-01 from the history; the history rotates
load_pairs() {
    if [ -r "$PAIRS_FILE" ]; then
        PAIRS="$(grep -E '^[a-z0-9.+-]+=[^ ]+$' "$PAIRS_FILE")" || die "no pairs in $PAIRS_FILE"
    else
        PAIRS="$(pairs)" || exit 1
    fi
    [ "$(wc -l <<< "$PAIRS")" -eq 65 ] || die "expected 65 packages, got $(wc -l <<< "$PAIRS")"
}

cmd_list() { load_pairs; echo "$PAIRS"; echo "($(echo "$PAIRS" | wc -l) packages)" >&2; }

cmd_check() {
    load_pairs
    local missing=0 p pkg ver
    while IFS= read -r p; do
        pkg="${p%%=*}"; ver="${p#*=}"
        if apt-cache madison "$pkg" 2>/dev/null | awk -F'|' '{gsub(/ /,"",$2); print $2}' | grep -qxF "$ver"; then
            :
        else
            echo "MISSING  $pkg $ver (not in the apt indexes; try 'sudo apt update')"; missing=$((missing + 1))
        fi
    done <<< "$PAIRS"
    echo "packages: $(echo "$PAIRS" | wc -l), unavailable: $missing"
    module_ok || missing=$((missing + 1))
    overlay_ok || missing=$((missing + 1))
    [ "$missing" -eq 0 ] || die "$missing problem(s); do not --apply"
    echo "ready"
}

# apt-get simulation; fails if apt would REMOVE anything.  The packages are
# held, so the change has to be allowed explicitly (apt then clears the holds;
# --apply sets them again).
simulate() {
    [ -n "$PAIRS" ] || load_pairs
    local out
    # shellcheck disable=SC2086
    out="$(apt-get install -s --allow-change-held-packages --no-install-recommends $PAIRS 2>&1)" || {
        echo "$out"; die "apt refuses this set (above)"; }
    # Here-strings, not pipes: grep -q exiting early must not SIGPIPE the writer
    # under pipefail.  An empty result (everything already installed) is fine.
    { grep -E '^(Inst|Remv|Conf)' <<< "$out" || true; } | awk '{print $1}' | sort | uniq -c
    if grep -q '^Remv' <<< "$out"; then
        grep '^Remv' <<< "$out"
        die "apt would REMOVE the packages above; do not --apply"
    fi
    grep -E '^Inst nvidia-l4t-(kernel|bootloader|initrd|display-kernel) ' <<< "$out" || true
    grep -qE '^Inst ' <<< "$out" || echo "(every package is already at the target version)"
    echo "simulation: upgrade only, nothing removed"
}

cmd_apply() {
    [ "$(id -u)" -eq 0 ] || die "--apply needs root (sudo bash $0 --apply)"
    if systemctl is-active --quiet dashcam-v04; then die "stop dashcam-v04 first: sudo systemctl stop dashcam-v04"; fi
    module_ok >/dev/null || die "the 5.15.199 module is not ready ($MODULE)"
    overlay_ok || die "the JetsonIO boot entry is not right (above). Nothing was changed by this run; but if an earlier --apply already ran, do NOT reboot: fix the entry first (the original extlinux.conf is in /var/backups/l4t-upgrade-*/)"
    load_pairs
    echo "== simulation"
    simulate
    echo "== backups in $BACKUP_DIR"
    mkdir -p "$BACKUP_DIR"
    cp -a "$EXTLINUX" "$BACKUP_DIR/"
    dpkg -l > "$BACKUP_DIR/dpkg-l.before.txt"
    echo "$PAIRS" > "$BACKUP_DIR/pairs.txt"
    # From here on the machine is being changed: any failure must say so.
    trap 'rc=$?; [ "$rc" -eq 0 ] || echo "== FAILED (rc=$rc): do NOT reboot or power off. Fix the error above, run sudo dpkg --configure -a, then rerun --apply. Backups: $BACKUP_DIR" >&2' EXIT
    echo "== IMX296 driver for $KNEW (placed first: no package owns it, and the kernel package's depmod then sees it)"
    install -D -m 0644 "$MODULE" "$MODDEST"
    echo "== upgrading"
    # shellcheck disable=SC2086
    apt-get install -y --allow-change-held-packages --no-install-recommends $PAIRS
    # apt clears the hold of every held package it installs (seen 2026-10-01: all 65
    # went from "hi" to "ii"), so hold them again; holds_check below verifies it
    # shellcheck disable=SC2046
    apt-mark hold $(cut -d= -f1 <<< "$PAIRS") > /dev/null
    echo "== held again: $(wc -l <<< "$PAIRS") packages"
    depmod -a "$KNEW"
    echo "== boot entry"
    sed -i 's/^DEFAULT primary$/DEFAULT JetsonIO/' "$EXTLINUX"
    echo "== checks before any reboot"
    local kver ok=1 out
    kver="$(grep -aoE -m1 'Linux version [^ ]+' /boot/Image | awk '{print $3}' || true)"
    echo "/boot/Image: ${kver:-unknown}"
    [ "$kver" = "$KNEW" ] || { echo "WARNING: /boot/Image is not $KNEW — do not reboot, report this"; ok=0; }
    for m in kernel/drivers/media/usb/uvc/uvcvideo.ko kernel/drivers/usb/class/cdc-acm.ko \
             kernel/drivers/media/i2c/imx296.ko updates/drivers/media/platform/tegra/camera/tegra-camera.ko; do
        if [ -f "/lib/modules/$KNEW/$m" ]; then echo "ok  $m"; else echo "WARNING: missing /lib/modules/$KNEW/$m — do not reboot"; ok=0; fi
    done
    if grep -q '^kernel/drivers/media/i2c/imx296.ko:' "/lib/modules/$KNEW/modules.dep"; then echo "ok  imx296 in modules.dep"; else echo "WARNING: imx296 not in modules.dep"; ok=0; fi
    if modprobe -n -q -S "$KNEW" imx296; then echo "ok  imx296 and its dependencies resolve for $KNEW"; else echo "WARNING: modprobe cannot resolve imx296 for $KNEW"; ok=0; fi
    # The root file system is on NVMe, and nvme/pcie are modules: the initrd must
    # carry them, and must itself be whole (a cut-off archive can still list them).
    if out="$(initrd_check /boot/initrd "$KNEW" "/lib/modules/$KNEW/kernel/drivers" \
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
  1. Have HDMI + a USB keyboard (or the serial console) attached for this first boot:
     there is no 5.15.185 kernel to fall back to any more.  If it hangs, pick
     "primary kernel" in the boot menu (it boots without the camera overlay).
  2. sudo reboot, on bench power.  The first boot writes the 36.5.2 bootloader
     capsule into the other QSPI slot: a few minutes and an extra automatic
     restart.  Do NOT cut power.
  3. Then:  bash ~/drive_logs/tools/verify_platform.sh
     (expects kernel 5.15.199-tegra, R36.5.2, UEFI 36.5.2, the IMX296 detected,
     csi_test 5/5, both cameras, CUDA/TensorRT).
  Going back: bash ~/drive_logs/tools/l4t_rollback_36_5_0.sh --check, --simulate; sudo systemctl stop
  dashcam-v04; sudo bash ~/drive_logs/tools/l4t_rollback_36_5_0.sh --apply (it restores DEFAULT JetsonIO
  itself; its imx296.ko for 5.15.185 stays in place).
EOT
}

case "${1:-}" in
    --list)     cmd_list ;;
    --check)    cmd_check ;;
    --simulate) simulate ;;
    --apply)    cmd_apply ;;
    *) sed -n '2,23p' "$0"; exit 1 ;;
esac
