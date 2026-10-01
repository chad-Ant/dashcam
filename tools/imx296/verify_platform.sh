#!/bin/bash
#
# verify_platform.sh — check this Jetson after an L4T change (upgrade/rollback):
# kernel and release, bootloader, the IMX296 driver and overlay, Argus, both
# cameras, then the camera/GPU tests in the dev container.
#
# Changes nothing: no sudo, no service is stopped or started.  usb_test needs the
# UGREEN free, so it is skipped while dashcam-v04 is running (stop it first if you
# want it: sudo systemctl stop dashcam-v04, then start it again afterwards).
#
# Usage:  bash verify_platform.sh [--quick]     --quick: platform checks only, no tests
# Output: a PASS/FAIL/SKIP line per check, a summary, and every log under
#         ~/drive_logs/platform_checks/<timestamp>/
#
# Written 2026-10-01 by Claude.

set -uo pipefail

QUICK=0; [ "${1:-}" = "--quick" ] && QUICK=1
HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"   # also when run through a symlink
# extlinux_default_check, extlinux_entry_check (shared with the upgrade/rollback scripts)
. "$HERE/l4t_checks.sh" || { echo "verify_platform: cannot load $HERE/l4t_checks.sh"; exit 1; }
OVERLAY="${OVERLAY:-tegra234-p3767-camera-p3768-imx296-cam1.dtbo}"
REPO="${REPO:-/home/jetson/dashcam}"
IMAGE="${IMAGE:-l4t-ml-gpio}"
CNAME="verify_platform_$$"
UGREEN_BYID="/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0"
LOGDIR="/home/jetson/drive_logs/platform_checks/$(date +%Y%m%d-%H%M%S)"
mkdir -p "$LOGDIR"

PASS=0; FAIL=0; SKIP=0
pass() { echo "PASS  $*"; PASS=$((PASS + 1)); }
fail() { echo "FAIL  $*"; FAIL=$((FAIL + 1)); }
skip() { echo "SKIP  $*"; SKIP=$((SKIP + 1)); }
info() { echo "      $*"; }
exec > >(tee "$LOGDIR/summary.txt") 2>&1

echo "== platform ($(date '+%F %T'), logs in $LOGDIR)"
KREL="$(uname -r)"
REL="$(head -1 /etc/nv_tegra_release | sed -E 's/.*REVISION: ([0-9.]+),.*/36.\1/')"
BIOS="$(cat /sys/class/dmi/id/bios_version 2>/dev/null | cut -d- -f1)"
JP="$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null)"
info "kernel $KREL, L4T R$REL, JetPack $JP, UEFI $BIOS"
case "$KREL/$REL" in
    5.15.185-tegra/36.5.0|5.15.199-tegra/36.5.2) pass "kernel and L4T release belong together ($KREL, R$REL)" ;;
    *) fail "kernel $KREL does not belong to L4T R$REL" ;;
esac
if [ "$BIOS" = "$REL" ]; then pass "bootloader/UEFI $BIOS matches the rootfs"
else info "bootloader/UEFI $BIOS, rootfs R$REL (a mismatch; 36.5.2 UEFI on R36.5.0 was tested and works)"; fi
# The 65 packages of this release, by name (the frozen lists next to this script).
PAIRS_FILE="$HERE/l4t_${REL//./_}.pairs"
if [ ! -r "$PAIRS_FILE" ]; then
    fail "no package list for R$REL ($PAIRS_FILE)"
else
    if OUT="$(holds_check "$PAIRS_FILE")"; then pass "$OUT"; else fail "$OUT"; fi
    if OUT="$(versions_check "$PAIRS_FILE")"; then pass "$OUT (R$REL)"; else fail "$OUT"; fi
fi
# Installed nvidia-l4t packages that carry an L4T version (36.x.y-... or 5.15.x-tegra-...)
# but not this release's; packages with their own scheme (cudadebuggingsupport) are skipped.
LEFT="$(dpkg-query -W -f='${db:Status-Abbrev} ${Version}\n' 'nvidia-l4t-*' 2>/dev/null \
        | awk '$1=="ii" && $2 ~ /^(36\.[0-9]+\.[0-9]+-|5\.15\.[0-9]+-tegra-)/ {print $2}' \
        | grep -vcE "^(${REL//./\\.}-|${KREL}-${REL//./\\.}-)" || true)"
[ "$LEFT" -eq 0 ] && pass "every nvidia-l4t package is at R$REL" || fail "$LEFT nvidia-l4t packages are not at R$REL"

echo "== boot entry and IMX296 driver"
if OUT="$(extlinux_default_check /boot/extlinux/extlinux.conf JetsonIO)"; then pass "extlinux $OUT"
else fail "extlinux: ${OUT//$'\n'/; } (the camera overlay is not booted)"; fi
if OUT="$(extlinux_entry_check /boot/extlinux/extlinux.conf JetsonIO "$OVERLAY")"; then pass "$OUT"
else fail "${OUT//$'\n'/; }"; fi
KO="/lib/modules/$KREL/kernel/drivers/media/i2c/imx296.ko"
if [ -f "$KO" ]; then
    VM="$(modinfo -F vermagic "$KO" | awk '{print $1}')"
    [ "$VM" = "$KREL" ] && pass "imx296.ko built for $KREL" || fail "imx296.ko is built for $VM, running $KREL"
else
    fail "no imx296.ko for $KREL (copy imx296_driver/out-$KREL/imx296.ko there, then depmod -a $KREL)"
fi
grep -q '^imx296 ' /proc/modules && pass "imx296 module loaded" || fail "imx296 module not loaded"
journalctl -k -b --no-pager 2>/dev/null > "$LOGDIR/kernel.log"
DET="$(grep -E 'imx296 [0-9]+-001a: Detected IMX296' "$LOGDIR/kernel.log" | tail -1 | sed -E 's/.*(imx296 [0-9]+-001a: Detected [^ ]+ \([A-Za-z]+\)).*/\1/')"
[ -n "$DET" ] && pass "sensor probed: $DET" || fail "no 'Detected IMX296' in this boot's kernel log"
BAD="$(grep -cE 'imx296 .*(failed: -121|request timed out)|uncorr_err' "$LOGDIR/kernel.log" || true)"
[ "$BAD" -eq 0 ] && pass "no sensor I2C or capture errors in the kernel log" || info "kernel log has $BAD sensor/capture error lines (see kernel.log)"
for u in nvargus-daemon imx296-reload; do
    systemctl is-active --quiet "$u" && pass "$u active" || fail "$u not active"
done

echo "== cameras"
v4l2-ctl --list-devices > "$LOGDIR/v4l2-devices.txt" 2>&1
CSI="$(awk '/vi-output, imx296/{getline; print $1; exit}' "$LOGDIR/v4l2-devices.txt")"
[ -n "$CSI" ] && pass "IMX296 video node: $CSI" || fail "no vi-output imx296 node"
if [ -e "$UGREEN_BYID" ]; then pass "UGREEN present: $(readlink -f "$UGREEN_BYID")"; else fail "UGREEN not found at its by-id path"; fi
V04="$(systemctl is-active dashcam-v04 2>/dev/null)"
info "dashcam-v04: $V04"
if [ "$V04" = "active" ]; then
    journalctl -u dashcam-v04 -b --no-pager > "$LOGDIR/dashcam-v04.log" 2>&1
    N="$(grep -c 'segment opened' "$LOGDIR/dashcam-v04.log" || true)"
    [ "$N" -ge 1 ] && pass "v0.4 is recording ($N segment(s) this boot)" || info "v0.4 active, no segment opened yet"
fi

if [ "$QUICK" -eq 1 ]; then
    echo "== tests skipped (--quick)"
else
    echo "== tests in the dev container ($IMAGE)"
    BUILD="$(ls -td "$REPO"/bin/build_* 2>/dev/null | head -1)"
    if [ -z "$BUILD" ]; then
        skip "no build under $REPO/bin (run make in the container first)"
    else
        B="bin/$(basename "$BUILD")"
        info "build: $B"
        docker run -d --rm --name "$CNAME" --network=host --privileged --ipc=host --runtime nvidia -w /user/dashcam \
            -v /dev:/dev -v /tmp/argus_socket:/tmp/argus_socket -v "$REPO":/user/dashcam \
            -v /etc/localtime:/etc/localtime:ro "$IMAGE" sleep infinity > /dev/null \
            || { fail "cannot start the dev container"; CNAME=""; }
        if [ -n "$CNAME" ]; then
            trap 'docker stop -t 3 "$CNAME" > /dev/null 2>&1' EXIT
            run() {   # run <name> <timeout s> <command...>; PASS on exit 0
                local name="$1" t="$2"; shift 2
                docker exec "$CNAME" bash -c "cd /user/dashcam && timeout $t $*" > "$LOGDIR/$name.log" 2>&1
                local rc=$?
                local res; res="$(grep -aE 'RESULT|tests passed|PASSED|FAILED' "$LOGDIR/$name.log" | tail -1)"
                if [ "$rc" -eq 0 ]; then pass "$name ${res:+— $res}"; else fail "$name (rc $rc) ${res:+— $res} — see $name.log"; fi
            }
            if [ -n "$CSI" ]; then run csi_test 240 "$B/csi_test 0"; else skip "csi_test (no IMX296 node)"; fi
            run csi_rtp_test 180 "$B/csi_rtp_test"
            run camera_gst_test 300 "$B/camera_gst_test"
            grep -q 'CSI: all .* entries' "$LOGDIR/camera_gst_test.log" && pass "camera_gst_test checked the CSI dictionary against live Argus" \
                || info "camera_gst_test did not run the CSI dictionary check"
            run scan_cameras 60 "$B/scan_cameras"
            NCAM="$(grep -oE 'Found [0-9]+ valid camera' "$LOGDIR/scan_cameras.log" | grep -oE '[0-9]+')"
            [ "${NCAM:-0}" -ge 2 ] && pass "scan_cameras found $NCAM cameras" || fail "scan_cameras found ${NCAM:-0} camera(s), expected 2"
            if [ "$V04" = "active" ]; then
                skip "usb_test (dashcam-v04 holds the UGREEN)"
            else
                v4l2-ctl -d "$UGREEN_BYID" --list-ctrls > "$LOGDIR/ugreen-ctrls-before.txt" 2>&1
                run usb_test 300 "$B/usb_test"
                v4l2-ctl -d "$UGREEN_BYID" --list-ctrls > "$LOGDIR/ugreen-ctrls-after.txt" 2>&1
                cmp -s "$LOGDIR/ugreen-ctrls-before.txt" "$LOGDIR/ugreen-ctrls-after.txt" \
                    && pass "UGREEN controls unchanged by usb_test" || fail "usb_test changed the UGREEN controls (see the two ctrls files)"
            fi
            run cuda 120 "python3 -c \"import torch; assert torch.cuda.is_available(); x=torch.randn(512,512,device='cuda'); print('cuda ok', torch.cuda.get_device_name(0), float((x@x).sum())!=0)\""
            if [ -f "$REPO/models/culane_res18_fp16.engine" ]; then
                run tensorrt 180 "/usr/src/tensorrt/bin/trtexec --loadEngine=models/culane_res18_fp16.engine --iterations=50 --warmUp=500"
                info "$(grep -oE 'GPU Compute Time: min = [0-9.]+ ms, max = [0-9.]+ ms, mean = [0-9.]+ ms' "$LOGDIR/tensorrt.log" | head -1)"
            else
                skip "tensorrt (no models/culane_res18_fp16.engine)"
            fi
        fi
    fi
fi

echo
echo "== summary: $PASS passed, $FAIL failed, $SKIP skipped  (logs: $LOGDIR)"
[ "$FAIL" -eq 0 ]
