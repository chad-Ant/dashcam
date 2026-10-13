#!/bin/bash
#
# ch341_module.sh — the CH340/CH341 USB-serial driver (ch341.ko) for the
# Jetson's kernel.  The CYD uploads and logs over a CH340 (USB 1a86:7523).
# The Tegra kernel does not ship this driver (CONFIG_USB_SERIAL_CH341 is not set
# in 5.15.199-tegra): the board shows in lsusb, but no /dev/ttyUSB* appears.
#
#   ch341_module.sh check           report only, changes nothing: the driver
#                                   for the running kernel (on disk, loaded),
#                                   each CH340 and its tty, and whether brltty
#                                   is set to grab it.  Exit 0 when the driver
#                                   is on disk, every CH340 plugged in has a
#                                   tty and brltty is kept off 1a86:7523.
#   ch341_module.sh build [OUTDIR]  build ch341.ko for the running kernel from
#                                   the kernel.org stable source of the same
#                                   version, check vermagic and every imported
#                                   symbol CRC, then print the install commands.
#                                   OUTDIR default: ~/drive_logs/tools/ch341_driver/out-<kernel>
#
# The install needs root, so this script never installs anything: it prints
# the commands.  Run it on the host, not in the container: it builds against
# the host's /lib/modules/<kernel>/build headers.
#
# Source: drivers/usb/serial/ch341.c at stable tag v<kernel version>.  The
# sha256 must match a pinned value below, or the build refuses (set
# CH341_ACCEPT_SHA256=<sha> after reading the file to accept another).  For
# 5.15.199 it is byte-identical to the copy in NVIDIA's R36.4.3, R36.4.4 and
# R36.5 public_sources (kernel-jammy-src), checked 2026-10-12.
# CH341_SRC=/path/to/ch341.c builds from a local file instead of downloading.
#
# Exit codes: 0 ok, 1 usage or build failure, 2 check found a problem,
# 3 refused (unpinned source, CRC mismatch, wrong vermagic).
#
# Install (root), as `build` prints it:
#   sudo systemctl mask --now brltty-udev.service
#   sudo install -D -m 0644 OUTDIR/ch341.ko /lib/modules/<kernel>/updates/ch341.ko
#   sudo depmod -a <kernel> && sudo modprobe ch341
# Undo (unload first: once depmod has forgotten it, modprobe -r cannot find it):
#   sudo modprobe -r ch341       # fails while the port is open (picocom)
#   sudo rm /lib/modules/<kernel>/updates/ch341.ko && sudo depmod -a <kernel>
#   sudo systemctl unmask brltty-udev.service
set -euo pipefail

KVER="$(uname -r)"
KBASE="${KVER%%-*}"                                  # 5.15.199-tegra -> 5.15.199
KDIR="/lib/modules/$KVER/build"
declare -A PINNED=(
    [5.15.199]=f66d070eab6235b8a5c7a06a283d2feecb8fa3d81bc1323c847c2d2cbf7bd410
)
URL="https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/usb/serial/ch341.c?h=v$KBASE"
IDS="1a86:7523 1a86:7522 1a86:5523"                  # the CH34x IDs ch341.c claims that a CYD can carry

say() { echo "ch341_module: $*"; }
die() { echo "ch341_module: $1" >&2; exit "${2:-1}"; }

# USB devices with a CH34x ID, one sysfs path per line.
ch34x_devices() {
    local d id
    for d in /sys/bus/usb/devices/*; do
        [ -f "$d/idVendor" ] || continue
        id="$(cat "$d/idVendor"):$(cat "$d/idProduct")"
        case " $IDS " in *" $id "*) echo "$d" ;; esac
    done
}

do_check() {
    local bad=0 path loaded=no d tty by l
    say "kernel $KVER"
    if path="$(modinfo -n ch341 2>/dev/null)"; then
        say "driver on disk: $path (vermagic $(modinfo -F vermagic ch341))"
    else
        say "driver on disk: NONE for $KVER (build it: $0 build)"; bad=1
    fi
    grep -q '^ch341 ' /proc/modules && loaded=yes
    say "driver loaded: $loaded"
    local n=0
    while read -r d; do
        [ -n "$d" ] || continue
        n=$((n + 1))
        tty="$(ls -d "$d"/*/ttyUSB* 2>/dev/null | head -1 || true)"
        if [ -n "$tty" ]; then
            tty="/dev/$(basename "$tty")"
            by=""
            for l in /dev/serial/by-id/*; do
                [ -e "$l" ] && [ "$(readlink -f "$l")" = "$tty" ] && { by="$l"; break; }
            done
            say "CH34x at $(basename "$d") ($(cat "$d/idVendor"):$(cat "$d/idProduct")): $tty ${by:+($by)}"
        else
            say "CH34x at $(basename "$d") ($(cat "$d/idVendor"):$(cat "$d/idProduct")): NO tty"; bad=1
        fi
    done < <(ch34x_devices)
    [ "$n" -gt 0 ] || say "no CH34x device plugged in"
    # Ubuntu's brltty claims 1a86:7523 (an NLS eReader braille display uses that
    # chip) through usbfs, which takes the interface away from ch341.  A rules
    # file of the same name in /etc replaces the packaged one; only active
    # (uncommented) rule lines count.
    local rules=/usr/lib/udev/rules.d/85-brltty.rules unit
    [ -e /etc/udev/rules.d/85-brltty.rules ] && rules=/etc/udev/rules.d/85-brltty.rules
    unit="$(systemctl is-enabled brltty-udev.service 2>/dev/null || true)"
    if grep -qs '^[^#]*1a86/7523' "$rules" && [ "$unit" != masked ] && [ "$unit" != masked-runtime ]; then
        say "brltty-udev.service is NOT masked: brltty will grab a 1a86:7523 device ($(systemctl is-active brltty-udev.service 2>/dev/null || true) now)"
        say "  fix: sudo systemctl mask --now brltty-udev.service   (then replug the CYD)"
        bad=1
    fi
    [ "$bad" = 0 ] && say "OK" || say "install and undo commands: the header of $0, or peripherals/cyd/README.md"
    return $((bad * 2))
}

do_build() {
    local out="${1:-$HOME/drive_logs/tools/ch341_driver/out-$KVER}"
    [ -d "$KDIR" ] || die "no kernel headers at $KDIR"
    command -v make >/dev/null && command -v gcc >/dev/null || die "make and gcc are needed (host, not container)"
    WORK="$(mktemp -d)"; trap 'rm -rf "$WORK"' EXIT
    local work="$WORK"
    if [ -n "${CH341_SRC:-}" ]; then
        cp "$CH341_SRC" "$work/ch341.c"
        say "source: $CH341_SRC"
    else
        command -v curl >/dev/null || die "curl is needed to fetch the source (or set CH341_SRC)"
        curl -fsSL -o "$work/ch341.c" "$URL" || die "download failed: $URL"
        say "source: $URL"
    fi
    local sha; sha="$(sha256sum "$work/ch341.c" | cut -d' ' -f1)"
    if [ "$sha" != "${PINNED[$KBASE]:-}" ] && [ "$sha" != "${CH341_ACCEPT_SHA256:-}" ]; then
        die "source sha256 $sha is not the pinned one for $KBASE (${PINNED[$KBASE]:-none}); read it, then set CH341_ACCEPT_SHA256=$sha" 3
    fi
    echo 'obj-m := ch341.o' > "$work/Makefile"
    make -C "$KDIR" M="$work" modules > "$work/build.log" 2>&1 || { cat "$work/build.log"; die "build failed"; }

    local vm; vm="$(modinfo -F vermagic "$work/ch341.ko")"
    case "$vm" in "$KVER "*) ;; *) die "vermagic '$vm' is not for $KVER" 3 ;; esac
    # Every symbol the module imports must have the CRC the running kernel exports.
    local bad=0 n=0 crc sym want
    while read -r crc sym; do
        n=$((n + 1))
        want="$(awk -v s="$sym" '$2 == s {print $1; exit}' "$KDIR/Module.symvers")"
        if [ -z "$want" ] || [ "$((crc))" != "$((want))" ]; then
            echo "CRC MISMATCH: $sym (module $crc, kernel ${want:-missing})"; bad=$((bad + 1))
        fi
    done < <(/sbin/modprobe --dump-modversions "$work/ch341.ko")
    [ "$bad" -eq 0 ] || die "$bad of $n symbol CRCs do not match $KVER: do not install" 3
    [ "$n" -gt 0 ] || die "no modversions in the module" 3

    mkdir -p "$out"
    cp "$work/ch341.ko" "$work/ch341.c" "$work/build.log" "$out/"
    ( cd "$out" && sha256sum ch341.c ch341.ko > SHA256SUMS )
    say "built $out/ch341.ko: vermagic $vm, all $n symbol CRCs match, depends $(modinfo -F depends "$out/ch341.ko")"
    cat <<EOF

Install (root; the module is unsigned, so the kernel logs one taint line):
  sudo systemctl mask --now brltty-udev.service      # brltty otherwise grabs 1a86:7523
  sudo install -D -m 0644 $out/ch341.ko /lib/modules/$KVER/updates/ch341.ko
  sudo depmod -a $KVER
  sudo modprobe ch341                                # binds a CYD already plugged in
  $0 check                                           # expect a /dev/ttyUSB* per CH34x
Undo (unload first: after depmod forgets it, modprobe -r cannot find it):
  sudo modprobe -r ch341                             # fails while the port is open (picocom)
  sudo rm /lib/modules/$KVER/updates/ch341.ko && sudo depmod -a $KVER
  sudo systemctl unmask brltty-udev.service
A new kernel needs a new build (the module is tied to $KVER).
EOF
}

case "${1:-}" in
    check) do_check ;;
    build) do_build "${2:-}" ;;
    *) echo "usage: $0 check | build [OUTDIR]"; exit 1 ;;
esac
