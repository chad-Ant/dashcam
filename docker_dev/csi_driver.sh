#!/bin/bash
#
# csi_driver.sh — load the IMX296 sensor driver for the RUNNING kernel, and
# never leave nvargus-daemon stopped.
#
# Why: on 2026-09-28 the Jetson's own driver unit stopped nvargus-daemon, then
# failed to load a module built for 5.15.185-tegra into 5.15.199-tegra (an apt
# kernel upgrade), and exited with Argus still stopped: no CSI camera and no
# Argus at all.  This script checks everything it can BEFORE touching Argus,
# and restarts Argus on every exit once it has stopped it.
#
# It does not fix the mismatch itself: a module is built for one kernel.  Build
# the driver against the running kernel (the vendor's source, then
# `make -C /lib/modules/$(uname -r)/build M=$PWD modules`, install it under
# /lib/modules/$(uname -r)/ and `sudo depmod`), or boot the kernel it was built
# for.  `csi_driver.sh check` says which case you are in.
#
# Usage:
#   csi_driver.sh check   report only, changes nothing: running kernel, the
#                         module found for it (and for which other kernels),
#                         its vermagic, loaded or not, Argus's state.  Exit 0
#                         when `load` would succeed or has nothing to do.
#   csi_driver.sh load    (root) load the module if it is not loaded yet.  With
#                         Argus running: stop it, load, start it again, on
#                         every exit path.  Argus not running (at boot, when
#                         the unit is ordered before it): load only.
#
# Settings (environment; csi-driver.service sets CSI_MODULE):
#   CSI_MODULE       module name (default nv_imx296; "-" and "_" are the same)
#   CSI_MODULE_PATH  a .ko outside /lib/modules to insmod instead of modprobe
#   ARGUS_UNIT       default nvargus-daemon.service
#   KERNEL_RELEASE, MODULES_DIR, PROC_MODULES   for the host test only
#
# Exit codes: 0 ok / nothing to do, 1 usage, 2 no module for this kernel,
# 3 module built for another kernel, 4 the load failed.

set -u

MODULE="${CSI_MODULE:-nv_imx296}"
MODULE_PATH="${CSI_MODULE_PATH:-}"
ARGUS="${ARGUS_UNIT:-nvargus-daemon.service}"
KREL="${KERNEL_RELEASE:-$(uname -r)}"
MODULES_DIR="${MODULES_DIR:-/lib/modules}"
PROC_MODULES="${PROC_MODULES:-/proc/modules}"

# /proc/modules lists names with "_"; files may use either.
MODNAME="${MODULE//-/_}"
FILE_GLOB="${MODNAME//_/[-_]}"

say()  { echo "csi_driver: $*"; }
fail() { local rc="$1"; shift; echo "csi_driver: $*" >&2; exit "$rc"; }

is_loaded() { grep -q "^${MODNAME} " "$PROC_MODULES" 2>/dev/null; }

# The module files for kernel $1 (one per line), searched under MODULES_DIR.
modules_for() {
    [ -d "$MODULES_DIR/$1" ] || return 0
    find "$MODULES_DIR/$1" -type f \( -name "${FILE_GLOB}.ko" -o -name "${FILE_GLOB}.ko.*" \) 2>/dev/null | sort
}

# The kernels under MODULES_DIR that have the module, space separated.
kernels_with_module() {
    local k out=""
    for k in "$MODULES_DIR"/*/; do
        k="$(basename "$k")"
        [ -n "$(modules_for "$k")" ] && out="$out $k"
    done
    echo "${out# }"
}

vermagic_of() { modinfo -F vermagic "$1" 2>/dev/null | awk 'NR == 1 { print $1 }'; }

# Sets FILE to the module to load and returns 0, or sets WHY and returns 2 (none
# for this kernel) or 3 (built for another kernel).  Changes nothing.
FILE=""
WHY=""
resolve_module() {
    FILE=""
    WHY=""
    if [ -n "$MODULE_PATH" ]; then
        [ -f "$MODULE_PATH" ] || { WHY="CSI_MODULE_PATH $MODULE_PATH does not exist"; return 2; }
        FILE="$MODULE_PATH"
    else
        FILE="$(modules_for "$KREL" | head -n 1)"
        if [ -z "$FILE" ]; then
            local others
            others="$(kernels_with_module)"
            if [ -n "$others" ]; then
                WHY="no $MODULE for the running kernel $KREL; built for: $others."
                WHY="$WHY Rebuild it for $KREL (then depmod), or boot that kernel."
            else
                WHY="no $MODULE under $MODULES_DIR for any kernel (running $KREL)."
                WHY="$WHY Is CSI_MODULE the driver's name?"
            fi
            return 2
        fi
    fi
    local vm
    vm="$(vermagic_of "$FILE")"
    if [ "$vm" != "$KREL" ]; then
        WHY="$FILE is built for kernel '${vm:-unknown}', running $KREL: it cannot load."
        WHY="$WHY Rebuild it for $KREL."
        return 3
    fi
    return 0
}

argus_active() { systemctl is-active --quiet "$ARGUS"; }

cmd_check() {
    local rc=0
    say "running kernel: $KREL"
    say "kernels with $MODULE: $(kernels_with_module)"
    if is_loaded; then
        say "$MODNAME is loaded"
    elif resolve_module; then
        say "would load $FILE (vermagic $KREL)"
    else
        rc=$?
        say "cannot load: $WHY"
    fi
    if argus_active; then say "$ARGUS: active"; else say "$ARGUS: not active"; fi
    return "$rc"
}

ARGUS_STOPPED=0
restart_argus() {
    [ "$ARGUS_STOPPED" = 1 ] || return 0
    ARGUS_STOPPED=0
    if systemctl start "$ARGUS"; then
        say "$ARGUS started again"
    else
        echo "csi_driver: could not start $ARGUS again: systemctl start $ARGUS" >&2
    fi
}

cmd_load() {
    if is_loaded; then
        say "$MODNAME already loaded; nothing to do"
        return 0
    fi
    # Every check first: a refusal here leaves Argus exactly as it was.
    resolve_module || fail $? "$WHY (Argus left as it was)"

    trap restart_argus EXIT
    trap 'exit 130' INT TERM HUP        # so the EXIT trap runs on a signal too
    if argus_active; then
        say "stopping $ARGUS to load $MODNAME"
        ARGUS_STOPPED=1                 # whatever stop returns: start it on exit
        systemctl stop "$ARGUS" || fail 4 "could not stop $ARGUS; $MODNAME not loaded"
    fi

    if [ -n "$MODULE_PATH" ]; then
        insmod "$FILE" || fail 4 "insmod $FILE failed (see dmesg)"
    else
        modprobe "$MODULE" || fail 4 "modprobe $MODULE failed (see dmesg; after installing it, run depmod)"
    fi
    is_loaded || fail 4 "$MODNAME not in $PROC_MODULES after loading it"
    say "loaded $FILE"
    return 0                            # the EXIT trap restarts Argus
}

case "${1:-}" in
    check) cmd_check ;;
    load)  cmd_load ;;
    *)     echo "usage: $0 check|load" >&2; exit 1 ;;
esac
