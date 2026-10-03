#!/bin/bash
#
# csi_driver.sh — (re)load the IMX296 sensor driver for the RUNNING kernel, and
# never leave nvargus-daemon stopped.
#
# Why: the vendor's unit, imx296-reload.service (a copy is in
# docker_dev/imx296-reload.service.jetson), stops nvargus-daemon, runs
# `modprobe -r imx296` then `modprobe imx296`, and starts Argus again.  It checks
# nothing.  On kernel 5.15.199-tegra (an apt upgrade), which then had no imx296
# at all, its `modprobe -r` failed ("Module imx296 not found") and it exited with
# Argus still stopped: no CSI camera and no Argus (found 2026-09-28).  It never
# tried to load the 5.15.185 module.  On a kernel that has the module (5.15.185,
# and 5.15.199 since the 2026-10-01 rebuild) it works.  This script checks
# everything it can BEFORE touching Argus or the loaded module, and starts Argus
# again on every exit once it has stopped it.
#
# It does not fix a missing module: a module is built for one kernel.  Build
# the driver against the running kernel (the vendor's source, then
# `make -C /lib/modules/$(uname -r)/build M=$PWD modules`, install it under
# /lib/modules/$(uname -r)/ and `sudo depmod`), or boot the kernel it was built
# for.  `csi_driver.sh check` says which case you are in.
#
# Usage:
#   csi_driver.sh check   report only, changes nothing: running kernel, the file
#                         modprobe would load for it (depmod's choice) and its
#                         vermagic, module files depmod does not know, the other
#                         kernels that have the module, loaded or not, Argus's
#                         state.  Exit 0 when `reload` (and so `load`) would
#                         succeed.
#   csi_driver.sh reload  (root) the vendor unit's job, checked first: wait
#                         CSI_RELOAD_DELAY s, stop Argus if it runs, unload the
#                         module if it is loaded, load it, start Argus again.
#                         A module that cannot load is refused before anything
#                         is touched.  csi-driver.service runs this after
#                         nvargus-daemon has started: Argus started before the
#                         sensor was ready gives a black screen.
#   csi_driver.sh load    (root) load the module if it is not loaded yet, with
#                         Argus stopped around the load if it runs.
#
# Argus is started again only if this script stopped it.  Under systemd
# ($INVOCATION_ID is set) that start is queued with --no-block and then watched
# for up to ARGUS_START_WAIT s: a blocking start from inside a unit waits for
# ever if Argus's start job is ordered after that unit (HANDOVER item L).
#
# Settings (environment; csi-driver.service sets CSI_MODULE and CSI_RELOAD_DELAY):
#   CSI_MODULE        module name (default imx296; "-" and "_" are the same)
#   CSI_MODULE_PATH   a .ko outside /lib/modules to insmod (rmmod on reload)
#                     instead of modprobe
#   CSI_RELOAD_DELAY  whole seconds `reload` waits after its checks (default 0)
#   ARGUS_UNIT        default nvargus-daemon.service
#   ARGUS_START_WAIT  whole seconds to watch a queued Argus start (default 20)
#   KERNEL_RELEASE, MODULES_DIR, PROC_MODULES   for the host test only (modinfo
#                     itself always reads /lib/modules)
#
# Exit codes: 0 ok / nothing to do, 1 usage, 2 no module for this kernel (or
# one depmod does not know), 3 module built for another kernel, 4 stopping
# Argus, the unload or the load failed, 5 Argus was stopped and is not running
# again (this wins over 0 and 4: the messages say what else happened).

set -u

MODULE="${CSI_MODULE:-imx296}"
MODULE_PATH="${CSI_MODULE_PATH:-}"
RELOAD_DELAY="${CSI_RELOAD_DELAY:-0}"
ARGUS="${ARGUS_UNIT:-nvargus-daemon.service}"
ARGUS_START_WAIT="${ARGUS_START_WAIT:-20}"
KREL="${KERNEL_RELEASE:-$(uname -r)}"
MODULES_DIR="${MODULES_DIR:-/lib/modules}"
PROC_MODULES="${PROC_MODULES:-/proc/modules}"
VENDOR_UNIT=imx296-reload.service

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

# The module files for the running kernel that modules.dep does not list, one
# per line: "new FILE" when FILE arrived after the last depmod, "old FILE" when
# depmod ran after it and preferred another.  Judged by FILE's ctime, not its
# mtime: cp -p, install -p and tar keep an old mtime, never an old ctime.
depmod_unknown() {
    local dep="$MODULES_DIR/$KREL/modules.dep" f rel
    while IFS= read -r f; do
        [ -n "$f" ] || continue
        rel="${f#"$MODULES_DIR/$KREL/"}"
        if [ -f "$dep" ] && awk -F: -v r="$rel" -v a="$f" '$1 == r || $1 == a { k = 1 } END { exit !k }' "$dep"; then
            continue
        fi
        if [ ! -f "$dep" ] || [ -n "$(find "$f" -cnewer "$dep" 2>/dev/null)" ]; then
            echo "new $f"
        else
            echo "old $f"
        fi
    done <<< "$(modules_for "$KREL")"
}

# Sets FILE to the module to load and returns 0, or sets WHY and returns 2 (none
# for this kernel, or one depmod does not know) or 3 (built for another kernel).
# NOTE gets what is worth saying either way.  Changes nothing.
FILE=""
WHY=""
NOTE=""
resolve_module() {
    FILE=""
    WHY=""
    NOTE=""
    if [ -n "$MODULE_PATH" ]; then
        [ -f "$MODULE_PATH" ] || { WHY="CSI_MODULE_PATH $MODULE_PATH does not exist"; return 2; }
        FILE="$MODULE_PATH"
    else
        # The file modprobe loads is the one modules.dep names (depmod's choice:
        # updates/ before kernel/), not the first file by name.
        FILE="$(modinfo -k "$KREL" -F filename "$MODULE" 2>/dev/null | head -n 1)"
        local unknown new old
        unknown="$(depmod_unknown)"
        new="$(sed -n 's/^new //p' <<< "$unknown" | tr '\n' ' ')"
        old="$(sed -n 's/^old //p' <<< "$unknown" | tr '\n' ' ')"
        if [ -z "$FILE" ]; then
            if [ -n "$unknown" ]; then
                WHY="${new}${old}is installed for $KREL, but depmod does not know it, so modprobe"
                WHY="$WHY cannot load it.  Run: sudo depmod $KREL"
                return 2
            fi
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
        if [ "$FILE" = "(builtin)" ]; then
            WHY="$MODULE is built into kernel $KREL: there is no module to load or reload"
            return 2
        fi
        if [ ! -f "$FILE" ]; then
            WHY="modules.dep names $FILE, which does not exist.  Run: sudo depmod $KREL"
            return 2
        fi
        [ -n "$new" ] && NOTE="${new}was installed after the last depmod; modprobe loads $FILE instead.  Run: sudo depmod $KREL."
        [ -n "$old" ] && NOTE="${NOTE:+$NOTE  }${old}is not used: depmod prefers $FILE."
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

argus_state() { systemctl is-active "$ARGUS" 2>/dev/null; }
# Running, or about to: stop it before the module goes away under it.
argus_running() { case "$(argus_state)" in active|activating|reloading) return 0 ;; esac; return 1; }

cmd_check() {
    local rc=0
    say "running kernel: $KREL"
    say "kernels with $MODULE: $(kernels_with_module)"
    if resolve_module; then
        say "would load $FILE (vermagic $KREL)"
    else
        rc=$?
        say "cannot load: $WHY"
    fi
    [ -n "$NOTE" ] && say "note: $NOTE"
    if ! is_loaded; then
        say "$MODNAME is not loaded"
    elif [ "$rc" = 0 ]; then
        say "$MODNAME is loaded"
    else
        say "$MODNAME is loaded; reload would refuse and leave it loaded"
    fi
    say "$ARGUS: $(argus_state || true)"
    vendor_note
    return "$rc"
}

# Both units reload the sensor driver after Argus starts, and race each other.
vendor_note() {
    if [ "$(systemctl is-enabled "$VENDOR_UNIT" 2>/dev/null)" = enabled ]; then
        say "note: $VENDOR_UNIT is enabled; with csi-driver.service, disable it" \
            "(sudo systemctl disable $VENDOR_UNIT): both reload the sensor driver"
    fi
}

ARGUS_STOPPED=0

# Starts Argus again if this script stopped it; returns 1 if it is not running.
restart_argus() {
    [ "$ARGUS_STOPPED" = 1 ] || return 0
    ARGUS_STOPPED=0
    if [ -z "${INVOCATION_ID:-}" ]; then
        if systemctl start "$ARGUS"; then
            say "$ARGUS started again"
            return 0
        fi
        echo "csi_driver: could not start $ARGUS again, so Argus is down: sudo systemctl start $ARGUS" >&2
        return 1
    fi
    # Under systemd: a blocking start would wait for ever on a job ordered after
    # this unit (HANDOVER item L).  Queue it, then watch it.
    if ! systemctl --no-block start "$ARGUS"; then
        echo "csi_driver: could not queue a start of $ARGUS, so Argus is down: sudo systemctl start $ARGUS" >&2
        return 1
    fi
    local i state=""
    for ((i = 0; i < ARGUS_START_WAIT * 4; i++)); do
        state="$(argus_state)"
        if [ "$state" = active ]; then
            say "$ARGUS started again"
            return 0
        fi
        sleep 0.25
    done
    echo "csi_driver: $ARGUS is '${state:-unknown}', not active, ${ARGUS_START_WAIT} s after its start was queued:" \
         "journalctl -u $ARGUS; sudo systemctl start $ARGUS" >&2
    return 1
}

on_exit() {
    local rc=$?
    restart_argus || rc=5
    exit "$rc"
}

guard_argus() {
    trap on_exit EXIT
    trap 'exit 130' INT TERM HUP        # so the EXIT trap runs on a signal too
}

stop_argus() {
    argus_running || return 0
    say "stopping $ARGUS to $1 $MODNAME"
    ARGUS_STOPPED=1                     # whatever stop returns: start it on exit
    systemctl stop "$ARGUS" || fail 4 "could not stop $ARGUS; $MODNAME not touched"
}

load_module() {
    if [ -n "$MODULE_PATH" ]; then
        insmod "$FILE" || fail 4 "insmod $FILE failed (see dmesg)"
    else
        modprobe "$MODULE" || fail 4 "modprobe $MODULE failed (see dmesg; after installing it, run depmod)"
    fi
    is_loaded || fail 4 "$MODNAME not in $PROC_MODULES after loading it"
}

cmd_load() {
    if is_loaded; then
        say "$MODNAME already loaded; nothing to do"
        return 0
    fi
    # Every check first: a refusal here leaves Argus exactly as it was.
    resolve_module || fail $? "$WHY (Argus left as it was)"
    [ -n "$NOTE" ] && say "note: $NOTE"

    guard_argus
    stop_argus load
    load_module
    say "loaded $FILE"
    return 0                            # the EXIT trap starts Argus again
}

# A failed unload may have removed the module all the same (another reload got
# there first): /proc/modules decides whether the loaded copy stays.
unload_failed() {
    is_loaded && fail 4 "$1 failed (in use?); the loaded copy stays"
    say "$1 failed, but $MODNAME is no longer loaded; loading it"
}

cmd_reload() {
    # Every check first: a refusal here leaves Argus and the loaded module as
    # they were.  The vendor unit unloads without checking anything.
    resolve_module || fail $? "$WHY (Argus and the loaded $MODNAME left as they were)"
    [ -n "$NOTE" ] && say "note: $NOTE"
    vendor_note

    guard_argus
    if [ "$RELOAD_DELAY" -gt 0 ]; then
        say "waiting ${RELOAD_DELAY} s before the reload"
        sleep "$RELOAD_DELAY"
    fi
    stop_argus reload
    if is_loaded; then
        if [ -n "$MODULE_PATH" ]; then
            rmmod "$MODNAME" || unload_failed "rmmod $MODNAME"
        else
            modprobe -r "$MODULE" || unload_failed "modprobe -r $MODULE"
        fi
        is_loaded && fail 4 "$MODNAME still in $PROC_MODULES after unloading it"
    fi
    load_module
    say "reloaded $FILE"
    return 0                            # the EXIT trap starts Argus again
}

uint() { [[ "$1" =~ ^[0-9]+$ ]]; }
uint "$RELOAD_DELAY" || fail 1 "CSI_RELOAD_DELAY must be whole seconds, not '$RELOAD_DELAY'"
uint "$ARGUS_START_WAIT" && [ "$ARGUS_START_WAIT" -gt 0 ] \
    || fail 1 "ARGUS_START_WAIT must be whole seconds above 0, not '$ARGUS_START_WAIT'"
RELOAD_DELAY=$((10#$RELOAD_DELAY)) ARGUS_START_WAIT=$((10#$ARGUS_START_WAIT))

case "${1:-}" in
    check)  cmd_check ;;
    reload) cmd_reload ;;
    load)   cmd_load ;;
    *)      echo "usage: $0 check|reload|load" >&2; exit 1 ;;
esac
