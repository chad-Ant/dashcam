#!/bin/bash
#
# test_csi_driver.sh — host test for csi_driver.sh, no Jetson needed.
#
# systemctl, modprobe, insmod, rmmod, modinfo and sleep are stubs on PATH that
# log every call and act on a fake /lib/modules tree (with a fake modules.dep,
# which the modinfo stub reads as the real one does) and a fake /proc/modules;
# nothing on the machine is touched.  Checks that Argus is never stopped when
# the load cannot work (the 2026-09-28 kernel mismatch), is always started again
# once it was stopped, and that a failed start is reported.  The sections
# marked K..P are the HANDOVER items of the second review; each has checks that
# fail on aa9deb1's script.
#
# Usage: docker_dev/test_csi_driver.sh      (prints RESULT: PASS / FAIL)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="$HERE/csi_driver.sh"
UNIT="$HERE/csi-driver.service"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
FAILS=0
KREL=5.15.199-tegra

check() {   # check <condition-exit-code> <what>
    if [ "$1" -eq 0 ]; then echo "  ok    $2"; else echo "  FAIL  $2"; FAILS=$((FAILS + 1)); fi
}

mkdir -p "$T/bin"
# systemctl [--no-block] is-active|is-enabled|stop|start UNIT.  Argus's state is
# in $T/argus (active|activating|inactive|failed...).  Flags in $STUB_DIR:
# stop_fails, stop_hangs (the stop never returns: Argus is left deactivating,
# the stub writes the script's pid, its process group, to $STUB_DIR/stopping
# and sleeps until killed), start_fails (Argus fails to start: a blocking start
# exits 1, a queued one ends "failed"), queue_fails (a queued start is refused),
# argus_waits (Argus's start job is ordered after the calling unit, as with
# aa9deb1's Before=: from inside a unit a blocking start never returns, a
# queued one stays queued), vendor_enabled (for is-enabled).
cat > "$T/bin/systemctl" <<'EOF'
#!/bin/bash
echo "systemctl $*" >> "$STUB_LOG"
noblock=0 quiet=0 args=()
for a in "$@"; do
    case "$a" in --no-block) noblock=1 ;; --quiet|-q) quiet=1 ;; *) args+=("$a") ;; esac
done
set -- "${args[@]}"
case "$1" in
    is-active)
        s="$(cat "$STUB_DIR/argus")"
        [ "$quiet" = 1 ] || echo "$s"
        [ "$s" = active ] ;;
    is-enabled)
        if [ -f "$STUB_DIR/vendor_enabled" ]; then echo enabled; else echo disabled; exit 1; fi ;;
    stop)
        [ -f "$STUB_DIR/stop_fails" ] && exit 1
        if [ -f "$STUB_DIR/stop_hangs" ]; then
            echo deactivating > "$STUB_DIR/argus"
            echo "$PPID" > "$STUB_DIR/stopping"
            exec /bin/sleep 10
        fi
        echo inactive > "$STUB_DIR/argus" ;;
    start)
        [ "$noblock" = 1 ] && [ -f "$STUB_DIR/queue_fails" ] && exit 1
        if [ -n "${INVOCATION_ID:-}" ] && [ -f "$STUB_DIR/argus_waits" ]; then
            [ "$noblock" = 1 ] && exit 0            # queued behind the caller: still inactive
            /bin/sleep 15; exit 1                   # the deadlock of item L
        fi
        if [ -f "$STUB_DIR/start_fails" ]; then
            echo failed > "$STUB_DIR/argus"
            [ "$noblock" = 1 ]; exit $?             # a queued job is accepted, then fails
        fi
        echo active > "$STUB_DIR/argus" ;;
    *) exit 1 ;;
esac
EOF
# modinfo -k KREL -F filename NAME: the file modules.dep names for NAME, as kmod
# resolves it, or "(builtin)" for a name in modules.builtin.  modinfo -F
# vermagic FILE: the "vermagic=" line of the fake .ko.
cat > "$T/bin/modinfo" <<'EOF'
#!/bin/bash
echo "modinfo $*" >> "$STUB_LOG"
if [ "$1" = -k ]; then
    [ "$3" = -F ] && [ "$4" = filename ] || exit 1
    want="${5//-/_}" d="$STUB_DIR/modules/$2"
    for f in modules.dep modules.builtin; do
        [ -f "$d/$f" ] || continue
        while IFS=: read -r rel _; do
            n="$(basename "$rel")"; n="${n%%.ko*}"
            [ "${n//-/_}" = "$want" ] || continue
            if [ "$f" = modules.builtin ]; then echo "(builtin)"; else echo "$d/$rel"; fi
            exit 0
        done < "$d/$f"
    done
    echo "modinfo: ERROR: Module $5 not found." >&2; exit 1
fi
[ "$1" = -F ] && [ "$2" = vermagic ] || exit 1
[ -f "$3" ] || { echo "modinfo: ERROR: Module $3 not found." >&2; exit 1; }
sed -n 's/^vermagic=//p' "$3"
EOF
# modprobe [-r] NAME / insmod FILE / rmmod NAME: edit the fake /proc/modules
# unless told to fail (load_fails, unload_fails).  unload_races: another reload
# unloads it first, so modprobe -r finds it gone and fails.  unload_noop:
# modprobe -r exits 0 but the module stays.  insmod loads "imx296".
cat > "$T/bin/modprobe" <<'EOF'
#!/bin/bash
echo "modprobe $*" >> "$STUB_LOG"
if [ "$1" = -r ]; then
    [ -f "$STUB_DIR/unload_fails" ] && exit 1
    [ -f "$STUB_DIR/unload_noop" ] && exit 0
    sed -i "/^${2//-/_} /d" "$STUB_DIR/proc_modules"
    [ -f "$STUB_DIR/unload_races" ] && exit 1
    exit 0
fi
[ -f "$STUB_DIR/load_fails" ] && exit 1
echo "${1//-/_} 20480 0 - Live 0x0" >> "$STUB_DIR/proc_modules"
EOF
cat > "$T/bin/insmod" <<'EOF'
#!/bin/bash
echo "insmod $*" >> "$STUB_LOG"
[ -f "$STUB_DIR/load_fails" ] && exit 1
echo "imx296 20480 0 - Live 0x0" >> "$STUB_DIR/proc_modules"
EOF
cat > "$T/bin/rmmod" <<'EOF'
#!/bin/bash
echo "rmmod $*" >> "$STUB_LOG"
[ -f "$STUB_DIR/unload_fails" ] && exit 1
sed -i "/^${1//-/_} /d" "$STUB_DIR/proc_modules"
EOF
# sleep: logged, returns at once.
cat > "$T/bin/sleep" <<'EOF'
#!/bin/bash
echo "sleep $*" >> "$STUB_LOG"
EOF
chmod +x "$T/bin/"*

# fake_module <kernel> <vermagic-kernel> [name] [top dir]: a fake .ko under
# <top dir> (default updates), then fake_depmod.  fake_module_nodep: no depmod.
fake_module_nodep() {
    local d="$T/modules/$1/${4:-updates}/drivers/media/i2c"
    mkdir -p "$d"
    printf 'vermagic=%s SMP preempt mod_unload aarch64\n' "$2" > "$d/${3:-imx296}.ko"
}
fake_module() { fake_module_nodep "$@"; fake_depmod "$1"; }
# fake_depmod <kernel>: modules.dep as Ubuntu's depmod writes it ("search
# updates ubuntu built-in"): one entry per module name, updates/ first.
fake_depmod() {
    local d="$T/modules/$1" f n seen=" "
    : > "$d/modules.dep"
    for f in $(cd "$d" && find updates kernel -name '*.ko' 2> /dev/null); do
        n="$(basename "$f" .ko)"; n="${n//-/_}"
        case "$seen" in *" $n "*) continue ;; esac
        seen="$seen$n "
        echo "$f:" >> "$d/modules.dep"
    done
}

# The script's environment: the stubs first on PATH, the fake tree, outside
# systemd unless INVOCATION_ID=... is added.
SCRIPT_ENV=(env -u INVOCATION_ID -u CSI_MODULE -u CSI_MODULE_PATH -u CSI_RELOAD_DELAY
    PATH="$T/bin:$PATH" STUB_LOG="$T/log" STUB_DIR="$T" ARGUS_START_WAIT=1
    KERNEL_RELEASE="$KREL" MODULES_DIR="$T/modules" PROC_MODULES="$T/proc_modules")
# reset_stubs <argus-state>: fresh stub state.  Set before the call, for one
# run: LOAD_FAILS=1, UNLOAD_FAILS=1, UNLOAD_RACES=1, UNLOAD_NOOP=1, STOP_FAILS=1,
# STOP_HANGS=1, START_FAILS=1, QUEUE_FAILS=1, ARGUS_WAITS=1, VENDOR_ENABLED=1,
# PRELOADED=1 (imx296 loaded).
reset_stubs() {
    local f
    : > "$T/log"; : > "$T/proc_modules"; rm -f "$T/stopping"
    for f in load_fails unload_fails unload_races unload_noop stop_fails stop_hangs start_fails \
             queue_fails argus_waits vendor_enabled; do
        rm -f "$T/$f"
    done
    [ "${LOAD_FAILS:-0}" = 1 ] && touch "$T/load_fails"
    [ "${UNLOAD_FAILS:-0}" = 1 ] && touch "$T/unload_fails"
    [ "${UNLOAD_RACES:-0}" = 1 ] && touch "$T/unload_races"
    [ "${UNLOAD_NOOP:-0}" = 1 ] && touch "$T/unload_noop"
    [ "${STOP_FAILS:-0}" = 1 ] && touch "$T/stop_fails"
    [ "${STOP_HANGS:-0}" = 1 ] && touch "$T/stop_hangs"
    [ "${START_FAILS:-0}" = 1 ] && touch "$T/start_fails"
    [ "${QUEUE_FAILS:-0}" = 1 ] && touch "$T/queue_fails"
    [ "${ARGUS_WAITS:-0}" = 1 ] && touch "$T/argus_waits"
    [ "${VENDOR_ENABLED:-0}" = 1 ] && touch "$T/vendor_enabled"
    [ "${PRELOADED:-0}" = 1 ] && echo "imx296 20480 0 - Live 0x0" > "$T/proc_modules"
    LOAD_FAILS=0 UNLOAD_FAILS=0 UNLOAD_RACES=0 UNLOAD_NOOP=0 STOP_FAILS=0 STOP_HANGS=0 START_FAILS=0
    QUEUE_FAILS=0 ARGUS_WAITS=0 VENDOR_ENABLED=0 PRELOADED=0
    echo "$1" > "$T/argus"
}
# run <argus-state> <mode> [VAR=value ...]: fresh stub state, runs the script,
# killed after 10 s.
run() {
    local mode="$2"
    reset_stubs "$1"; shift 2
    timeout -k 2 10 "${SCRIPT_ENV[@]}" "$@" bash "$SCRIPT" "$mode" > "$T/out" 2>&1
    RC=$?
}
# run_term <argus-state> <mode> [VAR=value ...]: as run, with Argus's stop
# hanging; once the script is in it, SIGTERM to the script's process group, as
# systemd sends to the unit's processes at TimeoutStartSec (or Ctrl-C).
run_term() {
    local mode="$2" pid i
    STOP_HANGS=1; reset_stubs "$1"; shift 2
    timeout --foreground -k 2 15 setsid "${SCRIPT_ENV[@]}" "$@" bash "$SCRIPT" "$mode" > "$T/out" 2>&1 &
    pid=$!
    for ((i = 0; i < 160; i++)); do
        [ -s "$T/stopping" ] && break
        /bin/sleep 0.05
    done
    [ -s "$T/stopping" ] && kill -TERM -- "-$(cat "$T/stopping")"
    wait "$pid"
    RC=$?
}
logged() { grep -q -E -- "$1" "$T/log"; }
said()   { grep -q -- "$1" "$T/out"; }
argus()  { cat "$T/argus"; }
loaded() { grep -q "^${1:-imx296} " "$T/proc_modules"; }
# in_order <regex>...: each regex matches a log line after the previous one's.
in_order() {
    local prev=0 n p
    for p in "$@"; do
        n="$(grep -n -E -- "$p" "$T/log" | awk -F: -v m="$prev" '$1 > m { print $1; exit }')"
        [ -n "$n" ] || return 1
        prev="$n"
    done
}
# A check run calls nothing but these (it must change nothing).
read_only() { ! grep -v -E '^(systemctl is-active|systemctl is-enabled|modinfo) ' "$T/log" > /dev/null; }

echo "test_csi_driver — csi_driver.sh against stubs"

echo; echo "--- the 2026-09-28 case: module only for 5.15.185-tegra, running $KREL ---"
rm -rf "$T/modules"; fake_module 5.15.185-tegra 5.15.185-tegra
run active load
check $((RC == 2 ? 0 : 1)) "load refuses with exit 2 (rc=$RC)"
! logged "systemctl stop"; check $? "Argus is not stopped"
[ "$(argus)" = active ]; check $? "Argus is still active"
! logged "modprobe"; check $? "no load is attempted"
said "running kernel $KREL" && said "built for: 5.15.185-tegra"
check $? "the message names both kernels"
run active check
check $((RC == 2 ? 0 : 1)) "check reports the same (exit $RC) and changes nothing"
read_only; check $? "... it only queries"
PRELOADED=1; run active reload
check $((RC == 2 ? 0 : 1)) "reload refuses with exit 2 (rc=$RC)"
! logged "systemctl stop" && ! logged "modprobe" && loaded; check $? "... before Argus or the loaded module is touched"

echo; echo "--- module built for the running kernel, Argus running ---"
fake_module $KREL $KREL
run active check
check $((RC == 0 ? 0 : 1)) "check: would load (exit $RC)"
read_only; check $? "check stops nothing"
run active load
check $((RC == 0 ? 0 : 1)) "load succeeds (exit $RC)"
in_order "^systemctl stop" "^modprobe imx296" "^systemctl start"; check $? "order: stop Argus, modprobe, start Argus"
[ "$(argus)" = active ]; check $? "Argus is active afterwards"

echo; echo "--- the load fails after Argus was stopped ---"
LOAD_FAILS=1; run active load
check $((RC == 4 ? 0 : 1)) "load reports the failure with exit 4 (rc=$RC)"
logged "systemctl stop" && logged "systemctl start"; check $? "Argus was stopped and started again"
[ "$(argus)" = active ]; check $? "Argus is active afterwards"

echo; echo "--- stopping Argus fails ---"
STOP_FAILS=1; run active load
check $((RC == 4 ? 0 : 1)) "exit 4 (rc=$RC)"
! logged "modprobe"; check $? "nothing is loaded under a running Argus"
logged "systemctl start"; check $? "Argus is started on the way out"

echo; echo "--- Argus not running (stopped by hand, or failed for good) ---"
run inactive load
check $((RC == 0 ? 0 : 1)) "load succeeds (exit $RC)"
! logged "systemctl stop" && ! logged "systemctl start"; check $? "Argus is neither stopped nor started"
[ "$(argus)" = inactive ]; check $? "Argus is left as it was (still inactive)"

echo; echo "--- already loaded ---"
PRELOADED=1; run active load
check $((RC == 0 ? 0 : 1)) "exit 0 (rc=$RC)"
! logged "systemctl stop" && ! logged "modprobe"; check $? "nothing is touched"

echo; echo "--- K: the module is named imx296 ---"
rm -rf "$T/modules"; fake_module $KREL $KREL imx296
run active check
check $((RC == 0 ? 0 : 1)) "check with the default name finds imx296.ko (exit $RC)"
said "would load $T/modules/$KREL/updates/drivers/media/i2c/imx296.ko"; check $? "... and names it"
run active load
check $((RC == 0 ? 0 : 1)) "load with the default name loads it (exit $RC)"
logged "^modprobe imx296$" && loaded; check $? "... as imx296"
grep -q -x "Environment=CSI_MODULE=imx296" "$UNIT"; check $? "the unit sets CSI_MODULE=imx296"
# shellcheck disable=SC2016  # the backquotes are literal
cause='its `modprobe -r` failed'
head -n 20 "$SCRIPT" | grep -q -F "$cause" && head -n 20 "$UNIT" | grep -q -F "$cause"
check $? "both headers name the old unit's real failure, its modprobe -r"

echo; echo "--- L: run by systemd while Argus runs (systemctl start/restart csi-driver) ---"
ARGUS_WAITS=1; run active load INVOCATION_ID=stub
check $((RC != 124 && RC != 137 ? 0 : 1)) "it returns: no blocking start on a job that waits for it (rc=$RC)"
logged "^systemctl --no-block start" && ! logged "^systemctl start"; check $? "... the Argus start is queued with --no-block"
check $((RC == 5 ? 0 : 1)) "... and, not seen running within ARGUS_START_WAIT, reported with exit 5 (rc=$RC)"
run active load INVOCATION_ID=stub
check $((RC == 0 ? 0 : 1)) "with Argus's job not waiting for it: exit 0 (rc=$RC)"
[ "$(argus)" = active ] && in_order "^systemctl stop" "^modprobe imx296" "^systemctl --no-block start" "^systemctl is-active"
check $? "... stop, load, queue the start, see Argus active"
run active load
logged "^systemctl start" && ! logged "no-block"; check $? "outside systemd the start is a plain blocking one"
grep -q -x "After=nvargus-daemon.service.*" "$UNIT" && ! grep -q "^Before=.*nvargus" "$UNIT"
check $? "the unit is ordered After=nvargus-daemon, not Before= (no job of Argus's waits for it)"

echo; echo "--- M: reload, the vendor unit's job with the checks first ---"
PRELOADED=1; run active reload CSI_RELOAD_DELAY=5
check $((RC == 0 ? 0 : 1)) "reload with the module loaded and Argus running: exit 0 (rc=$RC)"
in_order "^modinfo -k $KREL -F filename imx296" "^sleep 5$" "^systemctl stop" "^modprobe -r imx296" \
    "^modprobe imx296" "^systemctl start"
check $? "order: check, wait 5 s, stop Argus, modprobe -r, modprobe, start Argus"
loaded && [ "$(argus)" = active ]; check $? "... loaded, Argus active"
run active reload
check $((RC == 0 ? 0 : 1)) "reload with nothing loaded: exit 0 (rc=$RC)"
! logged "modprobe -r" && logged "^modprobe imx296" && ! logged "^sleep"; check $? "... loads only, no wait by default"
UNLOAD_FAILS=1 PRELOADED=1; run active reload
check $((RC == 4 ? 0 : 1)) "modprobe -r fails: exit 4 (rc=$RC)"
! logged "^modprobe imx296" && loaded && [ "$(argus)" = active ]; check $? "... no load, the loaded copy stays, Argus back up"
said "modprobe -r imx296 failed (in use?); the loaded copy stays"; check $? "... and says so"
UNLOAD_NOOP=1 PRELOADED=1; run active reload
check $((RC == 4 ? 0 : 1)) "modprobe -r exits 0 but the module stays: exit 4 (rc=$RC)"
! logged "^modprobe imx296" && said "still in"; check $? "... no load on top of it, and says so"
UNLOAD_RACES=1 PRELOADED=1; run active reload
check $((RC == 0 ? 0 : 1)) "modprobe -r fails because another reload unloaded it first: exit 0 (rc=$RC)"
logged "^modprobe imx296" && loaded && [ "$(argus)" = active ] && said "is no longer loaded; loading it"
check $? "... loaded all the same, Argus back up"
for s in activating reloading; do
    PRELOADED=1; run $s reload
    check $((RC == 0 ? 0 : 1)) "Argus $s (about to run, e.g. Restart=on-failure): exit 0 (rc=$RC)"
    in_order "^systemctl stop" "^modprobe -r imx296" "^modprobe imx296" "^systemctl start" && [ "$(argus)" = active ]
    check $? "... stopped before the unload, like a running one, and started again"
done
VENDOR_ENABLED=1 PRELOADED=1; run active reload
check $((RC == 0 ? 0 : 1)) "the vendor unit still enabled: reload goes on (exit $RC)"
said "imx296-reload.service is enabled"; check $? "... and says to disable it (the two race)"
rm -rf "$T/modules"; fake_module $KREL 5.15.185-tegra
PRELOADED=1; run active reload CSI_RELOAD_DELAY=5
check $((RC == 3 ? 0 : 1)) "a module built for another kernel: reload refuses, exit 3 (rc=$RC)"
! logged "^sleep" && ! logged "systemctl stop" && ! logged "modprobe" && loaded
check $? "... before the wait, Argus or the loaded module is touched"
rm -rf "$T/modules"; fake_module $KREL $KREL
PRELOADED=1; run active reload INVOCATION_ID=stub
check $((RC == 0 ? 0 : 1)) "under systemd: exit 0 (rc=$RC)"
logged "^systemctl --no-block start" && [ "$(argus)" = active ]; check $? "... Argus queued and seen active"
PRELOADED=1; run inactive reload
check $((RC == 0 ? 0 : 1)) "Argus not running: exit 0 (rc=$RC)"
! logged "systemctl stop" && ! logged "systemctl start" && loaded; check $? "... reloaded, Argus left alone"
printf 'vermagic=%s SMP\n' $KREL > "$T/other.ko"
PRELOADED=1; run active reload CSI_MODULE_PATH="$T/other.ko"
check $((RC == 0 ? 0 : 1)) "CSI_MODULE_PATH: exit 0 (rc=$RC)"
in_order "^rmmod imx296" "^insmod $T/other.ko"; check $? "... rmmod, then insmod"
UNLOAD_FAILS=1 PRELOADED=1; run active reload CSI_MODULE_PATH="$T/other.ko"
check $((RC == 4 ? 0 : 1)) "CSI_MODULE_PATH, rmmod fails: exit 4 (rc=$RC)"
! logged "^insmod" && loaded && said "rmmod imx296 failed (in use?)"; check $? "... no insmod, and says so"
run active reload CSI_RELOAD_DELAY=x
check $((RC == 1 ? 0 : 1)) "CSI_RELOAD_DELAY=x: usage error, exit 1 (rc=$RC)"
! logged "systemctl stop" && ! logged "modprobe"; check $? "... nothing touched"
for w in 0 x; do
    PRELOADED=1; run active reload ARGUS_START_WAIT=$w
    check $((RC == 1 ? 0 : 1)) "ARGUS_START_WAIT=$w: usage error, exit 1 (rc=$RC)"
    ! logged "systemctl stop" && ! logged "modprobe"; check $? "... nothing touched"
done
grep -q -x "ExecStart=/usr/local/sbin/dashcam-csi-driver reload" "$UNIT" \
    && grep -q -x "Environment=CSI_RELOAD_DELAY=5" "$UNIT"
check $? "the unit runs reload after a 5 s wait, as the vendor unit does"
grep -q -x "TimeoutStartSec=180" "$UNIT"
check $? "the unit bounds the oneshot (TimeoutStartSec=180: 5 s + Argus's 90 s stop + 20 s watch fit)"

echo; echo "--- N: the file modprobe loads, as depmod chose it ---"
rm -rf "$T/modules"
fake_module_nodep $KREL 5.15.185-tegra imx296 kernel
fake_module $KREL $KREL imx296 updates
run active check
check $((RC == 0 ? 0 : 1)) "stale kernel/ copy, good updates/ copy (depmod's choice): check passes (exit $RC)"
said "would load $T/modules/$KREL/updates/"; check $? "... and judges the updates/ file"
read_only; check $? "... it only queries"
rm -rf "$T/modules"
fake_module_nodep $KREL $KREL imx296 kernel
fake_module $KREL 5.15.185-tegra imx296 updates
run active check
check $((RC == 3 ? 0 : 1)) "good kernel/ copy, stale updates/ copy (depmod's choice): exit 3 (rc=$RC)"
run active load
check $((RC == 3 ? 0 : 1)) "... load refuses it (rc=$RC)"
! logged "systemctl stop" && ! logged "modprobe"; check $? "... before Argus is touched"
rm -rf "$T/modules"; fake_module_nodep $KREL $KREL
run active check
check $((RC == 2 ? 0 : 1)) "installed, but depmod has not run: exit 2 (rc=$RC)"
said "depmod does not know it" && said "sudo depmod $KREL"; check $? "... and it says to run depmod"
run active load
check $((RC == 2 ? 0 : 1)) "... load refuses (rc=$RC)"
! logged "systemctl stop" && ! logged "modprobe"; check $? "... before Argus is touched"
rm -rf "$T/modules"
fake_module $KREL $KREL imx296 kernel
touch -d "1 hour ago" "$T/modules/$KREL/modules.dep"
fake_module_nodep $KREL $KREL imx296 updates
run active check
check $((RC == 0 ? 0 : 1)) "a copy installed after the last depmod: check still passes on the one modprobe loads (exit $RC)"
said "updates/drivers/media/i2c/imx296.ko was installed after the last depmod"; check $? "... with a note to run depmod"
touch -d "3 days ago" "$T/modules/$KREL/updates/drivers/media/i2c/imx296.ko"
run active check
check $((RC == 0 ? 0 : 1)) "... the same copy with its old mtime kept (cp -p, tar): exit 0 (rc=$RC)"
said "updates/drivers/media/i2c/imx296.ko was installed after the last depmod" && ! said "is not used"
check $? "... still noted as installed after depmod (its ctime), not as passed over"
rm -rf "$T/modules"
fake_module_nodep $KREL $KREL imx296 kernel
touch -d "2 hours ago" "$T/modules/$KREL/kernel/drivers/media/i2c/imx296.ko"
fake_module $KREL $KREL imx296 updates
run active check
check $((RC == 0 ? 0 : 1)) "an older copy depmod passed over: exit 0 (rc=$RC)"
said "kernel/drivers/media/i2c/imx296.ko is not used"; check $? "... noted as not used"
rm -rf "$T/modules"; fake_module $KREL $KREL
rm -f "$T/modules/$KREL/updates/drivers/media/i2c/imx296.ko"
run active check
check $((RC == 2 ? 0 : 1)) "modules.dep names a file that is gone: exit 2 (rc=$RC)"
rm -rf "$T/modules"; mkdir -p "$T/modules/$KREL"
echo "kernel/drivers/media/i2c/imx296.ko" > "$T/modules/$KREL/modules.builtin"
run active check
check $((RC == 2 ? 0 : 1)) "built into the kernel (modinfo says '(builtin)'): exit 2 (rc=$RC)"
said "is built into kernel $KREL"; check $? "... and says so"
run active reload
check $((RC == 2 ? 0 : 1)) "... reload refuses (rc=$RC)"
! logged "systemctl stop" && ! logged "modprobe"; check $? "... before Argus is touched"
rm -rf "$T/modules"; fake_module $KREL 5.15.185-tegra
PRELOADED=1; run active check
check $((RC == 3 ? 0 : 1)) "loaded, but the file for this kernel cannot load: check fails, exit 3 (rc=$RC)"
said "is loaded; reload would refuse and leave it loaded" && read_only; check $? "... says reload would refuse, changes nothing"

echo; echo "--- O: Argus cannot be started again ---"
rm -rf "$T/modules"; fake_module $KREL $KREL
START_FAILS=1; run active load
check $((RC == 5 ? 0 : 1)) "the start fails: exit 5, not 0 (rc=$RC)"
said "could not start nvargus-daemon.service again"; check $? "... and says Argus is down"
START_FAILS=1 LOAD_FAILS=1; run active load
check $((RC == 5 ? 0 : 1)) "the load and the start fail: exit 5 (rc=$RC)"
said "modprobe imx296 failed"; check $? "... and both failures are logged"
START_FAILS=1 PRELOADED=1; run active reload INVOCATION_ID=stub
check $((RC == 5 ? 0 : 1)) "under systemd, the queued start ends failed: exit 5 (rc=$RC)"
said "is 'failed', not active"; check $? "... and says so"
QUEUE_FAILS=1 PRELOADED=1; run active reload INVOCATION_ID=stub
check $((RC == 5 ? 0 : 1)) "under systemd, the start cannot be queued: exit 5 (rc=$RC)"
said "could not queue a start of nvargus-daemon.service"; check $? "... and says Argus is down"

echo; echo "--- a step hangs: SIGTERM (systemd at TimeoutStartSec, or Ctrl-C) ---"
for inv in "" stub; do
    where="outside systemd"; [ -n "$inv" ] && where="under systemd"
    PRELOADED=1; run_term active reload ${inv:+INVOCATION_ID=$inv}
    check $((RC == 130 ? 0 : 1)) "$where, SIGTERM while Argus's stop hangs: exit 130 (rc=$RC)"
    in_order "^systemctl stop" "^systemctl (--no-block )?start" && [ "$(argus)" = active ]
    check $? "... Argus started again on the way out"
    ! logged "^modprobe" && loaded; check $? "... the loaded module untouched"
done

echo; echo "--- P: run directly ---"
[ -x "$HERE/test_csi_driver.sh" ]; check $? "test_csi_driver.sh is executable"
[ -x "$SCRIPT" ]; check $? "csi_driver.sh is executable"

echo; echo "--- names, paths, notes ---"
rm -rf "$T/modules"; fake_module $KREL $KREL nv-imx296
run active load CSI_MODULE=nv_imx296
check $((RC == 0 ? 0 : 1)) "nv-imx296.ko is found for CSI_MODULE=nv_imx296 (exit $RC)"
rm -rf "$T/modules"; mkdir -p "$T/modules/$KREL"
run active check
check $((RC == 2 ? 0 : 1)) "no module anywhere: exit 2 (rc=$RC)"
said "Is CSI_MODULE the driver's name"; check $? "... and it asks whether the name is right"
printf 'vermagic=5.15.185-tegra SMP\n' > "$T/other.ko"
run active load CSI_MODULE_PATH="$T/other.ko"
check $((RC == 3 ? 0 : 1)) "CSI_MODULE_PATH built for another kernel: exit 3 (rc=$RC)"
! logged "systemctl stop" && ! logged "insmod"; check $? "... refused before Argus is touched"
printf 'vermagic=%s SMP\n' $KREL > "$T/other.ko"
run active load CSI_MODULE_PATH="$T/other.ko"
check $((RC == 0 ? 0 : 1)) "CSI_MODULE_PATH built for this kernel: insmod'd (exit $RC)"
logged "insmod $T/other.ko" && [ "$(argus)" = active ]; check $? "... with insmod, Argus back up"
fake_module $KREL $KREL
VENDOR_ENABLED=1; run active check
said "imx296-reload.service is enabled"; check $? "check says when the vendor unit is still enabled"
read_only; check $? "... it only queries"
run active bogus
check $((RC == 1 ? 0 : 1)) "an unknown command: usage, exit 1"

echo
if [ "$FAILS" -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL ($FAILS)"; fi
[ "$FAILS" -eq 0 ]
