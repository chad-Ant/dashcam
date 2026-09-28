#!/bin/bash
#
# test_csi_driver.sh — host test for csi_driver.sh, no Jetson needed.
#
# systemctl, modprobe, insmod and modinfo are stubs on PATH that log every call
# and act on a fake /lib/modules tree and a fake /proc/modules; nothing on the
# machine is touched.  Checks that Argus is never stopped when the load cannot
# work (the 2026-09-28 kernel mismatch), and is always started again once it
# was stopped.
#
# Usage: docker_dev/test_csi_driver.sh      (prints RESULT: PASS / FAIL)

set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
SCRIPT="$HERE/csi_driver.sh"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
FAILS=0

check() {   # check <condition-exit-code> <what>
    if [ "$1" -eq 0 ]; then echo "  ok    $2"; else echo "  FAIL  $2"; FAILS=$((FAILS + 1)); fi
}

mkdir -p "$T/bin"
# systemctl: is-active/stop/start on one unit; state in $T/argus (active|inactive).
cat > "$T/bin/systemctl" <<'EOF'
#!/bin/bash
echo "systemctl $*" >> "$STUB_LOG"
case "$1" in
    is-active) [ "$(cat "$STUB_DIR/argus")" = active ] ;;
    stop)  [ -f "$STUB_DIR/stop_fails" ] && exit 1; echo inactive > "$STUB_DIR/argus" ;;
    start) echo active > "$STUB_DIR/argus" ;;
    *) exit 1 ;;
esac
EOF
# modinfo -F vermagic FILE: the "vermagic=" line written into the fake .ko.
cat > "$T/bin/modinfo" <<'EOF'
#!/bin/bash
echo "modinfo $*" >> "$STUB_LOG"
[ "$1" = -F ] && [ "$2" = vermagic ] || exit 1
sed -n 's/^vermagic=//p' "$3"
EOF
# modprobe NAME / insmod FILE: append to the fake /proc/modules unless told to fail.
cat > "$T/bin/modprobe" <<'EOF'
#!/bin/bash
echo "modprobe $*" >> "$STUB_LOG"
[ -f "$STUB_DIR/load_fails" ] && exit 1
echo "${1//-/_} 20480 0 - Live 0x0" >> "$STUB_DIR/proc_modules"
EOF
cat > "$T/bin/insmod" <<'EOF'
#!/bin/bash
echo "insmod $*" >> "$STUB_LOG"
[ -f "$STUB_DIR/load_fails" ] && exit 1
echo "nv_imx296 20480 0 - Live 0x0" >> "$STUB_DIR/proc_modules"
EOF
chmod +x "$T/bin/"*

fake_module() {   # fake_module <kernel> <vermagic-kernel> [name]
    local d="$T/modules/$1/updates/drivers/media/i2c"
    mkdir -p "$d"
    printf 'vermagic=%s SMP preempt mod_unload aarch64\n' "$2" > "$d/${3:-nv_imx296}.ko"
}

# run <argus-state> <mode> [VAR=value ...]: fresh stub state, runs the script.
# Set before the call: LOAD_FAILS=1, STOP_FAILS=1, PRELOADED=1 (for one run).
run() {
    local argus="$1" mode="$2"; shift 2
    : > "$T/log"; : > "$T/proc_modules"; rm -f "$T/load_fails" "$T/stop_fails"
    [ "${LOAD_FAILS:-0}" = 1 ] && touch "$T/load_fails"
    [ "${STOP_FAILS:-0}" = 1 ] && touch "$T/stop_fails"
    [ "${PRELOADED:-0}" = 1 ] && echo "nv_imx296 20480 0 - Live 0x0" > "$T/proc_modules"
    LOAD_FAILS=0 STOP_FAILS=0 PRELOADED=0
    echo "$argus" > "$T/argus"
    env PATH="$T/bin:$PATH" STUB_LOG="$T/log" STUB_DIR="$T" \
        KERNEL_RELEASE=5.15.199-tegra MODULES_DIR="$T/modules" PROC_MODULES="$T/proc_modules" \
        "$@" bash "$SCRIPT" "$mode" > "$T/out" 2>&1
    RC=$?
}
logged() { grep -q -- "$1" "$T/log"; }
said()   { grep -q -- "$1" "$T/out"; }
argus()  { cat "$T/argus"; }

echo "test_csi_driver — csi_driver.sh against stubs"

echo; echo "--- the 2026-09-28 case: module only for 5.15.185-tegra, running 5.15.199-tegra ---"
rm -rf "$T/modules"; fake_module 5.15.185-tegra 5.15.185-tegra
run active load
check $((RC == 2 ? 0 : 1)) "load refuses with exit 2 (rc=$RC)"
! logged "systemctl stop"; check $? "Argus is not stopped"
[ "$(argus)" = active ]; check $? "Argus is still active"
! logged "modprobe"; check $? "no load is attempted"
said "running kernel 5.15.199-tegra" && said "built for: 5.15.185-tegra"
check $? "the message names both kernels"
run active check
check $((RC == 2 ? 0 : 1)) "check reports the same (exit $RC) and changes nothing"
! logged "systemctl stop" && ! logged "modprobe"; check $? "... no stop, no load"

echo; echo "--- module built for the running kernel, Argus running ---"
fake_module 5.15.199-tegra 5.15.199-tegra
run active check
check $((RC == 0 ? 0 : 1)) "check: would load (exit $RC)"
! logged "systemctl stop"; check $? "check stops nothing"
run active load
check $((RC == 0 ? 0 : 1)) "load succeeds (exit $RC)"
grep -n "" "$T/log" | grep -E "systemctl stop|modprobe nv_imx296|systemctl start" | cut -d: -f1 | tr '\n' ' ' > "$T/order"
read -r a b c _ < "$T/order"
[ -n "${c:-}" ] && [ "$a" -lt "$b" ] && [ "$b" -lt "$c" ]
check $? "order: stop Argus, modprobe, start Argus"
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

echo; echo "--- at boot: Argus not running yet ---"
run inactive load
check $((RC == 0 ? 0 : 1)) "load succeeds (exit $RC)"
! logged "systemctl stop" && ! logged "systemctl start"; check $? "Argus is neither stopped nor started"
[ "$(argus)" = inactive ]; check $? "Argus is left to systemd (still inactive)"

echo; echo "--- already loaded ---"
PRELOADED=1; run active load
check $((RC == 0 ? 0 : 1)) "exit 0 (rc=$RC)"
! logged "systemctl stop" && ! logged "modprobe"; check $? "nothing is touched"

echo; echo "--- names and paths ---"
rm -rf "$T/modules"; fake_module 5.15.199-tegra 5.15.199-tegra nv-imx296
run active load
check $((RC == 0 ? 0 : 1)) "nv-imx296.ko is found for CSI_MODULE=nv_imx296 (exit $RC)"
rm -rf "$T/modules"; mkdir -p "$T/modules/5.15.199-tegra"
run active check
check $((RC == 2 ? 0 : 1)) "no module anywhere: exit 2 (rc=$RC)"
said "Is CSI_MODULE the driver's name"; check $? "... and it asks whether the name is right"
printf 'vermagic=5.15.185-tegra SMP\n' > "$T/other.ko"
run active load CSI_MODULE_PATH="$T/other.ko"
check $((RC == 3 ? 0 : 1)) "CSI_MODULE_PATH built for another kernel: exit 3 (rc=$RC)"
! logged "systemctl stop" && ! logged "insmod"; check $? "... refused before Argus is touched"
printf 'vermagic=5.15.199-tegra SMP\n' > "$T/other.ko"
run active load CSI_MODULE_PATH="$T/other.ko"
check $((RC == 0 ? 0 : 1)) "CSI_MODULE_PATH built for this kernel: insmod'd (exit $RC)"
logged "insmod $T/other.ko" && [ "$(argus)" = active ]; check $? "... with insmod, Argus back up"
run active bogus
check $((RC == 1 ? 0 : 1)) "an unknown command: usage, exit 1"

echo
if [ "$FAILS" -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL ($FAILS)"; fi
[ "$FAILS" -eq 0 ]
