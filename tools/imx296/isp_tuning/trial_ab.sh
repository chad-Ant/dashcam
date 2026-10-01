#!/bin/bash
# trial_ab.sh [--auto] <candidate>...        Run as root: sudo bash trial_ab.sh c8_sh15 c8_v5off c8_v5tab15
#
# A/B of ISP override candidates (<candidate>.isp next to this script) against the base c5_rpi100T.isp,
# in A-B-A order: base, cand1, base, cand2, base, ...  Each step installs the profile, restarts
# nvargus-daemon, saves the daemon's journal from that restart, and captures one tune_session.py session as
# jetson (--with-unset; --manual = fixed 675 lines / code 184 with AE/AWB locked, the bench default; --auto =
# neither, for daylight). Output: trial_<stamp>/NN_<profile>/, plus trial.log and manifest.txt there.
# Analyse with: python3 ab_report.py trial_<stamp>   (in the l4t-ml-gpio container)
#
# Safety (the 2026-10-01 review of trial_c7.sh):
#  - refuses unless the installed profile is the base, the recorder unit is inactive, no dashcam binary runs
#    and nothing but nvargus-daemon holds /dev/video0 or the UGREEN;
#  - copies the entry profile, the base and the candidates into a root-only backup directory, hashes the
#    copies and installs only from there (nothing is read twice from a jetson-writable path);
#  - installs by rename, so the override file is never half-written; refuses a candidate named 'entry';
#  - the capture runs in the background and the script waits on it, so Ctrl-C, SIGTERM and a closed terminal
#    (and Ctrl-\) act at once: the capture is stopped (TERM, then KILL of its process group), then the entry profile is put
#    back and nvargus-daemon restarted (a TERM/HUP sent to the script's PID waits for a running systemctl or
#    sleep; a tty Ctrl-C reaches those too). Signals are ignored during the restore, and a signal that lands
#    as the restore starts cannot cut it short. The file restore and the daemon restart are reported apart.
# Never starts or stops the recorder. Power loss or SIGKILL of this script cannot be trapped: the backup
# directory printed at the start holds entry.isp, the profile to put back.
set -euo pipefail
HERE="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
SET=/var/nvidia/nvcam/settings/camera_overrides.isp
BASE=c5_rpi100T
UG=/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0
OWNER=jetson
MODE=--manual
CAPTURE_TIMEOUT=150

usage() { echo "usage: sudo bash $0 [--auto] <candidate>...  (candidates: <name>.isp in $HERE)"; exit 2; }
[ "${1:-}" = --auto ] && { MODE=--auto; shift; }
[ $# -ge 1 ] || usage
[ "$(id -u)" -eq 0 ] || { echo "Run with: sudo bash $0 [--auto] <candidate>..."; exit 1; }

# ---- checks before anything is touched -------------------------------------------------------------
for c in "$@"; do
    [[ "$c" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "Bad candidate name: $c"; exit 1; }
    [ "$c" != "$BASE" ] || { echo "$BASE is the base; list candidates only"; exit 1; }
    [ "$c" != entry ] || { echo "'entry' is reserved (the backup of the installed profile is entry.isp)"; exit 1; }
    [ -s "$HERE/$c.isp" ] || { echo "Missing: $HERE/$c.isp"; exit 1; }
done
[ -s "$HERE/$BASE.isp" ] || { echo "Missing: $HERE/$BASE.isp"; exit 1; }
cmp -s "$SET" "$HERE/$BASE.isp" || { echo "The installed profile is not $BASE.isp; refusing (nothing changed)."; exit 1; }
if systemctl is-active --quiet dashcam-v04; then echo 'dashcam-v04 is active; stop it first.'; exit 1; fi
if pgrep -f 'dashcam_v0_[0-9]' > /dev/null; then echo 'A dashcam binary is running:'; pgrep -af 'dashcam_v0_[0-9]'; exit 1; fi
ARGUS_PIDS=" $(pgrep -x nvargus-daemon | tr '\n' ' ') "
HOLDERS=""
for pid in $(fuser /dev/video0 "$(readlink -f "$UG")" 2>/dev/null); do
    [[ "$ARGUS_PIDS" == *" $pid "* ]] || HOLDERS="$HOLDERS $pid"
done
if [ -n "$HOLDERS" ]; then echo 'Something holds a camera:'; ps -o pid,user,args -p "$(tr -s ' ' ',' <<< "${HOLDERS# }")"; exit 1; fi
[ -x /usr/bin/python3 ] && [ -r "$HERE/tune_session.py" ] || { echo 'tune_session.py or python3 missing'; exit 1; }

# ---- backup and private copies ------------------------------------------------------------------------
STAMP="$(date +%Y%m%d_%H%M%S)"
BK="$(mktemp -d /var/backups/csi-trial.XXXXXX)"          # root, 0700
cp -a "$SET" "$BK/entry.isp"
install -m 0600 "$HERE/$BASE.isp" "$BK/$BASE.isp"
cmp -s "$SET" "$BK/$BASE.isp" || { echo "$BASE.isp changed while copying; refusing (nothing changed)."; rm -rf "$BK"; exit 1; }
for c in "$@"; do install -m 0600 "$HERE/$c.isp" "$BK/$c.isp"; done
OUT="$HERE/trial_$STAMP"
install -d -o "$OWNER" -g "$OWNER" -m 0755 "$OUT"
install -o "$OWNER" -g "$OWNER" -m 0644 /dev/null "$OUT/trial.log"
log() {     # never fails: under set -e a failed log inside restore() would end it before the profile is put back
    { printf '%s %s\n' "$(date +%T)" "$*" >> "$OUT/trial.log"; } 2> /dev/null || true
    printf '%s\n' "$*" 2> /dev/null || true
}
{
    echo "trial_ab.sh $MODE $*   started $(date -Is)"
    echo "backup dir: $BK"
    (cd "$BK" && sha256sum entry.isp "$BASE.isp" "${@/%/.isp}")
} > "$OUT/manifest.txt"
log "Backup of the installed profile: $BK/entry.isp"
log "Output: $OUT"

put() {     # install a private copy by rename, so the override is never half-written
    install -m 0644 "$1" "$SET.trial_ab.tmp" && mv -f "$SET.trial_ab.tmp" "$SET"
}

CPID=""; CUR=""
stop_capture() {
    [ -n "$CPID" ] || CPID="$(jobs -pr | head -n 1)"         # a signal between '&' and CPID=$!
    [ -n "$CPID" ] || return 0
    kill -TERM "$CPID" 2> /dev/null || true                 # timeout passes TERM to sudo, sudo to python
    for _ in $(seq 30); do kill -0 "$CPID" 2> /dev/null || break; sleep 0.5; done
    kill -KILL -- "-$CPID" 2> /dev/null || true             # timeout leads its own process group
    [ -n "$CUR" ] && pkill -KILL -f "tune_session.py $CUR" 2> /dev/null || true   # python in its own session (use_pty)
    wait "$CPID" 2> /dev/null || true
    CPID=""
    log "Capture stopped."
}

restore() {    # restore [exit code]: the EXIT trap (code = $?) or on_signal
    local rc=${1:-$?}
    trap '' INT TERM HUP QUIT PIPE
    trap - EXIT
    stop_capture
    local file_ok=0 daemon_ok=0
    if put "$BK/entry.isp" && cmp -s "$BK/entry.isp" "$SET"; then file_ok=1; fi
    if systemctl restart nvargus-daemon; then daemon_ok=1; fi
    if [ "$file_ok" = 1 ]; then
        log "Entry profile restored ($(sha256sum "$SET" | cut -c1-12))."
    else
        log "FILE RESTORE FAILED: put $BK/entry.isp back as $SET, then restart nvargus-daemon."; rc=1
    fi
    if [ "$daemon_ok" = 1 ]; then log "nvargus-daemon restarted."; else log "nvargus-daemon restart FAILED: systemctl restart nvargus-daemon"; rc=1; fi
    log "Recorder untouched. Exit $rc."
    chown -hR "$OWNER:$OWNER" "$OUT" 2> /dev/null || true
    exit "$rc"
}
on_signal() {
    trap '' INT TERM HUP QUIT PIPE  # first, so no second handler can run inside restore() and exit from it
    case " ${FUNCNAME[*]} " in *" restore "*) return 0 ;; esac   # restore() already running: let it finish
    log "Signal received; stopping."
    restore "$1"                    # not exit: run as the EXIT trap starts, exit would end the shell unrestored
}
trap restore EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'on_signal 129' HUP
trap 'on_signal 131' QUIT          # Ctrl-\ aborts like Ctrl-C

# journal lines of one restart, with timestamps, PIDs and addresses removed, for comparison with the base's
norm() { sed -E 's/^.*nvargus-daemon\[[0-9]+\]: //; s/0x[0-9a-fA-F]+|[0-9a-fA-F]{8,}|\[[0-9]+\]|\([0-9]+\)//g' | sort -u; }

step() {    # step <index> <profile name>
    local k="$1" name="$2" d t0 rc=0
    d="$OUT/$(printf '%02d' "$k")_$name"
    t0="$(date +%s)"
    put "$BK/$name.isp"
    cmp -s "$BK/$name.isp" "$SET" || { log "Install of $name did not take"; return 1; }
    systemctl restart nvargus-daemon
    sleep 3
    log "[$k] $name installed ($(sha256sum "$SET" | cut -c1-12)); capturing -> $d"
    CUR="$d"
    timeout -k 10s "${CAPTURE_TIMEOUT}s" sudo -u "$OWNER" /usr/bin/python3 "$HERE/tune_session.py" "$d" "$MODE" --with-unset \
        >> "$OUT/trial.log" 2>&1 &
    CPID=$!
    wait "$CPID" || rc=$?
    CPID=""
    pkill -KILL -f "tune_session.py $d" 2> /dev/null || true     # a python left in its own session (sudo use_pty)
    journalctl -u nvargus-daemon --since "@$t0" --no-pager -o short-precise > "$d.nvargus.log" 2>&1 || true
    chown -h "$OWNER:$OWNER" "$d.nvargus.log" 2> /dev/null || true
    if [ "$name" = "$BASE" ]; then
        [ -s "$OUT/base.nvargus.norm" ] || norm < "$d.nvargus.log" > "$OUT/base.nvargus.norm"
    elif [ -s "$OUT/base.nvargus.norm" ]; then
        local new; new="$(norm < "$d.nvargus.log" | comm -13 "$OUT/base.nvargus.norm" - | grep -i -E 'error|fail|invalid|unable|warn|pars' || true)"
        [ -z "$new" ] || { log "WARNING: nvargus logged lines for $name that the base did not (override rejected?):"; log "$new"; }
    fi
    [ "$rc" = 0 ] || { log "Capture of $name failed (rc $rc)"; return 1; }
}

k=0
step $k "$BASE"
for c in "$@"; do
    k=$((k + 1)); step $k "$c"
    k=$((k + 1)); step $k "$BASE"
done
log "All captures complete. Analyse: python3 ab_report.py $(basename "$OUT")  (l4t-ml-gpio container)"
