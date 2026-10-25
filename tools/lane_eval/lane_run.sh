#!/bin/bash
# lane_run.sh — orchestrates a lane test run (plan P5a, section 12): the
# per-camera lane_drive containers, the MKR raw-CAN logger, sysmon.py and
# tegrastats, plus a supervisor that restarts the CAN logger if it dies.
#
#   tools/lane_eval/lane_run.sh start [--minutes N] [--cams csi,usb] [--no-can]
#                                     [--plan day|night] [--csi-crop T,B] [--usb-crop T,B]
#                                     [-- extra lane_drive args (both cameras)]
#   tools/lane_eval/lane_run.sh status
#   tools/lane_eval/lane_run.sh stop
#
# Runs on the HOST as the jetson user (docker group).  dashcam-v04 and
# drive_session must not hold the cameras / MKR: start refuses while they do
# (stopping the service is a root step: sudo systemctl stop dashcam-v04).
# Output: /media/jetson/backup/lane_tests/run_NNN/ (outside the repo — the
# repo is public; LANE_ROOT overrides).  stop: docker stop -t 45 (SIGTERM →
# each lane_drive finalises and exits 0; 4 = finalised after a stop hang),
# exit codes recorded in run_end.json.

set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd -P)"
ROOT="${LANE_ROOT:-/media/jetson/backup/lane_tests}"
CURRENT="$ROOT/.current"
IMAGE=l4t-ml-gpio
UGREEN=/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0
MKR_GLOB="/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKRZero_*"
CANSTREAM="$REPO/peripherals/mkr_zero/tools/canstream"
CONTAINERS="lane_csi lane_usb lane_mkr"

say() { echo "[lane_run $(date +%H:%M:%S)] $*"; }
die() { say "ERROR: $*" >&2; exit 1; }
# event TYPE [DETAIL...] — host-side events.csv of the run
# (CLOCK_MONOTONIC ns, the clock lane_drive's CSVs use; awk's %d overflows at 2^31.)
event() { echo "$(python3 -c 'import time; print(time.monotonic_ns())'),$(date +%s%N),host,$1,${*:2}" >> "$RUN/events_host.csv"; }

running() { [ "$(docker inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" = true ]; }

# The newest complete build (the dashcam launchers' convention).
newest_build() {
    local d
    # build_<YYYYMMDD_HHMMSS>: newest first by name
    for d in $(printf '%s\n' "$REPO"/bin/build_[0-9]*_[0-9]* | sort -r); do
        [ -x "$d/lane_drive" ] && { echo "$d"; return; }
    done
}

preflight() {
    local ok=1
    if systemctl is-active -q dashcam-v04 2>/dev/null; then
        say "FAIL dashcam-v04 is active — it holds the UGREEN and the C3 (sudo systemctl stop dashcam-v04)"; ok=0
    fi
    for c in drive_csi drive_mkr dashcam_v04 $CONTAINERS; do
        running "$c" && { say "FAIL container $c is running"; ok=0; }
    done
    [ -e /dev/video0 ] || { say "FAIL /dev/video0 (IMX296) missing"; ok=0; }
    case ",$CAMS," in *,usb,*) [ -e "$UGREEN" ] || { say "FAIL UGREEN $UGREEN missing"; ok=0; } ;; esac
    if [ "$CAN" = 1 ]; then ls $MKR_GLOB >/dev/null 2>&1 || { say "FAIL MKR serial port missing"; ok=0; }; fi
    [ -n "$BUILD" ] || { say "FAIL no build with lane_drive (run make in the container)"; ok=0; }
    [ -f "$REPO/models/culane_res18_fp16.engine" ] || { say "FAIL engine models/culane_res18_fp16.engine missing"; ok=0; }
    local free_gb need_gb
    free_gb=$(df -BG --output=avail "$ROOT" | tail -1 | tr -dc 0-9)
    need_gb=$(awk -v m="${MINUTES:-60}" 'BEGIN { printf "%d", (m / 60 + 1) * 20 }')
    [ "$free_gb" -ge "$need_gb" ] || { say "FAIL $free_gb GB free on $ROOT, want >= $need_gb"; ok=0; }
    # Devices held by anything else (root holders in containers included).
    local holders
    holders=$(docker run --rm --privileged --pid=host -v /dev:/dev "$IMAGE" \
              fuser -v /dev/video0 "$UGREEN" $MKR_GLOB 2>&1 | grep -vE 'USER|nvargus-daemon' | grep -E '[0-9]' || true)
    [ -z "$holders" ] || { say "FAIL devices in use:"; echo "$holders"; ok=0; }
    [ $ok = 1 ]
}

cmd_start() {
    MINUTES=""; CAMS="csi,usb"; CAN=1; PLAN=""; EXTRA=(); CSI_CROP=""; USB_CROP=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --minutes) [ $# -ge 2 ] || die "--minutes needs a value"; MINUTES="$2"; shift 2 ;;
            --cams)    [ $# -ge 2 ] || die "--cams needs a value"; CAMS="$2"; shift 2 ;;
            --no-can)  CAN=0; shift ;;
            --plan)    [ $# -ge 2 ] || die "--plan needs a value"; PLAN="$2"; shift 2 ;;
            --csi-crop) [ $# -ge 2 ] || die "--csi-crop needs T,B"; CSI_CROP="$2"; shift 2 ;;
            --usb-crop) [ $# -ge 2 ] || die "--usb-crop needs T,B"; USB_CROP="$2"; shift 2 ;;
            --)        shift; EXTRA=("$@"); break ;;
            *)         die "unknown option $1" ;;
        esac
    done
    [ -f "$CURRENT" ] && die "a run is active: $(cat "$CURRENT") (lane_run.sh stop)"
    BUILD="$(newest_build)"
    mkdir -p "$ROOT" || die "cannot create $ROOT"
    say "preflight"
    preflight || die "preflight failed"

    local n=1
    while ! mkdir "$(printf '%s/run_%03d' "$ROOT" $n)" 2>/dev/null; do n=$((n + 1)); done
    RUN="$(printf '%s/run_%03d' "$ROOT" $n)"
    mkdir -p "$RUN/sys" "$RUN/can"
    echo "$RUN" > "$CURRENT"
    echo "mono_ns,realtime_ns,source,type,detail" > "$RUN/events_host.csv"
    say "run $RUN (build $(basename "$BUILD"))"

    local sha
    sha=$(sha256sum "$REPO/models/culane_res18_fp16.engine" | cut -c1-64)
    cat > "$RUN/run_config.json" <<EOF
{
 "run": "$(basename "$RUN")",
 "plan": "$PLAN",
 "cams": "$CAMS",
 "can": $CAN,
 "minutes": "${MINUTES}",
 "extra_lane_drive_args": "${EXTRA[*]:-}",
 "csi_crop": "$CSI_CROP",
 "usb_crop": "$USB_CROP",
 "build": "$(basename "$BUILD")",
 "git_sha": "$(git -C "$REPO" rev-parse HEAD 2>/dev/null)",
 "git_dirty": $(git -C "$REPO" status --porcelain 2>/dev/null | grep -q . && echo true || echo false),
 "engine_sha256": "$sha",
 "kernel": "$(uname -r)",
 "nvpmodel": "$(nvpmodel -q 2>/dev/null | head -1)",
 "host_start_realtime": "$(date --iso-8601=ns)"
}
EOF
    event run_start "$CAMS"

    local rel="${BUILD#"$REPO"/}"
    for cam in ${CAMS//,/ }; do
        local crop=()
        [ "$cam" = csi ] && [ -n "$CSI_CROP" ] && crop=(--crop "$CSI_CROP")
        [ "$cam" = usb ] && [ -n "$USB_CROP" ] && crop=(--crop "$USB_CROP")
        docker rm -f "lane_$cam" >/dev/null 2>&1
        docker run -d --init --name "lane_$cam" --runtime nvidia --privileged --network=host --ipc=host \
            -v /dev:/dev -v /tmp/argus_socket:/tmp/argus_socket -v "$REPO":/user/dashcam -v "$RUN":/run_dir \
            -v /etc/localtime:/etc/localtime:ro -w /user/dashcam "$IMAGE" \
            "$rel/lane_drive" --cam "$cam" --out /run_dir ${MINUTES:+--minutes "$MINUTES"} "${crop[@]}" "${EXTRA[@]}" \
            > /dev/null && say "lane_$cam started" || say "lane_$cam FAILED to start"
        event container_start "lane_$cam"
    done
    if [ "$CAN" = 1 ]; then start_mkr 0; fi

    # tegrastats + sysmon on the host.
    tegrastats --interval 1000 --logfile "$RUN/sys/tegrastats.txt" > /dev/null 2>&1 & echo $! > "$RUN/sys/tegrastats.pid"
    python3 -B "$REPO/tools/lane_eval/sysmon.py" --out "$RUN/sys" --run-dir "$RUN" \
        --containers "$(for c in ${CAMS//,/ }; do printf 'lane_%s,' "$c"; done)lane_mkr" \
        > "$RUN/sys/sysmon.log" 2>&1 & echo $! > "$RUN/sys/sysmon.pid"

    # Supervisor: a dead CAN logger is restarted into can/partN; camera
    # containers have their own restart policy (lane_drive).
    (
        part=0
        while [ -f "$CURRENT" ]; do
            sleep 10
            [ -f "$CURRENT" ] || break
            if [ "$CAN" = 1 ] && ! running lane_mkr; then
                part=$((part + 1))
                event can_logger_dead "exit $(docker inspect -f '{{.State.ExitCode}}' lane_mkr 2>/dev/null)"
                start_mkr "$part"
            fi
            for cam in ${CAMS//,/ }; do
                running "lane_$cam" || { [ -f "$RUN/.ended_lane_$cam" ] || {
                    event container_exited "lane_$cam exit $(docker inspect -f '{{.State.ExitCode}}' "lane_$cam" 2>/dev/null)"
                    touch "$RUN/.ended_lane_$cam"; }; }
            done
        done
    ) > "$RUN/sys/supervisor.log" 2>&1 & echo $! > "$RUN/sys/supervisor.pid"
    say "started; stop with: tools/lane_eval/lane_run.sh stop"
}

start_mkr() {   # start_mkr PART
    local out="$RUN/can"
    [ "$1" -gt 0 ] && out="$RUN/can/part$1"
    mkdir -p "$out"
    docker rm -f lane_mkr >/dev/null 2>&1
    docker run -d --init --name lane_mkr --privileged -v /dev:/dev -v "$CANSTREAM":/canstream:ro \
        -v "$out":/out -v /etc/localtime:/etc/localtime:ro -e PYTHONDONTWRITEBYTECODE=1 "$IMAGE" \
        python3 /canstream/mkr_stream_log.py 43200 /out > /dev/null \
        && say "MKR logger started (part $1)" || say "MKR logger FAILED to start"
    event can_logger_start "part $1"
}

cmd_stop() {
    [ -f "$CURRENT" ] || die "no active run"
    RUN="$(cat "$CURRENT")"
    rm -f "$CURRENT"                                   # ends the supervisor loop
    kill "$(cat "$RUN/sys/supervisor.pid" 2>/dev/null)" 2>/dev/null
    say "stopping $RUN"
    event run_stop
    local present=()
    for c in $CONTAINERS; do docker inspect "$c" >/dev/null 2>&1 && present+=("$c"); done
    [ ${#present[@]} -gt 0 ] && docker stop -t 45 "${present[@]}" > /dev/null
    {
        echo "{"
        echo " \"host_end_realtime\": \"$(date --iso-8601=ns)\","
        echo " \"exit_codes\": {"
        local first=1
        for c in "${present[@]}"; do
            [ $first = 1 ] || echo ","
            first=0
            printf '  "%s": %s' "$c" "$(docker inspect -f '{{.State.ExitCode}}' "$c")"
        done
        echo; echo " }"; echo "}"
    } > "$RUN/run_end.json"
    for c in "${present[@]}"; do docker logs "$c" > "$RUN/sys/$c.log" 2>&1; docker rm "$c" > /dev/null; done
    for p in sysmon tegrastats; do kill -TERM "$(cat "$RUN/sys/$p.pid" 2>/dev/null)" 2>/dev/null; done
    event run_end
    say "exit codes: $(tr -d '\n' < "$RUN/run_end.json" | sed 's/.*"exit_codes": //')"
    summarize "$RUN"
}

summarize() {
    local run="$1"
    for s in "$run"/*/summary.json; do
        [ -f "$s" ] && say "$(basename "$(dirname "$s")"): $(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print("processed", d.get("processed"), "rows", d.get("frames_rows"), "drops", d.get("frames_csv_drops"), "restarts", d.get("restarts"), "segments", d.get("review_segments"), "stop_hang", d.get("stop_hang"))' "$s")"
    done
    [ -f "$run/can/mkr_console.txt" ] && say "can: $(tail -1 "$run/can/mkr_console.txt" | cut -c1-160)"
    say "size $(du -sh "$run" | cut -f1)"
}

cmd_status() {
    [ -f "$CURRENT" ] || { say "no active run"; return 0; }
    RUN="$(cat "$CURRENT")"
    say "run $RUN"
    for c in $CONTAINERS; do
        printf '  %-9s %s\n' "$c" "$(docker inspect -f '{{.State.Status}} (started {{.State.StartedAt}})' "$c" 2>/dev/null || echo absent)"
    done
    for f in "$RUN"/*/counters.csv; do
        [ -f "$f" ] || continue
        printf '  %-4s %s\n' "$(basename "$(dirname "$f")")" "$(tail -1 "$f" | awk -F, '{printf "state %s lanes %s processed %s eff_hz %.1f src_fps %.1f restarts %s", $2, $3, $7, $16, $17, $18}')"
    done
    [ -f "$RUN/sys/sysmon.csv" ] && tail -1 "$RUN/sys/sysmon.csv" | awk -F, '{printf "  sys  MemAvailable %.2f GB  GPU %s%% @%s MHz  run %s GB (%s GB/h)\n", $3/1048576, $6, $7, $(NF-1), $NF}'
}

case "${1:-}" in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *)      echo "usage: $0 start [--minutes N] [--cams csi,usb] [--no-can] [--plan day|night] [--csi-crop T,B] [--usb-crop T,B] [-- lane_drive args] | status | stop" >&2; exit 1 ;;
esac
