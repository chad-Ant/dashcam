#!/bin/bash
# lane_run.sh — orchestrates a lane test run (plan P5a + P5b, section 12): the
# per-camera lane_drive containers, the MKR raw-CAN logger, can_live.py (live
# CAN state + clip triggers), lane_supervisor.py (restarts, degrade ladder,
# status.json), the CYD / ES3C28P status display (lane_status), sysmon.py and
# tegrastats.
#
#   tools/lane_eval/lane_run.sh start [--minutes N] [--cams csi,usb] [--no-can] [--expect-can]
#                                     [--plan day|night] [--csi-crop T,B] [--usb-crop T,B]
#                                     [--mount-ref DIR] [--display DEV|auto|none] [--idle-lead S]
#                                     [--csi-args "..."] [--usb-args "..."] [-- args for both cameras]
#   tools/lane_eval/lane_run.sh status
#   tools/lane_eval/lane_run.sh mark "text"          operator marker (event + clip on both cameras)
#   tools/lane_eval/lane_run.sh stop
#   tools/lane_eval/lane_run.sh calibrate [--cams csi,usb] [--seconds 40]   parked stills + roi_calib.py
#
# Runs on the HOST as the jetson user (docker group).  dashcam-v04 and
# drive_session must not hold the cameras / MKR: start refuses while they do
# (stopping the service is a root step: sudo systemctl stop dashcam-v04).
# Output: /media/jetson/backup/lane_tests/run_NNN/ (outside the repo — the
# repo is public; LANE_ROOT overrides).  stop: docker stop -t 45 (SIGTERM →
# each lane_drive finalises and exits 0; 4 = finalised after a stop hang),
# exit codes recorded in run_end.json.
#
# --plan night: CSI review at 3 fps q70 (plan 12.8).  --expect-can: the car is
# on, so no CAN frames is an alert.  --display auto: the CYD's CH340 by-id
# port when present.  --idle-lead S: sysmon/tegrastats S seconds before the
# cameras start (B0's idle phase).

set -u
REPO="$(cd "$(dirname "$0")/../.." && pwd -P)"
ROOT="${LANE_ROOT:-/media/jetson/backup/lane_tests}"
CURRENT="$ROOT/.current"
IMAGE=l4t-ml-gpio
UGREEN=/dev/v4l/by-id/usb-Image+_UGREEN_Camera_4K_LL-0000000001-video-index0
MKR_GLOB="/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKRZero_*"
CYD=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
CANSTREAM="$REPO/peripherals/mkr_zero/tools/canstream"
CONTAINERS="lane_csi lane_usb lane_mkr lane_disp"

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
    case ",$CAMS," in *,csi,*) [ -e /dev/video0 ] || { say "FAIL /dev/video0 (IMX296) missing"; ok=0; } ;; esac
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
              fuser -v /dev/video0 "$UGREEN" $MKR_GLOB ${DISPLAY_DEV:+"$DISPLAY_DEV"} 2>&1 \
              | grep -vE 'USER|nvargus-daemon' | grep -E '[0-9]' || true)
    [ -z "$holders" ] || { say "FAIL devices in use:"; echo "$holders"; ok=0; }
    [ $ok = 1 ]
}

# start_cam RUN CAM INSTANCE [MINUTES] — the camera container, from the
# arguments saved at start (sys/args_<cam>.txt, one per line).
start_cam() {
    local run="$1" cam="$2" inst="$3" mins="${4:-}" build args=()
    build="$(cat "$run/sys/build.txt")"
    mapfile -t args < "$run/sys/args_$cam.txt"
    [ -n "$mins" ] && args+=(--minutes "$mins")
    [ "$inst" -gt 0 ] && args+=(--instance "$inst")
    docker rm -f "lane_$cam" >/dev/null 2>&1
    docker run -d --init --name "lane_$cam" --runtime nvidia --privileged --network=host --ipc=host \
        -v /dev:/dev -v /tmp/argus_socket:/tmp/argus_socket -v "$REPO":/user/dashcam -v "$run":/run_dir \
        -v /etc/localtime:/etc/localtime:ro -w /user/dashcam "$IMAGE" \
        "$build/lane_drive" --cam "$cam" --out /run_dir "${args[@]}" > /dev/null \
        && say "lane_$cam started (instance $inst)" || say "lane_$cam FAILED to start"
    RUN="$run" event container_start "lane_$cam instance $inst"
}

start_mkr() {   # start_mkr RUN PART
    local run="$1" out="$1/can"
    [ "$2" -gt 0 ] && out="$run/can/part$2"
    mkdir -p "$out"
    docker rm -f lane_mkr >/dev/null 2>&1
    docker run -d --init --name lane_mkr --privileged -v /dev:/dev -v "$CANSTREAM":/canstream:ro \
        -v "$out":/out -v /etc/localtime:/etc/localtime:ro -e PYTHONDONTWRITEBYTECODE=1 "$IMAGE" \
        python3 /canstream/mkr_stream_log.py 43200 /out > /dev/null \
        && say "MKR logger started (part $2)" || say "MKR logger FAILED to start"
    RUN="$run" event can_logger_start "part $2"
}

cmd_start() {
    MINUTES=""; CAMS="csi,usb"; CAN=1; EXPECT_CAN=0; PLAN=""; EXTRA=(); CSI_CROP=""; USB_CROP=""
    MOUNT_REF=""; DISPLAY_ARG=auto; IDLE_LEAD=0; CSI_ARGS=""; USB_ARGS=""
    while [ $# -gt 0 ]; do
        case "$1" in
            --minutes) [ $# -ge 2 ] || die "--minutes needs a value"; MINUTES="$2"; shift 2 ;;
            --cams)    [ $# -ge 2 ] || die "--cams needs a value"; CAMS="$2"; shift 2 ;;
            --no-can)  CAN=0; shift ;;
            --expect-can) EXPECT_CAN=1; shift ;;
            --plan)    [ $# -ge 2 ] || die "--plan needs a value"; PLAN="$2"; shift 2 ;;
            --csi-crop) [ $# -ge 2 ] || die "--csi-crop needs T,B"; CSI_CROP="$2"; shift 2 ;;
            --usb-crop) [ $# -ge 2 ] || die "--usb-crop needs T,B"; USB_CROP="$2"; shift 2 ;;
            --mount-ref) [ $# -ge 2 ] || die "--mount-ref needs a directory"; MOUNT_REF="$2"; shift 2 ;;
            --display) [ $# -ge 2 ] || die "--display needs DEV|auto|none"; DISPLAY_ARG="$2"; shift 2 ;;
            --idle-lead) [ $# -ge 2 ] || die "--idle-lead needs seconds"; IDLE_LEAD="$2"; shift 2 ;;
            --csi-args) [ $# -ge 2 ] || die "--csi-args needs a string"; CSI_ARGS="$2"; shift 2 ;;
            --usb-args) [ $# -ge 2 ] || die "--usb-args needs a string"; USB_ARGS="$2"; shift 2 ;;
            --)        shift; EXTRA=("$@"); break ;;
            *)         die "unknown option $1" ;;
        esac
    done
    case "$PLAN" in ""|day|night) ;; *) die "--plan day|night" ;; esac
    DISPLAY_DEV=""
    case "$DISPLAY_ARG" in
        none) ;;
        auto) [ -e "$CYD" ] && DISPLAY_DEV="$CYD" ;;
        *)    DISPLAY_DEV="$DISPLAY_ARG" ;;
    esac
    [ -f "$CURRENT" ] && die "a run is active: $(cat "$CURRENT") (lane_run.sh stop)"
    BUILD="$(newest_build)"
    mkdir -p "$ROOT" || die "cannot create $ROOT"
    say "preflight"
    preflight || die "preflight failed"

    local n=1
    while ! mkdir "$(printf '%s/run_%03d' "$ROOT" $n)" 2>/dev/null; do n=$((n + 1)); done
    RUN="$(printf '%s/run_%03d' "$ROOT" $n)"
    mkdir -p "$RUN/sys" "$RUN/can" "$RUN/ctl"
    echo "$RUN" > "$CURRENT"
    echo "mono_ns,realtime_ns,source,type,detail" > "$RUN/events_host.csv"
    echo "mono_ns,source,type,priority,detail" > "$RUN/triggers.csv"
    local rel="${BUILD#"$REPO"/}"
    echo "$rel" > "$RUN/sys/build.txt"
    echo "$CAMS" > "$RUN/sys/cams.txt"
    docker rm lane_csi lane_usb lane_mkr lane_disp >/dev/null 2>&1   # exited leftovers of earlier runs
    say "run $RUN (build $(basename "$BUILD"))"

    # Per-camera lane_drive arguments (kept for supervisor restarts).
    local cam
    for cam in ${CAMS//,/ }; do
        {
            [ "$cam" = csi ] && [ -n "$CSI_CROP" ] && printf '%s\n' --crop "$CSI_CROP"
            [ "$cam" = usb ] && [ -n "$USB_CROP" ] && printf '%s\n' --crop "$USB_CROP"
            [ "$cam" = csi ] && [ "$PLAN" = night ] && printf '%s\n' --review-fps 3 --jpeg-q 70
            [ -n "$MOUNT_REF" ] && printf '%s\n' --mount-ref "$MOUNT_REF"
            [ "$cam" = csi ] && [ -n "$CSI_ARGS" ] && printf '%s\n' $CSI_ARGS
            [ "$cam" = usb ] && [ -n "$USB_ARGS" ] && printf '%s\n' $USB_ARGS
            [ ${#EXTRA[@]} -gt 0 ] && printf '%s\n' "${EXTRA[@]}"
        } > "$RUN/sys/args_$cam.txt"
    done

    local sha
    sha=$(sha256sum "$REPO/models/culane_res18_fp16.engine" | cut -c1-64)
    cat > "$RUN/run_config.json" <<EOF
{
 "run": "$(basename "$RUN")",
 "plan": "$PLAN",
 "cams": "$CAMS",
 "can": $CAN,
 "expect_can": $EXPECT_CAN,
 "minutes": "${MINUTES}",
 "extra_lane_drive_args": "${EXTRA[*]:-}",
 "csi_args": "$CSI_ARGS",
 "usb_args": "$USB_ARGS",
 "csi_crop": "$CSI_CROP",
 "usb_crop": "$USB_CROP",
 "mount_ref": "$MOUNT_REF",
 "display": "$DISPLAY_DEV",
 "idle_lead_s": $IDLE_LEAD,
 "build": "$(basename "$BUILD")",
 "git_sha": "$(git -C "$REPO" rev-parse HEAD 2>/dev/null)",
 "git_dirty": $(git -C "$REPO" status --porcelain 2>/dev/null | grep -q . && echo true || echo false),
 "engine_sha256": "$sha",
 "allow_xfail": "B5,B6,B8",
 "kernel": "$(uname -r)",
 "nvpmodel": "$(nvpmodel -q 2>/dev/null | head -1)",
 "host_start_realtime": "$(date --iso-8601=ns)"
}
EOF
    event run_start "$CAMS"

    # tegrastats + sysmon on the host.
    tegrastats --interval 1000 --logfile "$RUN/sys/tegrastats.txt" > /dev/null 2>&1 & echo $! > "$RUN/sys/tegrastats.pid"
    python3 -B "$REPO/tools/lane_eval/sysmon.py" --out "$RUN/sys" --run-dir "$RUN" \
        --containers "$(for c in ${CAMS//,/ }; do printf 'lane_%s,' "$c"; done)lane_mkr,lane_disp" \
        > "$RUN/sys/sysmon.log" 2>&1 & echo $! > "$RUN/sys/sysmon.pid"
    if [ "$IDLE_LEAD" -gt 0 ]; then
        say "idle lead-in ${IDLE_LEAD} s"; event idle_lead "$IDLE_LEAD"; sleep "$IDLE_LEAD"
    fi

    if [ "$CAN" = 1 ]; then
        start_mkr "$RUN" 0
        python3 -B "$REPO/tools/lane_eval/can_live.py" --run "$RUN" > "$RUN/sys/can_live.log" 2>&1 &
        echo $! > "$RUN/sys/can_live.pid"
    fi
    # Cameras: CSI first, USB 10 s later (the engine loads must not peak RAM together).
    local first=1
    for cam in ${CAMS//,/ }; do
        [ $first = 1 ] || sleep 10
        first=0
        start_cam "$RUN" "$cam" 0 "$MINUTES"
    done
    if [ -n "$DISPLAY_DEV" ]; then
        docker rm -f lane_disp >/dev/null 2>&1
        docker run -d --init --name lane_disp --privileged -v /dev:/dev -v "$REPO":/user/dashcam -v "$RUN":/run_dir \
            -w /user/dashcam "$IMAGE" "$rel/lane_status" --device "$DISPLAY_DEV" --status-file /run_dir/status.json \
            > /dev/null && say "status display on $DISPLAY_DEV" || say "status display FAILED to start"
    fi
    python3 -B "$REPO/tools/lane_eval/lane_supervisor.py" --run "$RUN" --cams "$CAMS" \
        $([ "$CAN" = 1 ] && echo --can) $([ "$EXPECT_CAN" = 1 ] && echo --expect-can) \
        ${MINUTES:+--minutes "$MINUTES"} > "$RUN/sys/supervisor.log" 2>&1 & echo $! > "$RUN/sys/supervisor.pid"
    say "started; the first 120 s run with lanes off — wait for 'ready' in: tools/lane_eval/lane_run.sh status"
}

cmd_stop() {
    [ -f "$CURRENT" ] || die "no active run"
    RUN="$(cat "$CURRENT")"
    rm -f "$CURRENT"
    say "stopping $RUN"
    event run_stop
    # The supervisor first (it must not restart what we stop), then the cameras
    # (they finalise: EOS, clips, dumps), then CAN, the display and monitors.
    kill -TERM "$(cat "$RUN/sys/supervisor.pid" 2>/dev/null)" 2>/dev/null
    local present=() c cams
    cams="$(cat "$RUN/sys/cams.txt" 2>/dev/null || echo csi,usb)"
    for c in ${cams//,/ }; do docker inspect "lane_$c" >/dev/null 2>&1 && present+=("lane_$c"); done
    [ ${#present[@]} -gt 0 ] && docker stop -t 45 "${present[@]}" > /dev/null
    kill -TERM "$(cat "$RUN/sys/can_live.pid" 2>/dev/null)" 2>/dev/null
    for c in lane_mkr lane_disp; do docker inspect "$c" >/dev/null 2>&1 && { docker stop -t 15 "$c" > /dev/null; present+=("$c"); }; done
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
        [ -f "$s" ] && say "$(basename "$(dirname "$s")"): $(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print("processed", d.get("processed"), "rows", d.get("frames_rows"), "drops", d.get("frames_csv_drops"), "restarts", d.get("restarts"), "segments", d.get("review_segments"), "clips", d.get("clips_written"), "dumps", d.get("dumps_written"), "mount", d.get("mount"), "stop_hang", d.get("stop_hang"))' "$s")"
    done
    [ -f "$run/can/mkr_console.txt" ] && say "can: $(tail -1 "$run/can/mkr_console.txt" | cut -c1-160)"
    say "size $(du -sh "$run" | cut -f1)"
}

cmd_status() {
    [ -f "$CURRENT" ] || { say "no active run"; return 0; }
    RUN="$(cat "$CURRENT")"
    say "run $RUN"
    local c st
    for c in $CONTAINERS; do
        st="$(docker inspect -f '{{.State.Status}} (started {{.State.StartedAt}})' "$c" 2>/dev/null | head -1)"
        [ -n "$st" ] && printf '  %-9s %s\n' "$c" "$st"
    done
    for f in "$RUN"/*/counters.csv; do
        [ -f "$f" ] || continue
        printf '  %-4s %s\n' "$(basename "$(dirname "$f")")" "$(tail -1 "$f" | awk -F, '{printf "state %s lanes %s processed %s eff_hz %.1f src_fps %.1f restarts %s clips %s dumps %s degrade %s", $2, $3, $7, $16, $17, $18, $26, $30, $33}')"
    done
    [ -f "$RUN/sys/sysmon.csv" ] && tail -1 "$RUN/sys/sysmon.csv" | awk -F, '{printf "  sys  MemAvailable %.2f GB  GPU %s%% @%s MHz  run %s GB (%s GB/h)\n", $3/1048576, $6, $7, $(NF-1), $NF}'
    [ -f "$RUN/status.json" ] && python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))
print("  %s%s  %s" % (d["overall"], " (ready)" if d.get("ready") else " (not ready)", d.get("disp_title")))
for k in ("disp_l1", "disp_l2", "disp_l3", "disp_l4"):
    print("    " + d.get(k, ""))
for a in d.get("alerts", []):
    print("  ALERT " + a)' "$RUN/status.json"
}

cmd_mark() {
    [ -f "$CURRENT" ] || die "no active run"
    RUN="$(cat "$CURRENT")"
    local text="${*:-mark}" mono
    mono=$(python3 -c 'import time; print(time.monotonic_ns())')
    echo "$mono,operator,mark,1,${text//,/;}" >> "$RUN/triggers.csv"
    event mark "${text//,/;}"
    say "mark: $text"
}

# calibrate: parked stills from each camera (lane_drive --stills, lanes on at
# once, no clips) into calib/CAL_NNN/, then roi_calib.py in the container.
cmd_calibrate() {
    CAMS="csi,usb"; local secs=40
    while [ $# -gt 0 ]; do
        case "$1" in
            --cams) CAMS="$2"; shift 2 ;;
            --seconds) secs="$2"; shift 2 ;;
            *) die "unknown option $1" ;;
        esac
    done
    [ -f "$CURRENT" ] && die "a run is active: $(cat "$CURRENT")"
    CAN=0; MINUTES=1; DISPLAY_DEV=""
    BUILD="$(newest_build)"
    preflight || die "preflight failed"
    mkdir -p "$ROOT/calib"
    local n=1
    while ! mkdir "$(printf '%s/calib/CAL_%03d' "$ROOT" $n)" 2>/dev/null; do n=$((n + 1)); done
    local cal; cal="$(printf '%s/calib/CAL_%03d' "$ROOT" $n)"
    local rel="${BUILD#"$REPO"/}" cam
    say "calibration $cal (${secs} s per camera, both at once)"
    for cam in ${CAMS//,/ }; do
        docker rm -f "lane_$cam" >/dev/null 2>&1
        docker run -d --init --name "lane_$cam" --runtime nvidia --privileged --network=host --ipc=host \
            -v /dev:/dev -v /tmp/argus_socket:/tmp/argus_socket -v "$REPO":/user/dashcam -v "$cal":/run_dir \
            -w /user/dashcam "$IMAGE" "$rel/lane_drive" --cam "$cam" --out /run_dir --lanes-off-s 0 --no-clips \
            --tensor-hz 0 --no-event-dumps --stills 10 --mount-ref-s 10 --random-clip-min 0 \
            --minutes "$(awk -v s="$secs" 'BEGIN { print s / 60 }')" > /dev/null
    done
    for cam in ${CAMS//,/ }; do docker wait "lane_$cam" > /dev/null; docker logs "lane_$cam" > "$cal/lane_$cam.log" 2>&1; docker rm "lane_$cam" > /dev/null; done
    docker run --rm -v "$REPO":/user/dashcam -v "$cal":/cal -w /user/dashcam "$IMAGE" \
        python3 -B tools/lane_eval/roi_calib.py /cal --cams "$CAMS" | tee "$cal/roi_calib.txt"
    say "calibration written to $cal (overlays: $cal/<cam>_overlay.jpg; start with --mount-ref $cal/mount)"
}

case "${1:-}" in
    start)      shift; cmd_start "$@" ;;
    stop)       cmd_stop ;;
    status)     cmd_status ;;
    mark)       shift; cmd_mark "$@" ;;
    calibrate)  shift; cmd_calibrate "$@" ;;
    _start_cam) shift; start_cam "$@" ;;
    _start_mkr) shift; start_mkr "$@" ;;
    *)          echo "usage: $0 start [--minutes N] [--cams csi,usb] [--no-can] [--expect-can] [--plan day|night] [--csi-crop T,B] [--usb-crop T,B] [--mount-ref DIR] [--display DEV|auto|none] [--idle-lead S] [--csi-args ...] [--usb-args ...] [-- lane_drive args] | status | mark TEXT | stop | calibrate [--cams ...] [--seconds N]" >&2; exit 1 ;;
esac
