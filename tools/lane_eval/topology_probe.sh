#!/bin/bash
# topology_probe.sh — RAM / latency cost of 1 vs 2 concurrent lane engines
# (lane test suite P0a).  Runs INSIDE the l4t-ml-gpio container:
#
#   docker run --rm --runtime nvidia -v /home/jetson/dashcam:/user/dashcam \
#       -v /home/jetson/drive_logs/lane_engines:/user/lane_engines \
#       -w /user/dashcam l4t-ml-gpio \
#       tools/lane_eval/topology_probe.sh --out /user/lane_engines/topology \
#           E0=models/culane_res18_fp16.engine E1=/user/lane_engines/E1_fp16_forced.engine
#
# For each engine: a bare CUDA context, then 1 and 2 concurrent trtexec
# processes (each one = one camera's detector in topology (b), one process per
# camera), paced at --hz (default 20, like LaneTargetHz) via --idleTime.
# /proc/meminfo inside a container is the host's, so MemAvailable is the real
# system figure.  Sampled at 10 Hz: MemAvailable and GPU load.  Reports the
# MemAvailable drop at steady state (baseline minus median during the run)
# and each process's latency percentiles.  CUDA/nvmap memory is not charged
# to RSS on Jetson, which is why MemAvailable is the measure here.
#
# Not measured here: the camera pipelines (Argus, VIC, NVJPG, MJPEG decode)
# — that is the live harness's B3 job.

set -u
OUT=""
HZ=20
DUR=30
SPECS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --out) [ $# -ge 2 ] || { echo "--out needs a value" >&2; exit 1; }; OUT="$2"; shift 2 ;;
        --hz)  [ $# -ge 2 ] || { echo "--hz needs a value" >&2; exit 1; }; HZ="$2"; shift 2 ;;
        --duration) [ $# -ge 2 ] || { echo "--duration needs a value" >&2; exit 1; }; DUR="$2"; shift 2 ;;
        *)     SPECS+=("$1"); shift ;;
    esac
done
[ -n "$OUT" ] && [ ${#SPECS[@]} -gt 0 ] || { echo "usage: $0 --out DIR [--hz N] [--duration S] NAME=engine ..." >&2; exit 1; }

REPO_ROOT="$(realpath "$(dirname "$0")/../..")"
OUT="$(realpath -m "$OUT")"             # resolved before anything is created
case "$OUT/" in
    "$REPO_ROOT"/*|/user/dashcam/*|/home/jetson/Documents/github_repos/dashcam/*)
        echo "ERROR: --out $OUT is inside the repo; refusing" >&2; exit 1 ;;
esac
OWNER="$(stat -c %u:%g "$(dirname "$OUT")" 2>/dev/null)"
NEW_OUT=1; [ -d "$OUT" ] && NEW_OUT=0
mkdir -p "$OUT" || exit 1

T=/usr/src/tensorrt/bin/trtexec
[ -x "$T" ] || { echo "ERROR: $T not found (run in the container)" >&2; exit 1; }
for spec in "${SPECS[@]}"; do
    [ -f "${spec#*=}" ] || { echo "ERROR: engine ${spec#*=} not found" >&2; exit 1; }
done
RC=0
GPU_LOAD=/sys/devices/platform/gpu.0/load

memavail_kb() { awk '/^MemAvailable:/ {print $2}' /proc/meminfo; }

GPU_FREQ=/sys/class/devfreq/17000000.gpu/cur_freq

# sample FILE — 10 Hz "t_s memavail_kb gpu_load_permille gpu_hz" until killed
sample() {
    while :; do
        echo "$(date +%s.%N) $(memavail_kb) $(cat $GPU_LOAD 2>/dev/null || echo -1) $(cat $GPU_FREQ 2>/dev/null || echo -1)"
        sleep 0.1
    done > "$1"
}

# summarise FILE BASE_KB LABEL — MemAvailable drop (median over the middle
# half of the run, and peak), mean/max GPU load, median GPU clock.
summarise() {
    python3 - "$1" "$2" "$3" <<'PYEOF'
import statistics, sys
path, base, label = sys.argv[1], int(sys.argv[2]), sys.argv[3]
rows = [l.split() for l in open(path) if len(l.split()) == 4]
m = [int(r[1]) for r in rows]; g = [int(r[2]) for r in rows]; f = [int(r[3]) for r in rows]
mid = m[len(m) // 4: 3 * len(m) // 4] or m
print(f"{label:26s} MemAvail drop median {(base - statistics.median(mid)) / 1024:5.0f} MB"
      f"  peak {(base - min(m)) / 1024:5.0f} MB   GPU load mean {statistics.mean(g) / 10:4.1f}%"
      f" max {max(g) / 10:4.1f}%   GPU clock median {statistics.median(f) / 1e6:4.0f} MHz")
PYEOF
}

latency() {   # trtexec log -> "mean p99" of end-to-end latency
    grep -E '\] Latency:' "$1" | sed -E 's/.*mean = ([0-9.]+) ms.*percentile\(99%\) = ([0-9.]+) ms.*/mean \1 ms  p99 \2 ms/'
}

# trtexec busy time per inference ~ GPU compute; idleTime spaces iterations
# so each process runs at roughly HZ (slightly under: idle + compute).
IDLE_MS=$(awk -v hz="$HZ" 'BEGIN { printf "%d", 1000/hz }')

echo "host: $(uname -r)  nvpmodel: $(cat /etc/nvpmodel.conf 2>/dev/null | grep -m1 -o 'PM_CONFIG DEFAULT=[0-9]*' || echo '?')"
echo "pacing: idleTime ${IDLE_MS} ms (~${HZ} Hz minus compute), ${DUR} s per case"

# Bare CUDA context cost (what a second process pays before any engine).
BASE=$(memavail_kb)
python3 - <<'EOF' &
import ctypes, time
ctypes.CDLL("libcudart.so").cudaFree(None)
time.sleep(4)
EOF
CP=$!
sleep 3
CTX=$(memavail_kb)
wait $CP || { echo "WARN: bare CUDA context probe failed" >&2; RC=1; }
printf "%-26s MemAvail drop %5.0f MB\n" "cuda context (bare)" $(( (BASE - CTX) / 1024 ))

for spec in "${SPECS[@]}"; do
    NAME="${spec%%=*}"; ENG="${spec#*=}"
    for N in 1 2; do
        sleep 3
        BASE=$(memavail_kb)
        S="$OUT/${NAME}_x${N}.samples"
        sample "$S" & SP=$!
        PIDS=()
        for i in $(seq 1 $N); do
            "$T" --loadEngine="$ENG" --idleTime="$IDLE_MS" --duration="$DUR" --warmUp=1000 \
                > "$OUT/${NAME}_x${N}_p${i}.log" 2>&1 &
            PIDS+=($!)
        done
        FAILED=0
        for p in "${PIDS[@]}"; do wait "$p" || FAILED=1; done
        kill $SP 2>/dev/null; wait $SP 2>/dev/null
        summarise "$S" "$BASE" "${NAME} x${N}"
        for i in $(seq 1 $N); do
            LAT="$(latency "$OUT/${NAME}_x${N}_p${i}.log")"
            [ -n "$LAT" ] || FAILED=1
            echo "    p$i: ${LAT:-no latency line}"
        done
        if [ $FAILED -ne 0 ]; then
            echo "    INVALID: a trtexec run failed (see $OUT/${NAME}_x${N}_p*.log)"
            RC=1
        fi
    done
done
# Hand the results to the owner of OUT's parent (never to root).
if [ -n "$OWNER" ] && [ "${OWNER%%:*}" != 0 ]; then
    if [ $NEW_OUT -eq 1 ]; then chown -R "$OWNER" "$OUT" 2>/dev/null
    else chown "$OWNER" "$OUT"/*.samples "$OUT"/*.log 2>/dev/null; fi
fi
exit $RC
