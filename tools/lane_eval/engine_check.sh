#!/bin/bash
# engine_check.sh — facts about UFLD v2 TensorRT engines (lane test suite P0a).
#
# Runs INSIDE the l4t-ml-gpio container (TRT engines are locked to the TRT
# version that built them, and only the container's TRT runs our engines):
#
#   docker run --rm --runtime nvidia \
#       -v /home/jetson/dashcam:/user/dashcam \
#       -v /home/jetson/drive_logs/lane_engines:/user/lane_engines \
#       -w /user/dashcam l4t-ml-gpio \
#       tools/lane_eval/engine_check.sh --out /user/lane_engines/check \
#           models/culane_res18_fp16.engine [more.engine ...]
#
# Per engine it records sha256, size, the plan's TRT version, the container's
# TRT/CUDA versions, every IO tensor (shape, dtype, format) and the device
# memory the engine needs, then (with --profile) runs trtexec for per-layer
# info, a per-layer profile and a timing summary.  Results land in
# OUT/<engine-basename>/ plus one line per engine in OUT/manifest.jsonl.
#
# OUT must be outside the repo: the repo is public and engines/profiles are
# build artefacts.  Exit 0 = all engines checked, 1 = usage/guard error,
# 2 = an engine failed to load.

set -u

usage() {
    echo "usage: $0 --out DIR [--profile] [--duration S] ENGINE [ENGINE ...]" >&2
    exit 1
}

OUT=""
PROFILE=0
DURATION=10
ENGINES=()
while [ $# -gt 0 ]; do
    case "$1" in
        --out)      [ $# -ge 2 ] || usage; OUT="$2"; shift 2 ;;
        --profile)  PROFILE=1; shift ;;
        --duration) [ $# -ge 2 ] || usage; DURATION="$2"; shift 2 ;;
        -h|--help)  usage ;;
        *)          ENGINES+=("$1"); shift ;;
    esac
done
[ -n "$OUT" ] && [ ${#ENGINES[@]} -gt 0 ] || usage

# Output-path guard: refuse anything inside the repo (realpath prefix check;
# git is not usable here — it refuses the root-in-container uid mismatch).
REPO_ROOT="$(realpath "$(dirname "$0")/../..")"
OUT_REAL="$(realpath -m "$OUT")"        # resolved before anything is created
case "$OUT_REAL/" in
    "$REPO_ROOT"/*|/user/dashcam/*|/home/jetson/Documents/github_repos/dashcam/*)
        echo "ERROR: --out $OUT_REAL is inside the repo ($REPO_ROOT); refusing" >&2
        exit 1 ;;
esac
mkdir -p "$OUT_REAL" || exit 1
# Owner for the results: whoever owns OUT's parent, unless that is root.
OWNER="$(stat -c %u:%g "$(dirname "$OUT_REAL")")"
CREATED=()

TRTEXEC=/usr/src/tensorrt/bin/trtexec
[ -x "$TRTEXEC" ] || { echo "ERROR: $TRTEXEC not found (run in the container)" >&2; exit 1; }

RC=0
for ENGINE in "${ENGINES[@]}"; do
    if [ ! -f "$ENGINE" ]; then
        echo "ERROR: $ENGINE: no such file" >&2
        RC=2
        continue
    fi
    NAME="$(basename "$ENGINE" .engine)"
    DIR="$OUT_REAL/$NAME"
    mkdir -p "$DIR"
    CREATED+=("$DIR")
    echo "=== $ENGINE -> $DIR"

    # Identity and IO introspection.  The python block writes one JSON object
    # to facts.json (also appended to manifest.jsonl) and exits non-zero if
    # the plan does not deserialize in this container.  TRT's own messages
    # (e.g. the "built on a different device model" warning) go to
    # load.log — the TRT Python logger prints to stdout, so stdout can't
    # carry the JSON.
    python3 - "$ENGINE" "$DIR/facts.json" > "$DIR/load.log" 2>&1 <<'EOF'
import hashlib, json, os, sys
import tensorrt as trt

path, out = sys.argv[1], sys.argv[2]

def emit(facts):
    with open(out, "w") as f:
        json.dump(facts, f, indent=1)

h = hashlib.sha256()
with open(path, "rb") as f:
    head = f.read(32)
    h.update(head)
    for chunk in iter(lambda: f.read(1 << 22), b""):
        h.update(chunk)

facts = {
    "engine": os.path.realpath(path),
    "size_bytes": os.path.getsize(path),
    "sha256": h.hexdigest(),
    "header_hex": head.hex(),
    "container_trt": trt.__version__,
}
try:
    import ctypes
    v = ctypes.c_int()
    ctypes.CDLL("libcudart.so").cudaRuntimeGetVersion(ctypes.byref(v))
    facts["container_cudart"] = v.value
except OSError:
    facts["container_cudart"] = None

logger = trt.Logger(trt.Logger.WARNING)
runtime = trt.Runtime(logger)
with open(path, "rb") as f:
    engine = runtime.deserialize_cuda_engine(f.read())
if engine is None:
    facts["load_ok"] = False
    emit(facts)
    sys.exit(2)

facts["load_ok"] = True
facts["device_memory_bytes"] = int(engine.device_memory_size_v2) \
    if hasattr(engine, "device_memory_size_v2") else int(engine.device_memory_size)
facts["num_layers"] = engine.num_layers
io = []
for i in range(engine.num_io_tensors):
    n = engine.get_tensor_name(i)
    io.append({
        "name": n,
        "mode": str(engine.get_tensor_mode(n)).split(".")[-1],
        "shape": list(engine.get_tensor_shape(n)),
        "dtype": str(engine.get_tensor_dtype(n)).split(".")[-1],
        "format": str(engine.get_tensor_format(n)).split(".")[-1],
    })
facts["io"] = io
emit(facts)
EOF
    PYRC=$?
    "$TRTEXEC" --loadEngine="$ENGINE" --getPlanVersionOnly > "$DIR/plan_version.txt" 2>&1
    python3 -c 'import json,sys; print(json.dumps(json.load(open(sys.argv[1]))))' \
        "$DIR/facts.json" >> "$OUT_REAL/manifest.jsonl" 2>/dev/null
    if [ $PYRC -ne 0 ]; then
        echo "ERROR: $ENGINE did not deserialize (see $DIR/facts.json)" >&2
        RC=2
        continue
    fi
    grep -E '"(size_bytes|sha256|device_memory_bytes|load_ok)"' "$DIR/facts.json"
    grep -F '[TRT]' "$DIR/load.log"

    if [ $PROFILE -eq 1 ]; then
        # Layer info (precision only appears for engines built with
        # --profilingVerbosity=detailed), per-layer profile in a separate run
        # so the benchmark numbers are not profiler-inflated, timing summary.
        "$TRTEXEC" --loadEngine="$ENGINE" \
            --dumpLayerInfo --exportLayerInfo="$DIR/layers.json" \
            --dumpProfile --separateProfileRun --exportProfile="$DIR/profile.json" \
            --exportTimes="$DIR/times.json" \
            --warmUp=2000 --duration="$DURATION" \
            > "$DIR/trtexec.log" 2>&1 || RC=2
        grep -E 'GPU Compute Time|Throughput|Latency:' "$DIR/trtexec.log" | sed 's/^\[[^]]*\] \[I\] //'
    fi
done
# The container runs as root; hand what this run wrote to whoever owns OUT's
# parent (the jetson user for ~/drive_logs) so it can be edited and removed.
# Only this run's files, and never to root.
if [ "${OWNER%%:*}" != 0 ]; then
    chown "$OWNER" "$OUT_REAL" "$OUT_REAL/manifest.jsonl" 2>/dev/null
    for d in "${CREATED[@]}"; do chown -R "$OWNER" "$d" 2>/dev/null; done
fi
exit $RC
