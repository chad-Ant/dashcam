#!/usr/bin/env bash
# Offline wrapper: source/captures read-only; only v2/results writable.
set -euo pipefail
chart_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
tuning_dir=$(cd -- "$chart_dir/../.." && pwd)
if [[ $# -eq 0 ]]; then
    printf 'Usage: bash %s colour|noise|detail --images "ABSOLUTE_GLOB" [--reference "ABSOLUTE_GLOB"]\n' "$0"
    exit 2
fi
for arg in "$@"; do
    if [[ "$arg" == --out || "$arg" == --out=* ]]; then
        printf '%s\n' 'The wrapper selects a fresh output under v2/results; omit --out.' >&2
        exit 2
    fi
done
mkdir -p -- "$chart_dir/results"
# Reserve a unique parent while leaving the report child nonexistent for Python.
run_parent=$(mktemp -d "$chart_dir/results/run_$(date +%Y%m%d_%H%M%S).XXXXXX")
printf 'Results: %s/report\n' "$run_parent"
exec docker run --rm --runtime=runc --network=none \
    --user "$(id -u):$(id -g)" \
    --env PYTHONDONTWRITEBYTECODE=1 --env OPENBLAS_NUM_THREADS=1 \
    -v /home/jetson/drive_logs:/home/jetson/drive_logs:ro \
    -v "$tuning_dir:$tuning_dir:ro" \
    -v "$run_parent:$run_parent:rw" \
    l4t-ml-gpio:latest python3 "$chart_dir/calibrate.py" "$@" --out "$run_parent/report"
