#!/bin/bash
#
# build_and_upload.sh [PORT|auto] - build the BusFaultInjection helper for the
# MKR Zero on the Jetson, instrumented (DASHCAM_WIRE_INSTRUMENT) so its 'c'
# control command can route transfers through the core's unbounded waits.
# Re-flash the production firmware afterwards (../../build_and_upload.sh auto).
#
# Runs inside the dev container like the production script:
#   docker run --rm --privileged -v /dev:/dev -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
#     /user/dashcam/peripherals/mkr_zero/helper_scripts/BusFaultInjection/build_and_upload.sh auto
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MKR="$(cd "$HERE/../.." && pwd)"
PORT="${1:-}"
# vendor/Wire needs arduino:samd 1.8.14 exactly (see check_core.sh).
source "$MKR/check_core.sh"
require_samd_core || exit 1
if [ "$PORT" = "auto" ]; then
    source "$MKR/find_port.sh"
    PORT=$(find_mkr_zero_port) || exit 1
    echo "upload port: $PORT"
fi
args=(compile --warnings all --fqbn arduino:samd:mkrzero --build-path /tmp/busfaultinjection
      --build-property "compiler.cpp.extra_flags=-DDASHCAM_WIRE_INSTRUMENT"
      --library "$MKR/vendor/Wire" --library "$MKR/vendor/CANBus" --library "$MKR/vendor/SdFat" --library "$MKR/lib")
[ -n "$PORT" ] && args+=(--upload --port "$PORT")
exec arduino-cli "${args[@]}" "$HERE"
