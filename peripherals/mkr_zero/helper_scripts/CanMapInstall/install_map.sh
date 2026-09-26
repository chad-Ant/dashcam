#!/bin/bash
#
# install_map.sh VEHICLE [PORT|auto] - put config/canmap.VEHICLE.txt on the MKR
# Zero's SD card through the CanMapInstall helper, then print its report.
# Re-flash the production firmware afterwards (build_and_upload.sh auto).
#
# Runs inside the dev container like build_and_upload.sh:
#   docker run --rm --privileged -v /dev:/dev -v ~/dashcam:/user/dashcam l4t-ml-gpio:latest \
#     /user/dashcam/peripherals/mkr_zero/helper_scripts/CanMapInstall/install_map.sh brio auto
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MKR="$(cd "$HERE/../.." && pwd)"
VEH="${1:?vehicle, e.g. brio}"; PORT="${2:-auto}"
MAP="$MKR/config/canmap.$VEH.txt"
[ -f "$MAP" ] || { echo "no $MAP"; exit 1; }
grep -q ')CANMAP"' "$MAP" && { echo "map text contains the raw-string delimiter"; exit 1; }

# The map rides in the image as a raw string. Generated, never committed.
{ printf 'static const char kMapVehicle[] = "%s";\n' "$VEH"
  printf 'static const char kMapText[] = R"CANMAP('; cat "$MAP"; printf ')CANMAP";\n'; } > "$HERE/canmap_text.h"
trap 'rm -f "$HERE/canmap_text.h"' EXIT

if [ "$PORT" = "auto" ]; then
    source "$MKR/find_port.sh"
    PORT=$(find_mkr_zero_port) || exit 1
fi
arduino-cli compile --warnings all --fqbn arduino:samd:mkrzero --build-path /tmp/canmapinstall \
    --library "$MKR/vendor/Wire" --library "$MKR/vendor/CANBus" --library "$MKR/vendor/SdFat" --library "$MKR/lib" \
    --upload --port "$PORT" "$HERE"
