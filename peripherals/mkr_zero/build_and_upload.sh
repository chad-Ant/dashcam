#!/bin/bash
#
# build_and_upload.sh — build / flash the MKR Zero telemetry master from Linux
# (the Jetson's l4t-ml-gpio container).  Linux twin of BuildAndUpload.cmd:
# same FQBN, same pinned core, same --library flags, same warning level — keep
# the two in step (the reasons for every flag are in BuildAndUpload.cmd).
#
#   ./build_and_upload.sh                    compile only
#   ./build_and_upload.sh /dev/ttyACM1       compile and upload
#   ./build_and_upload.sh auto               compile and upload to the MKR Zero
#                                            found by its USB identity
#   ./build_and_upload.sh [PORT|auto] amg    ...with the IMU in its raw (AMG)
#                                            mode; "amg" also works alone
#
# From the host (the image carries arduino-cli, arduino:samd 1.8.14 and the
# pinned libraries — see docker_dev/Dockerfile):
#   docker run --rm --privileged -v /dev:/dev -v ~/dashcam:/user/dashcam \
#     l4t-ml-gpio:latest /user/dashcam/peripherals/mkr_zero/build_and_upload.sh auto
#
# The MKR's USB port is a DEVELOPMENT link only (flashing + its Serial
# console); in production the board talks to the Jetson solely through the
# ESP32-C3 bridge (Serial1).  Upload resets the board into its SAM-BA
# bootloader (1200-baud touch) and it re-enumerates, possibly as a different
# ttyACM; if it never shows up, double-tap RESET and run "auto" again.  Close
# anything holding the MKR's port first (picocom, a serial monitor).
set -euo pipefail

SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$SKETCH_DIR/lib"
WIRE_DIR="$SKETCH_DIR/vendor/Wire"
CAN_DIR="$SKETCH_DIR/vendor/CANBus"
SDFAT_DIR="$SKETCH_DIR/vendor/SdFat"

FQBN="arduino:samd:mkrzero"
CORE_REQUIRED="1.8.14"

command -v arduino-cli >/dev/null || { echo "arduino-cli is not available on PATH"; exit 1; }

# vendor/Wire patches that exact core's Wire and uses its private SERCOM API:
# refuse to build against any other core (as BuildAndUpload.cmd does).
CORE_FOUND=$(arduino-cli core list | awk '$1 == "arduino:samd" { print $2 }')
if [ "$CORE_FOUND" != "$CORE_REQUIRED" ]; then
    echo "ERROR: arduino:samd $CORE_REQUIRED is required, found \"$CORE_FOUND\"."
    echo "       vendor/Wire is a patched copy of that core's Wire library — see"
    echo "       vendor/Wire/README.md.  On the Jetson (aarch64) the core is installed"
    echo "       by docker_dev/install_samd_aarch64.py (arduino-cli alone cannot)."
    exit 1
fi

PORT=""
EXTRA=()
for arg in "$@"; do
    case "$arg" in
        amg) EXTRA=(--build-property "compiler.cpp.extra_flags=-DDASHCAM_IMU_MODE_AMG") ;;
        *)   PORT="$arg" ;;
    esac
done
[ ${#EXTRA[@]} -gt 0 ] && echo "Building with the IMU in AMG (raw) mode."

# "auto": the MKR Zero by USB identity — Arduino VID 2341, PID 804f (sketch
# running) or 004f (bootloader).  Never a bare ttyACM guess: the ESP32-C3
# bridge is a ttyACM too.
if [ "$PORT" = "auto" ]; then
    source "$SKETCH_DIR/find_port.sh"
    PORT=$(find_mkr_zero_port) || exit 1
    echo "upload port: $PORT"
fi

# arduino-cli writes the build into the sketch's cache; keep it out of the repo.
BUILD_PATH="${BUILD_PATH:-/tmp/mkr_zero_build}"

args=(compile --warnings all "${EXTRA[@]}" --fqbn "$FQBN" --build-path "$BUILD_PATH"
      --library "$WIRE_DIR" --library "$CAN_DIR" --library "$SDFAT_DIR" --library "$LIB_DIR")
[ -n "$PORT" ] && args+=(--upload --port "$PORT")

exec arduino-cli "${args[@]}" "$SKETCH_DIR"
