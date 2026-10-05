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
#   ./build_and_upload.sh [PORT|auto] selftest[=N]
#                                            BENCH build: the raw CAN stream's
#                                            throughput self-test (below)
#
# "amg" and "selftest" combine, in any order and with or without a port. Without
# either, the build is bit-for-bit the production one.
#
# selftest / selftest=N defines DASHCAM_CAN_STREAM_SELFTEST=N (default 2400,
# about twice the measured bus; 1-10000). While the CAN drain is armed (the boot
# mode, DISCOVER) its timer interrupt also synthesizes N frames a second into
# the ring, with no SPI and no bus: id 0x7F0, DLC 8, data = a 32-bit sequence
# number, the ring fill and a check word (lib/CANSniffFunctions.h). So the USB
# stream's capacity can be measured on the bench, without a car: the Orin counts
# sequence gaps exactly, and the FS lines say where any loss happened. The board
# announces itself at boot ("CAN: SELFTEST BUILD") and adds st= and loop=N/Mus
# (passes since the last line / longest pass) to its status line. Synthetic
# frames count as drained, and therefore as a live vehicle. NEVER flash a
# selftest build to the car: its stream carries frames the vehicle never sent.
#
# From the host (the image carries arduino-cli, arduino:samd 1.8.14 and the
# pinned libraries — see docker_dev/Dockerfile):
#   docker run --rm --privileged -v /dev:/dev -v ~/dashcam:/user/dashcam \
#     l4t-ml-gpio:latest /user/dashcam/peripherals/mkr_zero/build_and_upload.sh auto
#
# The MKR's USB port is a PRODUCTION data path, no longer a development link:
# besides flashing and the human console, it carries the raw CAN stream — every
# frame, as "F ..." lines, plus an "FS ..." stats line each second (see
# lib/CANRawStream.h) — which the Orin records and decodes offline. The
# telemetry still goes through the ESP32-C3 bridge (Serial1), now without any
# CAN-derived values. So whatever holds the MKR's port in the car is a data
# consumer, not a debugging aid: stop it for the length of an upload, and
# expect the raw stream to be missing for that time.
#
# Upload resets the board into its SAM-BA bootloader (1200-baud touch) and it
# re-enumerates, possibly as a different ttyACM; if it never shows up,
# double-tap RESET and run "auto" again.  Close anything holding the MKR's port
# first (the Orin's raw-CAN recorder, picocom, a serial monitor).
set -euo pipefail

SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIB_DIR="$SKETCH_DIR/lib"
WIRE_DIR="$SKETCH_DIR/vendor/Wire"
CAN_DIR="$SKETCH_DIR/vendor/CANBus"
SDFAT_DIR="$SKETCH_DIR/vendor/SdFat"

FQBN="arduino:samd:mkrzero"

# vendor/Wire patches arduino:samd 1.8.14's Wire and uses its private SERCOM
# API: refuse to build against any other core (as BuildAndUpload.cmd does).
source "$SKETCH_DIR/check_core.sh"
require_samd_core || exit 1

PORT=""
DEFINES=()
for arg in "$@"; do
    case "$arg" in
        amg)
            DEFINES+=(-DDASHCAM_IMU_MODE_AMG)
            echo "Building with the IMU in AMG (raw) mode." ;;
        selftest|selftest=*)
            RATE="${arg#selftest}"; RATE="${RATE#=}"; RATE="${RATE:-2400}"
            # Digits only, and normalised to base 10: "selftest=02400" handed to
            # the compiler as 02400 would be an OCTAL literal, 1280 frames/s.
            if ! [[ "$RATE" =~ ^[0-9]{1,5}$ ]] || [ $((10#$RATE)) -lt 1 ] || [ $((10#$RATE)) -gt 10000 ]; then
                echo "selftest=N: N is frames per second, 1-10000 (got '$RATE')" >&2
                exit 2
            fi
            RATE=$((10#$RATE))
            DEFINES+=(-DDASHCAM_CAN_STREAM_SELFTEST=$RATE)
            echo "Building the BENCH self-test: $RATE synthetic CAN frames/s. Never flash this to the car." ;;
        *)  PORT="$arg" ;;
    esac
done
EXTRA=()
[ ${#DEFINES[@]} -gt 0 ] && EXTRA=(--build-property "compiler.cpp.extra_flags=${DEFINES[*]}")

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
