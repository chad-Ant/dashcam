#!/bin/bash
#
# build_and_upload.sh — build / flash the ESP_Sentinel bridge from Linux
# (the Jetson's l4t-ml-gpio container).  Linux twin of BuildAndUpload.cmd:
# same FQBN, same libraries, same warning level — keep the two in step.
#
#   ./build_and_upload.sh                   compile only
#   ./build_and_upload.sh /dev/ttyACM0      compile and upload
#   ./build_and_upload.sh auto              compile and upload to the C3 found
#                                           under /dev/serial/by-id
#
# From the host (the container has arduino-cli + the esp32 core baked in):
#   docker run --rm --privileged -v /dev:/dev -v ~/dashcam:/user/dashcam \
#     l4t-ml-gpio:latest /user/dashcam/peripherals/esp32-c3/build_and_upload.sh auto
#
# Stop anything holding the port first (commlink_test, a dashcam version that
# talks to the bridge, picocom): the upload needs the port exclusively.
set -euo pipefail

SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMMLINK_DIR="$SKETCH_DIR/lib/commLink"
HOSTLINK_DIR="$SKETCH_DIR/lib/hostLink"

# CDCOnBoot=cdc is REQUIRED: Serial must be the native USB port wired to the
# Jetson (hostLink.h #errors otherwise).  DebugLevel=none keeps the core's log
# macros off the binary USB channel.  See BuildAndUpload.cmd for the rest.
FQBN="esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app,CPUFreq=160,FlashMode=qio,FlashFreq=80,FlashSize=4M,DebugLevel=none,UploadSpeed=921600"

command -v arduino-cli >/dev/null || { echo "arduino-cli is not available on PATH"; exit 1; }

PORT="${1:-}"
if [ "$PORT" = "auto" ]; then
    PORT=$(ls /dev/serial/by-id/*Espressif*USB_JTAG* 2>/dev/null | head -1 || true)
    [ -n "$PORT" ] || { echo "no Espressif USB-JTAG device under /dev/serial/by-id"; exit 1; }
    PORT=$(readlink -f "$PORT")
    echo "upload port: $PORT"
fi

# arduino-cli writes the build into the sketch's cache; keep it out of the repo.
BUILD_PATH="${BUILD_PATH:-/tmp/esp32c3_bridge_build}"

args=(compile --warnings all --fqbn "$FQBN" --build-path "$BUILD_PATH"
      --library "$COMMLINK_DIR" --library "$HOSTLINK_DIR")
[ -n "$PORT" ] && args+=(--upload --port "$PORT")

exec arduino-cli "${args[@]}" "$SKETCH_DIR"
