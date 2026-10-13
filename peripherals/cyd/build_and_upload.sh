#!/bin/bash
# CYD dashboard - Linux compile / upload counterpart to BuildAndUpload.cmd.
#   ./build_and_upload.sh              compile only
#   ./build_and_upload.sh /dev/ttyUSB0 compile and upload
#   ./build_and_upload.sh auto         compile and upload to the one CH340 under /dev/serial/by-id
set -euo pipefail

SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GRAPHICS_DIR="$SKETCH_DIR/lib/cydGraphics"
STORAGE_DIR="$SKETCH_DIR/lib/cydStorage"
AUDIO_DIR="$SKETCH_DIR/lib/cydAudio"
FQBN="esp32:esp32:esp32"

command -v arduino-cli >/dev/null || { echo "arduino-cli is not available on PATH"; exit 1; }

port="${1:-}"
if [ "$port" = "auto" ]; then
    shopt -s nullglob
    ports=(/dev/serial/by-id/usb-1a86_*)
    if [ "${#ports[@]}" -ne 1 ]; then
        echo "auto: expected one CH340 under /dev/serial/by-id, found ${#ports[@]} (host driver: host/ch341_module.sh check)"
        exit 1
    fi
    port="${ports[0]}"
fi

args=(compile --warnings all --fqbn "$FQBN" --library "$STORAGE_DIR" --library "$GRAPHICS_DIR" --library "$AUDIO_DIR")
if [ -n "$port" ]; then
    args+=(--upload --port "$port")
fi
exec arduino-cli "${args[@]}" "$SKETCH_DIR"
