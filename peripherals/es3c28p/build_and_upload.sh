#!/bin/bash
# ES3C28P dashboard - Linux compile / upload counterpart to BuildAndUpload.cmd.
#   ./build_and_upload.sh              compile only
#   ./build_and_upload.sh /dev/ttyACM0 compile and upload
set -euo pipefail

SKETCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GRAPHICS_DIR="$SKETCH_DIR/lib/es3Graphics"
STORAGE_DIR="$SKETCH_DIR/lib/es3Storage"
AUDIO_DIR="$SKETCH_DIR/lib/es3Audio"
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,USBMode=${ES3_USB_MODE:-hwcdc},CDCOnBoot=cdc"

command -v arduino-cli >/dev/null || { echo "arduino-cli is not available on PATH"; exit 1; }

arduino-cli lib install --no-deps "JPEGDEC@1.8.4"

port="${1:-}"

args=(compile --warnings all --fqbn "$FQBN" --library "$STORAGE_DIR" --library "$GRAPHICS_DIR" --library "$AUDIO_DIR" --library "$SKETCH_DIR/lib/es3Board" --library "$SKETCH_DIR/lib/es3UsbLink" --library "$SKETCH_DIR/lib/es3Video" --library "$SKETCH_DIR/lib/es3Map")
if [ -n "$port" ]; then
    args+=(--upload --port "$port")
fi
exec arduino-cli "${args[@]}" "$SKETCH_DIR"
