#!/bin/bash
set -euo pipefail
BOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-}" in
    usb_sink|jpeg_from_sd) sketch="$1";;
    *) echo "Usage: ES3_USB_MODE=hwcdc|default $0 usb_sink|jpeg_from_sd [port]"; exit 1;;
esac
arduino-cli lib install --no-deps "JPEGDEC@1.8.4"
args=(compile --warnings all --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,USBMode=${ES3_USB_MODE:-hwcdc},CDCOnBoot=cdc"
      --library "$BOARD_DIR/lib/es3Board" --library "$BOARD_DIR/lib/es3Video" --library "$BOARD_DIR/lib/es3Map" --library "$BOARD_DIR/lib/es3UsbLink" --library "$BOARD_DIR/lib/es3Graphics"
      --library "$BOARD_DIR/lib/es3Storage" --library "$BOARD_DIR/lib/es3Audio")
if [ -n "${2:-}" ]; then args+=(--upload --port "$2"); fi
exec arduino-cli "${args[@]}" "$BOARD_DIR/bench/$sketch"
