#!/bin/bash
# Shared by the production uploader and SD-map installer. Sourcing does no I/O.
# Optional sysfs root is for isolated tests; production callers use the default.
find_mkr_zero_port() {
    local tty dev vid pid found="" count=0
    local tty_root="${1:-/sys/class/tty}"
    for tty in "$tty_root"/ttyACM*; do
        [ -e "$tty" ] || continue
        dev=$(readlink -f "$tty/device/..") || continue
        vid=$(cat "$dev/idVendor" 2>/dev/null || true)
        pid=$(cat "$dev/idProduct" 2>/dev/null || true)
        if [ "$vid" = 2341 ] && { [ "$pid" = 804f ] || [ "$pid" = 004f ]; }; then
            found="/dev/$(basename "$tty")"
            count=$((count + 1))
        fi
    done
    if [ "$count" -ne 1 ]; then
        echo "expected one MKR Zero (2341:804f/004f), found $count; specify its port explicitly" >&2
        return 1
    fi
    printf '%s\n' "$found"
}
