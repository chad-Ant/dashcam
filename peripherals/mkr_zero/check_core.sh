#!/bin/bash
# Shared by every Linux script that builds against vendor/Wire. Sourcing does no I/O.
# vendor/Wire is a patched copy of arduino:samd 1.8.14's Wire and calls that core's
# private SERCOM API (vendor/Wire/README.md), so a build against any other core is
# refused - as every BuildAndUpload.cmd does - rather than producing a binary
# against an API the copy was never tested with.
SAMD_CORE_REQUIRED="1.8.14"
require_samd_core() {
    command -v arduino-cli >/dev/null || { echo "arduino-cli is not available on PATH"; return 1; }
    local found
    found=$(arduino-cli core list | awk '$1 == "arduino:samd" { print $2 }')
    if [ "$found" != "$SAMD_CORE_REQUIRED" ]; then
        echo "ERROR: arduino:samd $SAMD_CORE_REQUIRED is required, found \"$found\"."
        echo "       vendor/Wire is a patched copy of that core's Wire library - see"
        echo "       vendor/Wire/README.md.  On the Jetson (aarch64) the core is installed"
        echo "       by docker_dev/install_samd_aarch64.py (arduino-cli alone cannot)."
        return 1
    fi
}
