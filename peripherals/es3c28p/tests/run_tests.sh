#!/bin/bash
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$TEST_DIR/../../.." && pwd)"
# The suite runs as root in the dev container: leave no root-owned __pycache__.
export PYTHONDONTWRITEBYTECODE=1
TEST_BUILD="$(mktemp -d)"
trap 'rm -rf "$TEST_BUILD"' EXIT
c++ -std=c++11 -Wall -Wextra -Werror "$TEST_DIR/calibration_test.cpp" -o "$TEST_BUILD/calibration"
"$TEST_BUILD/calibration"
c++ -std=c++11 -Wall -Wextra -Werror "$TEST_DIR/usb_protocol_test.cpp" -o "$TEST_BUILD/protocol"
# Video wire contract: C++11 as on the firmware side, C++17 as in the Jetson Makefile.
for std in c++11 c++17; do
    c++ -std=$std -Wall -Wextra -Werror "$TEST_DIR/video_protocol_test.cpp" -o "$TEST_BUILD/video_protocol_$std"
    "$TEST_BUILD/video_protocol_$std" "$TEST_DIR/video_golden_vectors.txt"
done
# Map extension (Es3MapProtocol.h): the same two standards.
for std in c++11 c++17; do
    c++ -std=$std -Wall -Wextra -Werror "$TEST_DIR/map_protocol_test.cpp" -o "$TEST_BUILD/map_protocol_$std"
    "$TEST_BUILD/map_protocol_$std" "$TEST_DIR/map_golden_vectors.txt"
done
# Map extension v2 (board-side panning): C++ vectors only.
for std in c++11 c++17; do
    c++ -std=$std -Wall -Wextra -Werror "$TEST_DIR/pan_protocol_test.cpp" -o "$TEST_BUILD/pan_protocol_$std"
    "$TEST_BUILD/pan_protocol_$std" "$TEST_DIR/pan_golden_vectors.txt"
done
c++ -std=c++11 -Wall -Wextra -Werror -I"$TEST_DIR/../lib/es3Graphics" -I"$TEST_DIR/../lib/es3UsbLink" "$TEST_DIR/video_state_test.cpp" -o "$TEST_BUILD/video_state"
"$TEST_BUILD/video_state"
for std in c++11 c++17; do
    c++ -std=$std -Wall -Wextra -Werror -I"$TEST_DIR/../lib/es3Graphics" -I"$TEST_DIR/../lib/es3UsbLink" \
        -I"$TEST_DIR/../lib/es3Video" -I"$TEST_DIR/../lib/es3Map" \
        "$TEST_DIR/map_surface_test.cpp" -o "$TEST_BUILD/map_surface_$std"
    "$TEST_BUILD/map_surface_$std"
    c++ -std=$std -Wall -Wextra -Werror -I"$TEST_DIR/../lib/es3Graphics" -I"$TEST_DIR/../lib/es3UsbLink" \
        -I"$TEST_DIR/../lib/es3Video" -I"$TEST_DIR/../lib/es3Map" \
        "$TEST_DIR/pan_state_test.cpp" -o "$TEST_BUILD/pan_state_$std"
    "$TEST_BUILD/pan_state_$std"
done
# Resource failures and SD media parsers use the same portable production code.
for std in c++11 c++17; do
    for test in resource media_header; do
        c++ -std=$std -Wall -Wextra -Werror -I"$TEST_DIR/../lib/es3Graphics" -I"$TEST_DIR/../lib/es3UsbLink" \
            "$TEST_DIR/${test}_test.cpp" -o "$TEST_BUILD/${test}_$std"
        "$TEST_BUILD/${test}_$std"
    done
done
# The Jetson link library compiles its own copies; they must not drift.
for header in Es3Protocol.h Es3VideoProtocol.h Es3MapProtocol.h; do
    cmp "$TEST_DIR/../lib/es3UsbLink/$header" "$REPO_DIR/lib/libes3display/$header"
done
ES3_PROTOCOL_TEST="$TEST_BUILD/protocol" python3 "$TEST_DIR/test_usb_client.py"
