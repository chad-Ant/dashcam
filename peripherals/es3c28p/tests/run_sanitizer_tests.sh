#!/bin/bash
# Portable firmware logic only; run_decoder_tests.sh covers pinned JPEGDEC.
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_BUILD="$(mktemp -d)"
trap 'rm -rf "$TEST_BUILD"' EXIT
for test in calibration video_state map_surface pan_state resource media_header; do
    c++ -std=c++17 -Wall -Wextra -Werror -g \
        -fsanitize=address,undefined -fno-sanitize-recover=all \
        -I"$TEST_DIR/../lib/es3Graphics" -I"$TEST_DIR/../lib/es3UsbLink" -I"$TEST_DIR/../lib/es3Video" \
        -I"$TEST_DIR/../lib/es3Map" \
        "$TEST_DIR/${test}_test.cpp" -o "$TEST_BUILD/$test"
    "$TEST_BUILD/$test"
done
