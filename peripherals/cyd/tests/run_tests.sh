#!/bin/bash
set -euo pipefail
TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEST_BUILD="$(mktemp -d)"
trap 'rm -rf "$TEST_BUILD"' EXIT
for standard in c++11 c++17; do
    c++ -std="$standard" -Wall -Wextra -Werror "$TEST_DIR/map_receiver_test.cpp" -o "$TEST_BUILD/receiver"
    "$TEST_BUILD/receiver"
done
c++ -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined -fno-sanitize-recover=all -g \
    "$TEST_DIR/map_receiver_test.cpp" -o "$TEST_BUILD/receiver-san"
"$TEST_BUILD/receiver-san"
