#!/usr/bin/env bash
set -euo pipefail
test_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
build_dir=$(mktemp -d /tmp/encoderpan-tests.XXXXXX)
flags=(-std=c++11 -Wall -Wextra -Wconversion -Wshadow -Werror -pedantic -g)
sources=("$test_dir/../src/EncoderPan.cpp" "$test_dir/test_encoderpan.cpp")
g++ "${flags[@]}" -fanalyzer -I"$test_dir/../src" "${sources[@]}" -o "$build_dir/analyzed"
"$build_dir/analyzed"
g++ "${flags[@]}" -fsanitize=address,undefined -fno-omit-frame-pointer -I"$test_dir/../src" "${sources[@]}" -o "$build_dir/sanitized"
"$build_dir/sanitized"
echo "Test binaries: $build_dir"
