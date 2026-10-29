#!/usr/bin/env bash
# Run in an isolated copy of the repository inside the build image.
set -euo pipefail
make -s -j2 dashcam_v0_4 monitoring_test config_test
build_dir=$(find bin -maxdepth 1 -type d -name 'build_*' | sort | tail -n 1)
"$build_dir/monitoring_test"
"$build_dir/config_test"
