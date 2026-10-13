#!/usr/bin/env bash
# Build l4t-ml-gpio:latest, archiving any existing image first.
#
# Workflow:
#   1. If l4t-ml-gpio:latest already exists, also tag it l4t-ml-gpio:build_<timestamp>.
#      :latest itself stays on the old image for the whole build: the launch scripts
#      and dashcam-v04.service run :latest, and the build takes 15+ minutes.
#   2. Always rebuild from the clean upstream Jetson base.  Building on the
#      previous application image permanently preserves removed packages and
#      was the cause of duplicate GStreamer plugin registrations.
#   3. docker build --tag moves :latest to the new image only when the build
#      succeeds.  A failed or interrupted build (error, Ctrl-C, hangup, kill)
#      leaves :latest where it was.  (Until 2026-10-12 this script untagged
#      :latest first, and a dropped download left no :latest at all.)
set -euo pipefail

IMAGE_NAME="l4t-ml-gpio"
LATEST="${IMAGE_NAME}:latest"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
ARCHIVE="${IMAGE_NAME}:build_${TIMESTAMP}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if docker image inspect "${LATEST}" > /dev/null 2>&1; then
    echo "[build.sh] Archiving ${LATEST} → ${ARCHIVE} (${LATEST} stays until the build succeeds)"
    docker tag "${LATEST}" "${ARCHIVE}"
else
    echo "[build.sh] No existing ${LATEST} found"
fi

BASE_IMAGE="dustynv/l4t-ml:r36.4.0"
echo "[build.sh] Building ${LATEST} from ${BASE_IMAGE} ..."
if ! docker build \
    --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
    --tag "${LATEST}" \
    --file "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"; then
    echo "[build.sh] Build FAILED: ${LATEST} is unchanged" >&2
    exit 1
fi

echo "[build.sh] Done: ${LATEST} (base: ${BASE_IMAGE})"
