#!/usr/bin/env bash
# Build l4t-ml-gpio:latest, archiving any existing image first.
#
# Workflow:
#   1. If l4t-ml-gpio:latest already exists, tag it as l4t-ml-gpio:build_<timestamp>
#      and remove the :latest tag.
#   2. Always rebuild from the clean upstream Jetson base.  Building on the
#      previous application image permanently preserves removed packages and
#      was the cause of duplicate GStreamer plugin registrations.
#   3. Build and tag the result as l4t-ml-gpio:latest.
set -euo pipefail

IMAGE_NAME="l4t-ml-gpio"
LATEST="${IMAGE_NAME}:latest"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
ARCHIVE="${IMAGE_NAME}:build_${TIMESTAMP}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if docker image inspect "${LATEST}" > /dev/null 2>&1; then
    echo "[build.sh] Archiving ${LATEST} → ${ARCHIVE}"
    docker tag "${LATEST}" "${ARCHIVE}"
    docker rmi "${LATEST}"
else
    echo "[build.sh] No existing ${LATEST} found"
fi

BASE_IMAGE="dustynv/l4t-ml:r36.4.0"
echo "[build.sh] Building ${LATEST} from ${BASE_IMAGE} ..."
docker build \
    --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
    --tag "${LATEST}" \
    --file "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo "[build.sh] Done: ${LATEST} (base: ${BASE_IMAGE})"
