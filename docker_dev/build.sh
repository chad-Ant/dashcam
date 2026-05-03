#!/usr/bin/env bash
# Build l4t-ml-gpio:latest, archiving any existing image first.
#
# Workflow:
#   1. If l4t-ml-gpio:latest already exists, tag it as l4t-ml-gpio:build_<timestamp>
#      and remove the :latest tag, then use the archived image as the build base.
#   2. If it does not exist, fall back to the Dockerfile ARG default
#      (dustynv/l4t-ml:r36.4.0) so the first build still works.
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
    BASE_IMAGE="${ARCHIVE}"
else
    echo "[build.sh] No existing ${LATEST} found; using upstream base from Dockerfile ARG default"
    BASE_IMAGE="dustynv/l4t-ml:r36.4.0"
fi

echo "[build.sh] Building ${LATEST} from ${BASE_IMAGE} ..."
docker build \
    --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
    --tag "${LATEST}" \
    --file "${SCRIPT_DIR}/Dockerfile" \
    "${SCRIPT_DIR}"

echo "[build.sh] Done: ${LATEST} (base: ${BASE_IMAGE})"
