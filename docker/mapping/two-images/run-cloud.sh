#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./docker/mapping/two-images/run-cloud.sh [docker|nerdctl]

Environment variables:
  IMAGE_NAME    Default: vehicle-cloud:latest
  SERVER_HOST   Default: 0.0.0.0
  SERVER_PORT   Default: 5000
  PCL_PUB_NAME  Default: cloud_sent
  FRAME_ID      Default: map
  LOG_DIR       Optional host directory to mount to /data/perception/logs
  MAPS_DIR      Optional host directory to mount to /data/perception/maps
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

choose_tool() {
  if [[ -n "${1:-}" ]]; then
    printf '%s\n' "$1"
    return 0
  fi

  if command -v docker >/dev/null 2>&1; then
    printf 'docker\n'
    return 0
  fi

  if command -v nerdctl >/dev/null 2>&1; then
    printf 'nerdctl\n'
    return 0
  fi

  echo "No supported container runtime found. Install docker or nerdctl." >&2
  exit 1
}

TOOL="$(choose_tool "${1:-}")"
if [[ "${TOOL}" != "docker" && "${TOOL}" != "nerdctl" ]]; then
  echo "Unsupported tool: ${TOOL}" >&2
  usage
  exit 1
fi

IMAGE_NAME="${IMAGE_NAME:-vehicle-cloud:latest}"
SERVER_HOST="${SERVER_HOST:-0.0.0.0}"
SERVER_PORT="${SERVER_PORT:-5000}"
PCL_PUB_NAME="${PCL_PUB_NAME:-cloud_sent}"
FRAME_ID="${FRAME_ID:-map}"
LOG_DIR="${LOG_DIR:-}"
MAPS_DIR="${MAPS_DIR:-}"

cmd=(run --rm -it)
if [[ "${TOOL}" == "docker" ]]; then
  cmd+=(--network host)
else
  cmd+=(--net host)
fi

cmd+=(
  -e "SERVER_HOST=${SERVER_HOST}"
  -e "SERVER_PORT=${SERVER_PORT}"
  -e "PCL_PUB_NAME=${PCL_PUB_NAME}"
  -e "FRAME_ID=${FRAME_ID}"
)

if [[ -n "${LOG_DIR}" ]]; then
  cmd+=(-v "${LOG_DIR}:/data/perception/logs")
fi

if [[ -n "${MAPS_DIR}" ]]; then
  cmd+=(-v "${MAPS_DIR}:/data/perception/maps")
fi

cmd+=("${IMAGE_NAME}")

echo "Using runtime: ${TOOL}"
echo "Image: ${IMAGE_NAME}"
set -x
"${TOOL}" "${cmd[@]}"
