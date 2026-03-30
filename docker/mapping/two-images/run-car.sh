#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./docker/mapping/two-images/run-car.sh [docker|nerdctl]

Environment variables:
  IMAGE_NAME         Default: vehicle-car:latest
  VEHICLE_ID         Default: vehicle1
  CLOUD_SERVER_URL   Default: http://127.0.0.1:5000
  GPS_SERIAL_PORT    Default: /dev/ttyUSB1
  GPS_BAUD_RATE      Default: 115200
  GPS_LAT0           Default: 39.95754418
  GPS_LON0           Default: 116.30460525
  LID_TOPIC          Default: /robot1/livox/lidar
  IMU_TOPIC          Default: /robot1/imu
  USE_LOOP_CLOSURE   Default: true
  USE_RVIZ           Default: false
  SENDER_NAME        Default: lidar_data_sender
  LIVOX_CONFIG_DIR   Optional host directory mounted to /config/livox
  LOG_DIR            Optional host directory mounted to /data/perception/logs
  PCD_DIR            Optional host directory mounted to /data/perception/pcd
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

IMAGE_NAME="${IMAGE_NAME:-vehicle-car:latest}"
VEHICLE_ID="${VEHICLE_ID:-vehicle1}"
CLOUD_SERVER_URL="${CLOUD_SERVER_URL:-http://127.0.0.1:5000}"
GPS_SERIAL_PORT="${GPS_SERIAL_PORT:-/dev/ttyUSB1}"
GPS_BAUD_RATE="${GPS_BAUD_RATE:-115200}"
GPS_LAT0="${GPS_LAT0:-39.95754418}"
GPS_LON0="${GPS_LON0:-116.30460525}"
LID_TOPIC="${LID_TOPIC:-/robot1/livox/lidar}"
IMU_TOPIC="${IMU_TOPIC:-/robot1/imu}"
USE_LOOP_CLOSURE="${USE_LOOP_CLOSURE:-true}"
USE_RVIZ="${USE_RVIZ:-false}"
SENDER_NAME="${SENDER_NAME:-lidar_data_sender}"
LIVOX_CONFIG_DIR="${LIVOX_CONFIG_DIR:-}"
LOG_DIR="${LOG_DIR:-}"
PCD_DIR="${PCD_DIR:-}"

cmd=(run --rm -it --privileged)
if [[ "${TOOL}" == "docker" ]]; then
  cmd+=(--network host)
else
  cmd+=(--net host)
fi

cmd+=(
  -e "VEHICLE_ID=${VEHICLE_ID}"
  -e "CLOUD_SERVER_URL=${CLOUD_SERVER_URL}"
  -e "GPS_SERIAL_PORT=${GPS_SERIAL_PORT}"
  -e "GPS_BAUD_RATE=${GPS_BAUD_RATE}"
  -e "GPS_LAT0=${GPS_LAT0}"
  -e "GPS_LON0=${GPS_LON0}"
  -e "LID_TOPIC=${LID_TOPIC}"
  -e "IMU_TOPIC=${IMU_TOPIC}"
  -e "USE_LOOP_CLOSURE=${USE_LOOP_CLOSURE}"
  -e "USE_RVIZ=${USE_RVIZ}"
  -e "SENDER_NAME=${SENDER_NAME}"
  -v /dev:/dev
)

if [[ -n "${LIVOX_CONFIG_DIR}" ]]; then
  cmd+=(-e "LIVOX_CONFIG_DIR=/config/livox" -v "${LIVOX_CONFIG_DIR}:/config/livox")
fi

if [[ -n "${LOG_DIR}" ]]; then
  cmd+=(-v "${LOG_DIR}:/data/perception/logs")
fi

if [[ -n "${PCD_DIR}" ]]; then
  cmd+=(-v "${PCD_DIR}:/data/perception/pcd")
fi

cmd+=("${IMAGE_NAME}")

echo "Using runtime: ${TOOL}"
echo "Image: ${IMAGE_NAME}"
echo "Vehicle: ${VEHICLE_ID}"
set -x
"${TOOL}" "${cmd[@]}"
