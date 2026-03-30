#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../shared/lib/common.sh"
source "${SCRIPT_DIR}/../shared/vehicle_inventory.sh"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/mapping/verify_vehicle2_mapping.sh

Environment variables:
  VEHICLE_HOST    Default: config/shared/vehicle_hosts.env -> VEHICLE2_HOST
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

DEFAULT_VEHICLE_HOST="$(vehicle_inventory_host "vehicle2" "nvidia")"
VEHICLE_HOST="${VEHICLE_HOST:-${DEFAULT_VEHICLE_HOST}}"
VEHICLE_EXECUTION_MODE="${VEHICLE_EXECUTION_MODE:-pod}"
SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null)

verify_script_body() {
  cat <<'EOF'
set -eo pipefail

set +u
source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true
source /home/nvidia/AutoDrive/install/setup.bash >/dev/null 2>&1 || true
source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOSTNAME=127.0.0.1
imu_info="$(timeout 8 rostopic info /imu)"
printf "%s\n" "${imu_info}"
echo ---
points_info="$(timeout 8 rostopic info /forward/rslidar_points)"
printf "%s\n" "${points_info}"
if printf "%s\n%s\n" "${imu_info}" "${points_info}" | grep -F "Publishers: None" >/dev/null; then
  echo "Missing ROS1 publishers for vehicle2 mapping inputs" >&2
  exit 1
fi
echo ---
timeout 8 rostopic echo /Odometry -n 1 | egrep "frame_id|child_frame_id" || true
echo ---
timeout 8 rostopic echo /cloud_registered -n 1 | egrep "frame_id|width|height" || true
EOF
}

execute_verify_script() {
  local verify_command=""
  verify_command="$(verify_script_body)"

  if [[ "${VEHICLE_EXECUTION_MODE}" == "ssh" ]]; then
    require_cmd ssh
    ssh "${SSH_OPTS[@]}" "${VEHICLE_HOST}" "bash -s" <<<"${verify_command}"
    return 0
  fi

  bash -s <<<"${verify_command}"
}

execute_verify_script
