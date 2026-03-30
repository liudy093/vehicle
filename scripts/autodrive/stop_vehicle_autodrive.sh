#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../shared/lib/common.sh"
source "${SCRIPT_DIR}/../shared/lib/vehicle_autodrive_common.sh"

DISPLAY_SCRIPT_PATH="${0}"

usage() {
  local vehicle_id="${1:-<vehicle-id>}"
  local host_ref="config/shared/vehicle_hosts.env -> VEHICLE1_HOST|VEHICLE2_HOST"

  if [[ "${vehicle_id}" == "vehicle1" || "${vehicle_id}" == "vehicle2" ]]; then
    host_ref="$(vehicle_autodrive_host_usage_ref "${vehicle_id}")"
  fi

  cat <<EOF
Usage:
  ${DISPLAY_SCRIPT_PATH} ${vehicle_id}

Description:
  Stop ${vehicle_id} autodrive processes using pod-native execution by default.
  SSH mode remains available as a fallback transport.
  This only stops the ROS/autodrive chain and does not send a dedicated P/N gear command.

Environment variables:
  VEHICLE_HOST         Default: ${host_ref}
  VEHICLE_PASSWORD     SSH password. If unset, the script prompts for it.
  SSH_IDENTITY_FILE    Optional SSH identity file passed to ssh -i
  REMOTE_AUTODRIVE_DIR Default: /home/nvidia/AutoDrive
  VEHICLE_EXECUTION_MODE Default: pod (pod or ssh)
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

VEHICLE_ID="${1:-}"
if [[ -z "${VEHICLE_ID}" ]]; then
  usage >&2
  exit 1
fi

vehicle_autodrive_validate_vehicle_id "${VEHICLE_ID}"
shift

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage "${VEHICLE_ID}"
  exit 0
fi

VEHICLE_EXECUTION_MODE="${VEHICLE_EXECUTION_MODE:-pod}"
EXECUTION_MODE="$(vehicle_autodrive_execution_mode)"

vehicle_autodrive_require_transport_tools "${EXECUTION_MODE}"

VEHICLE_HOST="${VEHICLE_HOST:-}"
REMOTE_AUTODRIVE_DIR="${REMOTE_AUTODRIVE_DIR:-/home/nvidia/AutoDrive}"
REMOTE_LOG_DIR="${REMOTE_AUTODRIVE_DIR}/datas/logs/remote_start"

STOP_SCRIPT="$(cat <<EOF
set -euo pipefail

REMOTE_AUTODRIVE_DIR='${REMOTE_AUTODRIVE_DIR}'
REMOTE_LOG_DIR='${REMOTE_LOG_DIR}'
REMOTE_MONITOR_SCRIPT_PATH='${REMOTE_LOG_DIR}/arrival_monitor.sh'
REMOTE_MONITOR_PID_FILE='${REMOTE_LOG_DIR}/arrival_monitor.pid'

$(vehicle_autodrive_remote_lib)

cd "\${REMOTE_AUTODRIVE_DIR}"
source_safe /opt/ros/foxy/setup.bash
source_safe "\${REMOTE_AUTODRIVE_DIR}/install/setup.bash"

cleanup_autodrive

sleep 2

reset_ros_daemon

echo "REMAINING_PROCESSES_BEGIN"
pgrep -af 'arrival_monitor.sh|launch_tracker.py|launch_global_planner.py|launch_lidar_object.py|launch_lidar.py|launch_defaults.py|/install/car_decision/lib/car_decision/car_decision|/install/local_path_planning/lib/local_path_planning/local_path_planning|/install/pid/lib/pid/pid|/install/car_control/lib/car_control/car_control|/install/global_path_planning/lib/global_path_planning/global_path_planning|/install/fusion/lib/fusion/fusion|/install/gps/lib/gps/gps|/install/imu/lib/imu/imu|/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node|/install/lidar_obstacle/lib/lidar_obstacle/lidar_obstacle|/install/left_lidar_obstacle/lib/left_lidar_obstacle/left_lidar_obstacle|/install/right_lidar_obstacle/lib/right_lidar_obstacle/right_lidar_obstacle|/install/regulator/lib/regulator/regulator|/install/sonic_obstacle/lib/sonic_obstacle/sonic_obstacle|/install/net_work/lib/net_work/net_work|/install/car_ori/lib/car_ori/car_ori' || true
echo "REMAINING_PROCESSES_END"
echo "REMAINING_NODES_BEGIN"
ros2 node list 2>/dev/null | egrep '^/car_decision$|^/local_path_planning$|^/pid$|^/car_control$|^/global_path_planning$|^/fusion$|^/gps$|^/imu$' || true
echo "REMAINING_NODES_END"
echo "AUTODRIVE_STOP_REQUESTED"
EOF
)"

case "${EXECUTION_MODE}" in
  ssh)
    VEHICLE_HOST="${VEHICLE_HOST:-$(vehicle_autodrive_default_host "${VEHICLE_ID}")}"
    vehicle_autodrive_prompt_password

    echo "Stopping ${VEHICLE_ID} autodrive on ${VEHICLE_HOST}..."
    vehicle_autodrive_remote_script "${VEHICLE_ID}" "${VEHICLE_HOST}" "${STOP_SCRIPT}" 180
    ;;
  *)
    echo "Stopping ${VEHICLE_ID} autodrive inside the vehicle pod..."
    vehicle_autodrive_run_script "${VEHICLE_ID}" "${VEHICLE_HOST}" "${STOP_SCRIPT}" 180
    ;;
esac

case "${EXECUTION_MODE}" in
  ssh)
    cat <<EOF
${VEHICLE_ID} autodrive stop requested successfully.
Remote host: ${VEHICLE_HOST}
EOF
    ;;
  *)
    cat <<EOF
${VEHICLE_ID} autodrive stop requested successfully.
Execution:   pod
EOF
    ;;
esac
