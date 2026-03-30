#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../shared/lib/common.sh"
source "${SCRIPT_DIR}/../shared/lib/vehicle_layout.sh"
source "${SCRIPT_DIR}/../shared/lib/vehicle_remote_runtime.sh"
source "${SCRIPT_DIR}/../shared/vehicle_inventory.sh"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/mapping/start_vehicle2_mapping.sh

Environment variables:
  VEHICLE_HOST       Default: config/shared/vehicle_hosts.env -> VEHICLE2_HOST
  CLOUD_SERVER_URL   Default: http://172.16.2.80:30404
  SEND_INTERVAL_SEC  Default: 0.5
  IMU_FRAME_ID       Default: imu_link
  ANGULAR_VELOCITY_AXES   Default: x,y,z
  ANGULAR_VELOCITY_SIGNS  Default: 1,1,1
  LINEAR_ACCELERATION_AXES  Default: x,y,z
  LINEAR_ACCELERATION_SIGNS Default: 1,1,1
  LINEAR_ACCELERATION_SCALE Default: 9.8
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

DEFAULT_VEHICLE_HOST="$(vehicle_inventory_host "vehicle2" "nvidia")"
VEHICLE_HOST="${VEHICLE_HOST:-${DEFAULT_VEHICLE_HOST}}"
VEHICLE_EXECUTION_MODE="${VEHICLE_EXECUTION_MODE:-pod}"
SSH_IDENTITY_FILE="${SSH_IDENTITY_FILE:-}"
SSH_OPTS=()
while IFS= read -r opt; do
  SSH_OPTS+=("${opt}")
done < <(build_ssh_opts "${SSH_IDENTITY_FILE}")
CLOUD_SERVER_URL="${CLOUD_SERVER_URL:-http://172.16.2.80:30404}"
SEND_INTERVAL_SEC="${SEND_INTERVAL_SEC:-0.5}"
IMU_FRAME_ID="${IMU_FRAME_ID:-imu_link}"
ANGULAR_VELOCITY_AXES="${ANGULAR_VELOCITY_AXES:-x,y,z}"
ANGULAR_VELOCITY_SIGNS="${ANGULAR_VELOCITY_SIGNS:-1,1,1}"
LINEAR_ACCELERATION_AXES="${LINEAR_ACCELERATION_AXES:-x,y,z}"
LINEAR_ACCELERATION_SIGNS="${LINEAR_ACCELERATION_SIGNS:-1,1,1}"
LINEAR_ACCELERATION_SCALE="${LINEAR_ACCELERATION_SCALE:-9.8}"
REMOTE_MAPPING_DIR="${REMOTE_MAPPING_DIR:-$(vehicle_layout_remote_mapping_dir)}"
REMOTE_MAPPING_CATKIN_DIR="${REMOTE_MAPPING_CATKIN_DIR:-$(vehicle_layout_remote_mapping_catkin_dir)}"
REMOTE_MAPPING_ROS2_DIR="${REMOTE_MAPPING_ROS2_DIR:-$(vehicle_layout_remote_mapping_ros2_dir)}"
REMOTE_AUTODRIVE_DIR="${REMOTE_AUTODRIVE_DIR:-$(vehicle_layout_remote_autodrive_dir)}"
REMOTE_RUNTIME_FUNCTIONS="$(vehicle_remote_runtime_functions)"
quoted_axes_list() {
  printf '"%s"' "${1//,/\",\"}"
}

execute_vehicle_script() {
  local script_content=""
  script_content="$(cat)"

  if [[ "${VEHICLE_EXECUTION_MODE}" == "ssh" ]]; then
    ssh "${SSH_OPTS[@]}" "${VEHICLE_HOST}" \
      "REMOTE_RUNTIME_FUNCTIONS=$(printf '%q' "${REMOTE_RUNTIME_FUNCTIONS}") VEHICLE_EXECUTION_MODE='${VEHICLE_EXECUTION_MODE}' CLOUD_SERVER_URL='${CLOUD_SERVER_URL}' SEND_INTERVAL_SEC='${SEND_INTERVAL_SEC}' IMU_FRAME_ID='${IMU_FRAME_ID}' ANGULAR_VELOCITY_AXES='[$(quoted_axes_list "${ANGULAR_VELOCITY_AXES}")]' ANGULAR_VELOCITY_SIGNS='[${ANGULAR_VELOCITY_SIGNS}]' LINEAR_ACCELERATION_AXES='[$(quoted_axes_list "${LINEAR_ACCELERATION_AXES}")]' LINEAR_ACCELERATION_SIGNS='[${LINEAR_ACCELERATION_SIGNS}]' LINEAR_ACCELERATION_SCALE='${LINEAR_ACCELERATION_SCALE}' REMOTE_MAPPING_DIR='${REMOTE_MAPPING_DIR}' REMOTE_MAPPING_CATKIN_DIR='${REMOTE_MAPPING_CATKIN_DIR}' REMOTE_MAPPING_ROS2_DIR='${REMOTE_MAPPING_ROS2_DIR}' REMOTE_AUTODRIVE_DIR='${REMOTE_AUTODRIVE_DIR}' bash -s" <<<"${script_content}"
    return 0
  fi

  REMOTE_RUNTIME_FUNCTIONS="${REMOTE_RUNTIME_FUNCTIONS}" \
  VEHICLE_EXECUTION_MODE="${VEHICLE_EXECUTION_MODE}" \
  CLOUD_SERVER_URL="${CLOUD_SERVER_URL}" \
  SEND_INTERVAL_SEC="${SEND_INTERVAL_SEC}" \
  IMU_FRAME_ID="${IMU_FRAME_ID}" \
  ANGULAR_VELOCITY_AXES="[$(quoted_axes_list "${ANGULAR_VELOCITY_AXES}")]" \
  ANGULAR_VELOCITY_SIGNS="[${ANGULAR_VELOCITY_SIGNS}]" \
  LINEAR_ACCELERATION_AXES="[$(quoted_axes_list "${LINEAR_ACCELERATION_AXES}")]" \
  LINEAR_ACCELERATION_SIGNS="[${LINEAR_ACCELERATION_SIGNS}]" \
  LINEAR_ACCELERATION_SCALE="${LINEAR_ACCELERATION_SCALE}" \
  REMOTE_MAPPING_DIR="${REMOTE_MAPPING_DIR}" \
  REMOTE_MAPPING_CATKIN_DIR="${REMOTE_MAPPING_CATKIN_DIR}" \
  REMOTE_MAPPING_ROS2_DIR="${REMOTE_MAPPING_ROS2_DIR}" \
  REMOTE_AUTODRIVE_DIR="${REMOTE_AUTODRIVE_DIR}" \
  bash -s <<<"${script_content}"
}

cleanup_vehicle_state() {
  if [[ "${VEHICLE_EXECUTION_MODE}" == "ssh" ]]; then
    return 0
  fi

eval "${REMOTE_RUNTIME_FUNCTIONS}"
vehicle_remote_stop_systemd_unit perception-vehicle2-rslidar
vehicle_remote_stop_systemd_unit perception-vehicle2-gps-node
vehicle_remote_stop_systemd_unit perception-vehicle2-gps-pose-bridge
vehicle_remote_stop_systemd_unit perception-vehicle2-gps-imu-bridge
vehicle_remote_stop_systemd_unit perception-vehicle2-sender
vehicle_remote_stop_systemd_unit perception-vehicle2-ros1-bridge
vehicle_remote_stop_systemd_unit perception-vehicle2-fastlio
vehicle_remote_stop_systemd_unit perception-vehicle2-fastlio-http-sender
vehicle_remote_stop_mapping_processes
rm -f \
  /tmp/rslidar_sdk_vehicle2.log \
  /tmp/gps_node.log \
  /tmp/gps_pose_bridge_vehicle2.log \
  /tmp/gps_imu_bridge_vehicle2.log \
  /tmp/sender_node_vehicle2.log \
  /tmp/ros1_bridge_vehicle2.log \
  /tmp/fastlio_mapping_vehicle2.log \
  /tmp/fastlio_http_sender_vehicle2.log
}

if [[ "${VEHICLE_EXECUTION_MODE}" == "ssh" ]]; then
  require_cmd ssh
fi

cleanup_vehicle_state

execute_vehicle_script <<'EOF'
set -euo pipefail

eval "${REMOTE_RUNTIME_FUNCTIONS}"

source_safe() {
  set +u
  # ROS setup scripts frequently reference unset variables internally.
  source "$1" >/dev/null 2>&1
  set -u
}

cd "${REMOTE_AUTODRIVE_DIR}"
source_safe /opt/ros/foxy/setup.bash
source_safe "${REMOTE_AUTODRIVE_DIR}/install/setup.bash"

resolve_bridge_bin() {
  for candidate in \
    /opt/ros/foxy/lib/ros1_bridge/dynamic_bridge \
    ${REMOTE_MAPPING_ROS2_DIR}/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge \
    /home/nvidia/ros2_ws/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge
  do
    if [[ -x "${candidate}" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

vehicle_remote_stop_mapping_processes

vehicle_remote_launch_detached perception-vehicle2-rslidar \
  "source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_AUTODRIVE_DIR}/install/setup.bash >/dev/null 2>&1 || true; cd ${REMOTE_AUTODRIVE_DIR}; exec ros2 launch rslidar_sdk start.py >/tmp/rslidar_sdk_vehicle2.log 2>&1"
vehicle_remote_launch_detached perception-vehicle2-gps-node \
  "source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_AUTODRIVE_DIR}/install/setup.bash >/dev/null 2>&1 || true; cd ${REMOTE_AUTODRIVE_DIR}; exec python3 -c 'import gps.gps as m; m.main()' >/tmp/gps_node.log 2>&1"
vehicle_remote_launch_detached perception-vehicle2-gps-pose-bridge \
  "source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_AUTODRIVE_DIR}/install/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_MAPPING_ROS2_DIR}/install/setup.bash >/dev/null 2>&1 || true; exec ${REMOTE_MAPPING_ROS2_DIR}/install/data_sender_ros2/lib/data_sender_ros2/gps_pose_bridge_node --ros-args -p input_topic:=/gps_data -p output_topic:=/gps >/tmp/gps_pose_bridge_vehicle2.log 2>&1"
vehicle_remote_launch_detached perception-vehicle2-gps-imu-bridge \
  "source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_AUTODRIVE_DIR}/install/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_MAPPING_ROS2_DIR}/install/setup.bash >/dev/null 2>&1 || true; exec ${REMOTE_MAPPING_ROS2_DIR}/install/data_sender_ros2/lib/data_sender_ros2/gps_imu_bridge_node --ros-args -p input_topic:=/gps_data -p output_topic:=/imu -p frame_id:='${IMU_FRAME_ID}' -p angular_velocity_axes:='${ANGULAR_VELOCITY_AXES}' -p angular_velocity_signs:='[${ANGULAR_VELOCITY_SIGNS}]' -p linear_acceleration_axes:='${LINEAR_ACCELERATION_AXES}' -p linear_acceleration_signs:='[${LINEAR_ACCELERATION_SIGNS}]' -p linear_acceleration_scale:='${LINEAR_ACCELERATION_SCALE}' >/tmp/gps_imu_bridge_vehicle2.log 2>&1"
vehicle_remote_launch_detached perception-vehicle2-sender \
  "source /opt/ros/foxy/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_AUTODRIVE_DIR}/install/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_MAPPING_ROS2_DIR}/install/setup.bash >/dev/null 2>&1 || true; exec ${REMOTE_MAPPING_ROS2_DIR}/install/data_sender_ros2/lib/data_sender_ros2/sender_node --ros-args -p vehicle_id:=vehicle2 -p cloud_server_url:='${CLOUD_SERVER_URL}' -p cloud_topics:=/forward/rslidar_points,/left/rslidar_points,/right/rslidar_points -p gps_topic:=/gps -p send_interval_sec:='${SEND_INTERVAL_SEC}' >/tmp/sender_node_vehicle2.log 2>&1"

BRIDGE_BIN="$(resolve_bridge_bin)" || {
  echo "ros1_bridge dynamic_bridge not found on vehicle2" >&2
  exit 1
}

vehicle_remote_launch_detached perception-vehicle2-ros1-bridge \
  "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_AUTODRIVE_DIR}/install/setup.bash >/dev/null 2>&1 || true; if [[ -f ${REMOTE_MAPPING_ROS2_DIR}/install/setup.bash ]]; then source ${REMOTE_MAPPING_ROS2_DIR}/install/setup.bash >/dev/null 2>&1 || true; fi; if [[ -f /home/nvidia/ros2_ws/install/setup.bash ]]; then source /home/nvidia/ros2_ws/install/setup.bash >/dev/null 2>&1 || true; fi; export ROS_MASTER_URI=http://127.0.0.1:11311; export ROS_HOSTNAME=127.0.0.1; exec '${BRIDGE_BIN}' --bridge-all-2to1-topics >/tmp/ros1_bridge_vehicle2.log 2>&1"

vehicle_remote_launch_detached perception-vehicle2-fastlio \
  "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_MAPPING_CATKIN_DIR}/devel/setup.bash >/dev/null 2>&1 || true; exec roslaunch fast_lio2 mapping_robosense.launch lid_topic:=/forward/rslidar_points imu_topic:=/imu >/tmp/fastlio_mapping_vehicle2.log 2>&1"

sleep 2

vehicle_remote_launch_detached perception-vehicle2-fastlio-http-sender \
  "source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true; source ${REMOTE_MAPPING_CATKIN_DIR}/devel/setup.bash >/dev/null 2>&1 || true; exec ${REMOTE_MAPPING_CATKIN_DIR}/devel/lib/data_listener2/fastlio_http_sender __name:=fastlio_http_sender _vehicle_id:=vehicle2 _cloud_server_url:='${CLOUD_SERVER_URL}' _cloud_topic:=/cloud_registered _odom_topic:=/Odometry _send_interval_sec:='${SEND_INTERVAL_SEC}' >/tmp/fastlio_http_sender_vehicle2.log 2>&1"

echo vehicle_host=${HOSTNAME}
echo cloud_server_url=${CLOUD_SERVER_URL}
echo rslidar_log=/tmp/rslidar_sdk_vehicle2.log
echo gps_log=/tmp/gps_node.log
echo gps_pose_bridge_log=/tmp/gps_pose_bridge_vehicle2.log
echo gps_imu_bridge_log=/tmp/gps_imu_bridge_vehicle2.log
echo sender_log=/tmp/sender_node_vehicle2.log
echo ros1_bridge_log=/tmp/ros1_bridge_vehicle2.log
echo fastlio_log=/tmp/fastlio_mapping_vehicle2.log
echo fastlio_uploader_log=/tmp/fastlio_http_sender_vehicle2.log
EOF

cat <<EOF
Vehicle 2 mapping stack restart requested: ${VEHICLE_HOST}
Cloud server URL:      ${CLOUD_SERVER_URL}

Suggested checks:
  ssh ${SSH_OPTS[*]} ${VEHICLE_HOST} "bash -lc 'source /opt/ros/noetic/setup.bash >/dev/null 2>&1; rostopic info /imu; echo ---; rostopic info /forward/rslidar_points; echo ---; timeout 8 rostopic echo /Odometry -n 1 | egrep \"frame_id|child_frame_id\" || true'"
  ssh root@172.16.2.80 "nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io logs --tail 80 vehicle-cloud" | grep 'Received FAST-LIO data from vehicle2' | tail -n 10
EOF
