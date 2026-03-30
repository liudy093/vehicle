#!/usr/bin/env bash

vehicle_remote_stop_mapping_processes() {
  pkill -f "/opt/ros/foxy/bin/ros2 launch rslidar_sdk start.py" >/dev/null 2>&1 || true
  pkill -f "ros2 launch rslidar_sdk start.py" >/dev/null 2>&1 || true
  pkill -f "dynamic_bridge --bridge-all-2to1-topics" >/dev/null 2>&1 || true
  pkill -f "/opt/ros/foxy/lib/ros1_bridge/dynamic_bridge" >/dev/null 2>&1 || true
  pkill -f "/home/nvidia/ros2_ws/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge" >/dev/null 2>&1 || true
  pkill -f "/ros_bridge" >/dev/null 2>&1 || true
  pkill -f "fastlio_http_sender" >/dev/null 2>&1 || true
  pkill -f "/home/nvidia/mapping/catkin_ws/devel/lib/data_listener2/fastlio_http_sender" >/dev/null 2>&1 || true
  pkill -f "fastlio_mapping __name:=laserMapping" >/dev/null 2>&1 || true
  pkill -f "/home/nvidia/mapping/catkin_ws/devel/lib/fast_lio2/fastlio_mapping" >/dev/null 2>&1 || true
  pkill -f "/opt/ros/noetic/bin/roslaunch fast_lio2 mapping_robosense.launch" >/dev/null 2>&1 || true
  pkill -f "roslaunch fast_lio2 mapping_robosense.launch" >/dev/null 2>&1 || true
  pkill -f "ros2 launch ./launch/launch_defaults.py" >/dev/null 2>&1 || true
  pkill -f "python3 -c import gps.gps as m; m.main()" >/dev/null 2>&1 || true
  pkill -f "/home/nvidia/AutoDrive/install/gps/lib/gps/gps" >/dev/null 2>&1 || true
  pkill -x gps >/dev/null 2>&1 || true
  pkill -f "gps_pose_bridge_node" >/dev/null 2>&1 || true
  pkill -f "gps_imu_bridge_node" >/dev/null 2>&1 || true
  pkill -f "cgi410_raw_imu_bridge_node" >/dev/null 2>&1 || true
  pkill -f "cgi410_serial_bridge_node" >/dev/null 2>&1 || true
  pkill -f "data_sender_ros2.*sender_node" >/dev/null 2>&1 || true
  pkill -f "/home/nvidia/mapping/ros2_ws/install/data_sender_ros2/lib/data_sender_ros2/sender_node" >/dev/null 2>&1 || true
  pkill -f "/opt/ros/foxy/bin/ros2 launch rslidar_sdk start.py" >/dev/null 2>&1 || true
  pkill -f "rslidar_sdk_node" >/dev/null 2>&1 || true
}

vehicle_remote_stop_systemd_unit() {
  local unit_name="${1:?unit name is required}"

  if ! command -v systemctl >/dev/null 2>&1; then
    return 0
  fi

  systemctl stop "${unit_name}.service" >/dev/null 2>&1 || true
  systemctl reset-failed "${unit_name}.service" >/dev/null 2>&1 || true
}

vehicle_remote_launch_detached() {
  local unit_name="${1:?unit name is required}"
  local command="${2:?command is required}"
  local run_as_user="${3:-nvidia}"

  if [[ "${VEHICLE_EXECUTION_MODE:-ssh}" == "host" ]]; then
    if ! command -v systemd-run >/dev/null 2>&1; then
      echo "systemd-run is required for host-mode detached vehicle launches" >&2
      return 1
    fi

    vehicle_remote_stop_systemd_unit "${unit_name}"
    systemd-run --unit="${unit_name}" \
      --uid="${run_as_user}" --gid="${run_as_user}" \
      --collect \
      --setenv=HOME="/home/${run_as_user}" \
      --setenv=USER="${run_as_user}" \
      --setenv=LOGNAME="${run_as_user}" \
      --property=WorkingDirectory="/home/${run_as_user}" \
      /bin/bash -lc "${command}" >/dev/null
    return 0
  fi

  nohup /bin/bash -lc "${command}" >/dev/null 2>&1 < /dev/null &
}

vehicle_remote_runtime_functions() {
  declare -f vehicle_remote_stop_mapping_processes
  declare -f vehicle_remote_stop_systemd_unit
  declare -f vehicle_remote_launch_detached
}
