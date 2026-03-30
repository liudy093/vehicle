#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /workspace/livox_ws/devel/setup.bash
source /workspace/vehicle/02mapping/catkin_ws/devel/setup.bash

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
LIVOX_CONFIG_DIR="${LIVOX_CONFIG_DIR:-/config/livox}"
LIVOX_DRIVER_DIR="/workspace/livox_ws/src/livox_ros_driver2"
OVERLAY_FILE="/tmp/mid360.container.yaml"

cleanup() {
  jobs -pr | xargs -r kill || true
}
trap cleanup EXIT INT TERM

if [[ -d "${LIVOX_CONFIG_DIR}" ]]; then
  if [[ -f "${LIVOX_CONFIG_DIR}/MID360_config.json" ]]; then
    cp "${LIVOX_CONFIG_DIR}/MID360_config.json" "${LIVOX_DRIVER_DIR}/config/MID360_config.json"
  fi
  if [[ -f "${LIVOX_CONFIG_DIR}/msg_MID360.launch" ]]; then
    cp "${LIVOX_CONFIG_DIR}/msg_MID360.launch" "${LIVOX_DRIVER_DIR}/launch_ROS1/msg_MID360.launch"
  fi
fi

cat > "${OVERLAY_FILE}" <<EOF
common:
  lid_topic: "${LID_TOPIC}"
  imu_topic: "${IMU_TOPIC}"
pcd_save:
  pcd_save_en: false
EOF

roscore &

until rosparam list >/dev/null 2>&1; do
  sleep 1
done

roslaunch livox_ros_driver2 msg_MID360.launch &

roslaunch fast_lio2 mapping_mid360.launch \
  rviz:="${USE_RVIZ}" \
  config_overlay_file:="${OVERLAY_FILE}" &

python3 /workspace/vehicle/docker/mapping/two-images/car/gps_bridge.py \
  --serial-port "${GPS_SERIAL_PORT}" \
  --baud-rate "${GPS_BAUD_RATE}" \
  --lat0 "${GPS_LAT0}" \
  --lon0 "${GPS_LON0}" &

if [[ "${USE_LOOP_CLOSURE}" == "true" ]]; then
  rosrun fast_lio2 loop_closure _odom_topic:=/Odometry _cloud_topic:=/cloud_registered &
fi

rosrun data_listener2 data_sender __name:="${SENDER_NAME}" _vehicle_id:="${VEHICLE_ID}" _cloud_server_url:="${CLOUD_SERVER_URL}" &

until rosservice list | grep -q "/${SENDER_NAME}/update_gps"; do
  sleep 1
done

until rosservice call "/${SENDER_NAME}/update_gps" >/dev/null 2>&1; do
  sleep 2
done

until rosservice call "/${SENDER_NAME}/sync_gps" >/dev/null 2>&1; do
  sleep 2
done

wait -n
