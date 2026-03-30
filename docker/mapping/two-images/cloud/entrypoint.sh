#!/usr/bin/env bash
set -eo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"

# ROS setup scripts assume some variables may be unset, so avoid nounset here.
set +u
source /opt/ros/noetic/setup.bash
source /workspace/cloud_ws/devel/setup.bash
set -u

SERVER_HOST="${SERVER_HOST:-0.0.0.0}"
SERVER_PORT="${SERVER_PORT:-5000}"
PCL_PUB_NAME="${PCL_PUB_NAME:-cloud_sent}"
WEB_VIEWER_PORT="${WEB_VIEWER_PORT:-18080}"
WEB_VIEWER_TOPIC="${WEB_VIEWER_TOPIC:-/vehicle1/${PCL_PUB_NAME}/forward}"
WEB_VIEWER_TOPICS="${WEB_VIEWER_TOPICS:-vehicle1/fused=/vehicle1/${PCL_PUB_NAME}/fused,vehicle1/global_cloud=/vehicle1/global_cloud,vehicle2/fused=/vehicle2/${PCL_PUB_NAME}/fused,vehicle2/global_cloud=/vehicle2/global_cloud,global/global_map=/global_map,global/global_map_optimized=/global_map_optimized}"
WEB_VIEWER_MAX_POINTS="${WEB_VIEWER_MAX_POINTS:-20000}"
FUSED_CLOUD_TOPIC_NAME="${FUSED_CLOUD_TOPIC_NAME:-fused}"
FUSED_LIDAR_IDS="${FUSED_LIDAR_IDS:-forward,left,right}"
FUSED_LIDAR_EXTRINSICS="${FUSED_LIDAR_EXTRINSICS:-forward:0.0,0.0,0.0,0;left:-0.8,-0.6,0.0,90;right:0.8,-0.6,0.0,-90}"
GLOBAL_MAP_EXPORT_DIR="${GLOBAL_MAP_EXPORT_DIR:-/tmp/perception_maps}"
GLOBAL_MAP_EXPORT_PREFIX="${GLOBAL_MAP_EXPORT_PREFIX:-global_map}"
GLOBAL_MAP_LEAF_SIZE="${GLOBAL_MAP_LEAF_SIZE:-0.15}"
GLOBAL_FRAME_LEAF_SIZE="${GLOBAL_FRAME_LEAF_SIZE:-0.1}"

cleanup() {
  jobs -pr | xargs -r kill || true
}
trap cleanup EXIT INT TERM

roscore &

until rosparam list >/dev/null 2>&1; do
  sleep 1
done

python3 /workspace/cloud_ws/src/data_sender/scripts/cloud_pointcloud_web_viewer.py \
  _topic_name:="${WEB_VIEWER_TOPIC}" \
  _topic_names:="${WEB_VIEWER_TOPICS}" \
  _bind_host:=0.0.0.0 \
  _bind_port:="${WEB_VIEWER_PORT}" \
  _max_points:="${WEB_VIEWER_MAX_POINTS}" &

exec rosrun data_listener2 cloud_recv \
  _cloud_server_host:="${SERVER_HOST}" \
  _cloud_server_port:="${SERVER_PORT}" \
  _pcl_pub_name:="${PCL_PUB_NAME}" \
  _fused_cloud_topic_name:="${FUSED_CLOUD_TOPIC_NAME}" \
  _fused_lidar_ids:="${FUSED_LIDAR_IDS}" \
  _fused_lidar_extrinsics:="${FUSED_LIDAR_EXTRINSICS}" \
  _optimized_global_map_topic_name:=global_map_optimized \
  _global_map_export_dir:="${GLOBAL_MAP_EXPORT_DIR}" \
  _global_map_export_prefix:="${GLOBAL_MAP_EXPORT_PREFIX}" \
  _global_map_leaf_size:="${GLOBAL_MAP_LEAF_SIZE}" \
  _global_frame_leaf_size:="${GLOBAL_FRAME_LEAF_SIZE}"
