#!/usr/bin/env bash

vehicle_layout_remote_mapping_dir() {
  printf '/home/nvidia/mapping\n'
}

vehicle_layout_remote_mapping_catkin_dir() {
  printf '%s/catkin_ws\n' "${REMOTE_MAPPING_DIR:-$(vehicle_layout_remote_mapping_dir)}"
}

vehicle_layout_remote_mapping_ros2_dir() {
  printf '%s/ros2_ws\n' "${REMOTE_MAPPING_DIR:-$(vehicle_layout_remote_mapping_dir)}"
}

vehicle_layout_remote_autodrive_dir() {
  printf '/home/nvidia/AutoDrive\n'
}
