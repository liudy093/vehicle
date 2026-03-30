#pragma once

#include "Scancontext.h"

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <unordered_map>

namespace data_listener2 {

struct LidarExtrinsic {
    double x;
    double y;
    double z;
    double yaw_deg;
};

using LidarExtrinsicsMap = std::unordered_map<std::string, LidarExtrinsic>;

struct Ros2CloudPayload {
    std::string vehicle_id;
    std::string lidar_id;
    std::string cloud_topic;
    sensor_msgs::PointCloud2 cloud;
};

struct FastLioPayload {
    std::string vehicle_id;
    std::string cloud_topic;
    std::string odom_topic;
    sensor_msgs::PointCloud2 cloud;
    nav_msgs::Odometry odom;
};

struct ScanContextFeatures {
    Eigen::MatrixXd sc;
    Eigen::MatrixXd ring_key;
    Eigen::MatrixXd sector_key;
};

Ros2CloudPayload parseRos2CloudPayload(const std::string& request_body);
FastLioPayload parseFastLioPayload(const std::string& request_body);
LidarExtrinsicsMap parseLidarExtrinsicsConfig(const std::string& config);
sensor_msgs::PointCloud2 fuseCloudsToVehicleFrame(
    const std::string& vehicle_id,
    const std::unordered_map<std::string, sensor_msgs::PointCloud2>& clouds,
    const LidarExtrinsicsMap& extrinsics);
sensor_msgs::PointCloud2 transformCloudToGlobalFrame(
    const std::string& frame_id,
    const sensor_msgs::PointCloud2& cloud,
    const nav_msgs::Odometry& odom);
ScanContextFeatures buildScanContextFeatures(
    const sensor_msgs::PointCloud2& cloud,
    SCManager& sc_manager);
std::string buildCloudTopicName(
    const std::string& base_topic,
    const std::string& vehicle_id,
    const std::string& lidar_id);
std::string buildOptimizedGlobalTopicName(const std::string& topic_name);

}  // namespace data_listener2
