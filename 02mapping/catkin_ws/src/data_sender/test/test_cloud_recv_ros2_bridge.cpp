#include "data_listener2/cloud_recv_ros2_bridge.h"

#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

TEST(ParseRos2CloudPayloadTest, ReconstructsPointCloud2FromJsonPayload) {
    const std::string request_body = R"json(
{
  "vehicle_id": "vehicle2",
  "lidar_id": "forward",
  "cloud_topic": "/forward/rslidar_points",
  "header": {
    "stamp": {
      "sec": 12,
      "nanosec": 34
    },
    "frame_id": "rslidar"
  },
  "shape": {
    "height": 2,
    "width": 3
  },
  "fields": [
    {
      "name": "x",
      "offset": 0,
      "datatype": 7,
      "count": 1
    },
    {
      "name": "intensity",
      "offset": 4,
      "datatype": 7,
      "count": 1
    }
  ],
  "layout": {
    "is_bigendian": false,
    "point_step": 8,
    "row_step": 24,
    "is_dense": true
  },
  "gps": {
    "x": 1.0,
    "y": 2.0,
    "theta": 3.0
  },
  "data_b64": "AQIDBAUG"
}
)json";

    const data_listener2::Ros2CloudPayload payload =
        data_listener2::parseRos2CloudPayload(request_body);

    EXPECT_EQ(payload.vehicle_id, "vehicle2");
    EXPECT_EQ(payload.lidar_id, "forward");
    EXPECT_EQ(payload.cloud_topic, "/forward/rslidar_points");
    EXPECT_EQ(payload.cloud.header.stamp.sec, 12);
    EXPECT_EQ(payload.cloud.header.stamp.nsec, 34);
    EXPECT_EQ(payload.cloud.header.frame_id, "rslidar");
    EXPECT_EQ(payload.cloud.height, 2u);
    EXPECT_EQ(payload.cloud.width, 3u);
    ASSERT_EQ(payload.cloud.fields.size(), 2u);
    EXPECT_EQ(payload.cloud.fields[0].name, "x");
    EXPECT_EQ(payload.cloud.fields[0].offset, 0u);
    EXPECT_EQ(payload.cloud.fields[0].datatype, 7u);
    EXPECT_EQ(payload.cloud.fields[0].count, 1u);
    EXPECT_EQ(payload.cloud.fields[1].name, "intensity");
    EXPECT_EQ(payload.cloud.fields[1].offset, 4u);
    EXPECT_EQ(payload.cloud.fields[1].datatype, 7u);
    EXPECT_EQ(payload.cloud.fields[1].count, 1u);
    EXPECT_FALSE(payload.cloud.is_bigendian);
    EXPECT_EQ(payload.cloud.point_step, 8u);
    EXPECT_EQ(payload.cloud.row_step, 24u);
    EXPECT_TRUE(payload.cloud.is_dense);
    ASSERT_EQ(payload.cloud.data.size(), 6u);
    EXPECT_EQ(payload.cloud.data[0], 1u);
    EXPECT_EQ(payload.cloud.data[5], 6u);
}

TEST(ParseRos2CloudPayloadTest, ThrowsWhenRequiredFieldsAreMissing) {
    const std::string request_body = R"json({"vehicle_id":"vehicle2"})json";

    EXPECT_THROW(
        {
            try {
                (void)data_listener2::parseRos2CloudPayload(request_body);
            } catch (const std::runtime_error& error) {
                EXPECT_NE(std::string(error.what()).find("Missing required field"),
                          std::string::npos);
                throw;
            }
        },
        std::runtime_error);
}

TEST(ParseFastLioPayloadTest, ReconstructsCloudAndOdometryFromJsonPayload) {
    const std::string request_body = R"json(
{
  "vehicle_id": "vehicle3",
  "cloud_topic": "/cloud_registered",
  "odom_topic": "/Odometry",
  "header": {
    "stamp": {
      "sec": 21,
      "nanosec": 43
    },
    "frame_id": "body"
  },
  "shape": {
    "height": 1,
    "width": 2
  },
  "fields": [
    {
      "name": "x",
      "offset": 0,
      "datatype": 7,
      "count": 1
    }
  ],
  "layout": {
    "is_bigendian": false,
    "point_step": 16,
    "row_step": 32,
    "is_dense": true
  },
  "odom": {
    "header": {
      "stamp": {
        "sec": 30,
        "nanosec": 40
      },
      "frame_id": "camera_init"
    },
    "child_frame_id": "body",
    "pose": {
      "position": {
        "x": 1.0,
        "y": 2.0,
        "z": 3.0
      },
      "orientation": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "w": 1.0
      }
    }
  },
  "data_b64": "AQIDBA=="
}
)json";

    const data_listener2::FastLioPayload payload =
        data_listener2::parseFastLioPayload(request_body);

    EXPECT_EQ(payload.vehicle_id, "vehicle3");
    EXPECT_EQ(payload.cloud_topic, "/cloud_registered");
    EXPECT_EQ(payload.odom_topic, "/Odometry");
    EXPECT_EQ(payload.cloud.header.frame_id, "body");
    EXPECT_EQ(payload.cloud.header.stamp.sec, 21);
    EXPECT_EQ(payload.odom.header.frame_id, "camera_init");
    EXPECT_EQ(payload.odom.header.stamp.sec, 30);
    EXPECT_EQ(payload.odom.child_frame_id, "body");
    EXPECT_DOUBLE_EQ(payload.odom.pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(payload.odom.pose.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(payload.odom.pose.pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(payload.odom.pose.pose.orientation.w, 1.0);
    ASSERT_EQ(payload.cloud.data.size(), 4u);
    EXPECT_EQ(payload.cloud.data[0], 1u);
    EXPECT_EQ(payload.cloud.data[3], 4u);
}

TEST(BuildScanContextFeaturesTest, BuildsMatricesFromPointCloud2) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::PointXYZI point;

    point.x = 1.0F;
    point.y = 0.0F;
    point.z = 0.5F;
    point.intensity = 10.0F;
    pcl_cloud.push_back(point);

    point.x = 2.0F;
    point.y = 1.0F;
    point.z = 0.8F;
    point.intensity = 20.0F;
    pcl_cloud.push_back(point);

    point.x = 3.0F;
    point.y = -1.0F;
    point.z = 1.2F;
    point.intensity = 30.0F;
    pcl_cloud.push_back(point);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = "vehicle2";

    SCManager sc_manager;
    const data_listener2::ScanContextFeatures features =
        data_listener2::buildScanContextFeatures(cloud_msg, sc_manager);

    EXPECT_EQ(features.sc.rows(), sc_manager.PC_NUM_RING);
    EXPECT_EQ(features.sc.cols(), sc_manager.PC_NUM_SECTOR);
    EXPECT_EQ(features.ring_key.rows(), sc_manager.PC_NUM_RING);
    EXPECT_EQ(features.ring_key.cols(), 1);
    EXPECT_EQ(features.sector_key.rows(), 1);
    EXPECT_EQ(features.sector_key.cols(), sc_manager.PC_NUM_SECTOR);
    EXPECT_GT(features.sc.sum(), 0.0);
}

TEST(ParseLidarExtrinsicsConfigTest, ParsesCommaSeparatedExtrinsics) {
    const data_listener2::LidarExtrinsicsMap extrinsics =
        data_listener2::parseLidarExtrinsicsConfig(
            "forward:0.0,0.0,0.0,0;left:-1.0,0.5,0.0,90;right:1.0,0.5,0.0,-90");

    ASSERT_EQ(extrinsics.size(), 3u);
    EXPECT_DOUBLE_EQ(extrinsics.at("forward").x, 0.0);
    EXPECT_DOUBLE_EQ(extrinsics.at("left").x, -1.0);
    EXPECT_DOUBLE_EQ(extrinsics.at("left").y, 0.5);
    EXPECT_DOUBLE_EQ(extrinsics.at("left").yaw_deg, 90.0);
    EXPECT_DOUBLE_EQ(extrinsics.at("right").yaw_deg, -90.0);
}

TEST(FuseCloudsToVehicleFrameTest, AppliesYawAndTranslationPerLidar) {
    pcl::PointCloud<pcl::PointXYZI> forward_cloud;
    pcl::PointXYZI point;
    point.x = 1.0F;
    point.y = 0.0F;
    point.z = 0.2F;
    point.intensity = 10.0F;
    forward_cloud.push_back(point);

    pcl::PointCloud<pcl::PointXYZI> left_cloud;
    point.x = 1.0F;
    point.y = 0.0F;
    point.z = 0.5F;
    point.intensity = 20.0F;
    left_cloud.push_back(point);

    sensor_msgs::PointCloud2 forward_msg;
    pcl::toROSMsg(forward_cloud, forward_msg);
    forward_msg.header.stamp = ros::Time(1, 10);

    sensor_msgs::PointCloud2 left_msg;
    pcl::toROSMsg(left_cloud, left_msg);
    left_msg.header.stamp = ros::Time(2, 20);

    std::unordered_map<std::string, sensor_msgs::PointCloud2> clouds;
    clouds.emplace("forward", forward_msg);
    clouds.emplace("left", left_msg);

    data_listener2::LidarExtrinsicsMap extrinsics;
    extrinsics["forward"] = {0.0, 0.0, 0.0, 0.0};
    extrinsics["left"] = {0.0, 1.0, 0.0, 90.0};

    const sensor_msgs::PointCloud2 fused =
        data_listener2::fuseCloudsToVehicleFrame("vehicle2", clouds, extrinsics);

    EXPECT_EQ(fused.header.frame_id, "vehicle2/fused");
    EXPECT_EQ(fused.header.stamp.sec, 2);
    EXPECT_EQ(fused.header.stamp.nsec, 20);

    pcl::PointCloud<pcl::PointXYZI> fused_cloud;
    pcl::fromROSMsg(fused, fused_cloud);
    ASSERT_EQ(fused_cloud.size(), 2u);

    auto has_expected_point = [&](float x, float y, float z, float intensity) {
        return std::any_of(
            fused_cloud.begin(),
            fused_cloud.end(),
            [&](const pcl::PointXYZI& candidate) {
                return std::abs(candidate.x - x) < 1e-4F &&
                       std::abs(candidate.y - y) < 1e-4F &&
                       std::abs(candidate.z - z) < 1e-4F &&
                       std::abs(candidate.intensity - intensity) < 1e-4F;
            });
    };

    EXPECT_TRUE(has_expected_point(1.0F, 0.0F, 0.2F, 10.0F));
    EXPECT_TRUE(has_expected_point(0.0F, 2.0F, 0.5F, 20.0F));
}

TEST(TransformCloudToGlobalFrameTest, AppliesOdometryRotationAndTranslation) {
    pcl::PointCloud<pcl::PointXYZI> source_cloud;
    pcl::PointXYZI point;
    point.x = 1.0F;
    point.y = 0.0F;
    point.z = 0.5F;
    point.intensity = 15.0F;
    source_cloud.push_back(point);

    sensor_msgs::PointCloud2 source_msg;
    pcl::toROSMsg(source_cloud, source_msg);
    source_msg.header.stamp = ros::Time(5, 6);

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = 10.0;
    odom.pose.pose.position.y = 20.0;
    odom.pose.pose.position.z = 1.0;
    odom.pose.pose.orientation.z = std::sqrt(0.5);
    odom.pose.pose.orientation.w = std::sqrt(0.5);

    const sensor_msgs::PointCloud2 transformed =
        data_listener2::transformCloudToGlobalFrame("vehicle2/global", source_msg, odom);

    EXPECT_EQ(transformed.header.frame_id, "vehicle2/global");
    EXPECT_EQ(transformed.header.stamp.sec, 5);

    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
    pcl::fromROSMsg(transformed, transformed_cloud);
    ASSERT_EQ(transformed_cloud.size(), 1u);
    EXPECT_NEAR(transformed_cloud[0].x, 10.0F, 1e-4F);
    EXPECT_NEAR(transformed_cloud[0].y, 21.0F, 1e-4F);
    EXPECT_NEAR(transformed_cloud[0].z, 1.5F, 1e-4F);
    EXPECT_NEAR(transformed_cloud[0].intensity, 15.0F, 1e-4F);
}

TEST(BuildCloudTopicNameTest, BuildsCompatibilityAndVehicleScopedTopics) {
    EXPECT_EQ(
        data_listener2::buildCloudTopicName("/cloud_sent", "", "forward"),
        "/cloud_sent/forward");
    EXPECT_EQ(
        data_listener2::buildCloudTopicName("/cloud_sent", "", "fused"),
        "/cloud_sent/fused");
    EXPECT_EQ(
        data_listener2::buildCloudTopicName("/cloud_sent", "vehicle2", "forward"),
        "/vehicle2/cloud_sent/forward");
    EXPECT_EQ(
        data_listener2::buildCloudTopicName("/cloud_sent", "vehicle2", "fused"),
        "/vehicle2/cloud_sent/fused");
    EXPECT_EQ(
        data_listener2::buildCloudTopicName("/cloud_sent", "vehicle2", ""),
        "/vehicle2/cloud_sent");
}

TEST(BuildOptimizedGlobalTopicNameTest, ReturnsOptimizedGlobalMapTopic) {
    EXPECT_EQ(
        data_listener2::buildOptimizedGlobalTopicName("global_map_optimized"),
        "/global_map_optimized");
}

}  // namespace
