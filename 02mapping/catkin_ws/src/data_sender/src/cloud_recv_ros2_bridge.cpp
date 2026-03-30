#include "data_listener2/cloud_recv_ros2_bridge.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace data_listener2 {

namespace {

using boost::property_tree::ptree;
constexpr double kPi = 3.14159265358979323846;

const ptree& requireChild(const ptree& value, const char* key) {
    const auto child = value.get_child_optional(key);
    if (!child) {
        throw std::runtime_error(std::string("Missing required field: ") + key);
    }
    return child.get();
}

template <typename T>
T requireValue(const ptree& value, const char* key) {
    const auto parsed = value.get_optional<T>(key);
    if (!parsed) {
        throw std::runtime_error(std::string("Missing required field: ") + key);
    }
    return parsed.get();
}

std::vector<uint8_t> decodeBase64(const std::string& encoded) {
    static const std::array<int8_t, 256> kDecodeTable = [] {
        std::array<int8_t, 256> table{};
        table.fill(-1);
        for (int i = 'A'; i <= 'Z'; ++i) table[i] = i - 'A';
        for (int i = 'a'; i <= 'z'; ++i) table[i] = i - 'a' + 26;
        for (int i = '0'; i <= '9'; ++i) table[i] = i - '0' + 52;
        table[static_cast<size_t>('+')] = 62;
        table[static_cast<size_t>('/')] = 63;
        return table;
    }();

    if (encoded.size() % 4 != 0) {
        throw std::runtime_error("Invalid base64 length");
    }

    std::vector<uint8_t> decoded;
    decoded.reserve((encoded.size() / 4) * 3);

    for (size_t index = 0; index < encoded.size(); index += 4) {
        const char c0 = encoded[index];
        const char c1 = encoded[index + 1];
        const char c2 = encoded[index + 2];
        const char c3 = encoded[index + 3];

        if (kDecodeTable[static_cast<uint8_t>(c0)] < 0 ||
            kDecodeTable[static_cast<uint8_t>(c1)] < 0 ||
            (c2 != '=' && kDecodeTable[static_cast<uint8_t>(c2)] < 0) ||
            (c3 != '=' && kDecodeTable[static_cast<uint8_t>(c3)] < 0)) {
            throw std::runtime_error("Invalid base64 character");
        }

        const uint32_t sextet_a = static_cast<uint32_t>(kDecodeTable[static_cast<uint8_t>(c0)]);
        const uint32_t sextet_b = static_cast<uint32_t>(kDecodeTable[static_cast<uint8_t>(c1)]);
        const uint32_t sextet_c = c2 == '=' ? 0U : static_cast<uint32_t>(kDecodeTable[static_cast<uint8_t>(c2)]);
        const uint32_t sextet_d = c3 == '=' ? 0U : static_cast<uint32_t>(kDecodeTable[static_cast<uint8_t>(c3)]);

        const uint32_t triple =
            (sextet_a << 18U) | (sextet_b << 12U) | (sextet_c << 6U) | sextet_d;

        decoded.push_back(static_cast<uint8_t>((triple >> 16U) & 0xFFU));
        if (c2 != '=') {
            decoded.push_back(static_cast<uint8_t>((triple >> 8U) & 0xFFU));
        }
        if (c3 != '=') {
            decoded.push_back(static_cast<uint8_t>(triple & 0xFFU));
        }
    }

    return decoded;
}

sensor_msgs::PointField parsePointField(const ptree& field_json) {
    sensor_msgs::PointField field;
    field.name = requireValue<std::string>(field_json, "name");
    field.offset = requireValue<uint32_t>(field_json, "offset");
    field.datatype = requireValue<uint8_t>(field_json, "datatype");
    field.count = requireValue<uint32_t>(field_json, "count");
    return field;
}

std::vector<std::string> splitString(const std::string& value, char delimiter) {
    std::vector<std::string> parts;
    std::stringstream stream(value);
    std::string item;
    while (std::getline(stream, item, delimiter)) {
        if (!item.empty()) {
            parts.push_back(item);
        }
    }
    return parts;
}

}  // namespace

Ros2CloudPayload parseRos2CloudPayload(const std::string& request_body) {
    std::istringstream stream(request_body);
    ptree payload;
    boost::property_tree::read_json(stream, payload);

    Ros2CloudPayload result;
    result.vehicle_id = requireValue<std::string>(payload, "vehicle_id");
    result.lidar_id = requireValue<std::string>(payload, "lidar_id");
    result.cloud_topic = requireValue<std::string>(payload, "cloud_topic");

    const ptree& header = requireChild(payload, "header");
    const ptree& stamp = requireChild(header, "stamp");
    const ptree& shape = requireChild(payload, "shape");
    const ptree& layout = requireChild(payload, "layout");
    const ptree& fields = requireChild(payload, "fields");

    result.cloud.header.stamp =
        ros::Time(requireValue<uint32_t>(stamp, "sec"),
                  requireValue<uint32_t>(stamp, "nanosec"));
    result.cloud.header.frame_id = requireValue<std::string>(header, "frame_id");
    result.cloud.height = requireValue<uint32_t>(shape, "height");
    result.cloud.width = requireValue<uint32_t>(shape, "width");
    result.cloud.is_bigendian = requireValue<bool>(layout, "is_bigendian");
    result.cloud.point_step = requireValue<uint32_t>(layout, "point_step");
    result.cloud.row_step = requireValue<uint32_t>(layout, "row_step");
    result.cloud.is_dense = requireValue<bool>(layout, "is_dense");

    for (const auto& field_entry : fields) {
        result.cloud.fields.push_back(parsePointField(field_entry.second));
    }

    result.cloud.data = decodeBase64(requireValue<std::string>(payload, "data_b64"));

    return result;
}

FastLioPayload parseFastLioPayload(const std::string& request_body) {
    std::istringstream stream(request_body);
    ptree payload;
    boost::property_tree::read_json(stream, payload);

    FastLioPayload result;
    result.vehicle_id = requireValue<std::string>(payload, "vehicle_id");
    result.cloud_topic = requireValue<std::string>(payload, "cloud_topic");
    result.odom_topic = requireValue<std::string>(payload, "odom_topic");

    const ptree& header = requireChild(payload, "header");
    const ptree& stamp = requireChild(header, "stamp");
    const ptree& shape = requireChild(payload, "shape");
    const ptree& layout = requireChild(payload, "layout");
    const ptree& fields = requireChild(payload, "fields");

    result.cloud.header.stamp =
        ros::Time(requireValue<uint32_t>(stamp, "sec"),
                  requireValue<uint32_t>(stamp, "nanosec"));
    result.cloud.header.frame_id = requireValue<std::string>(header, "frame_id");
    result.cloud.height = requireValue<uint32_t>(shape, "height");
    result.cloud.width = requireValue<uint32_t>(shape, "width");
    result.cloud.is_bigendian = requireValue<bool>(layout, "is_bigendian");
    result.cloud.point_step = requireValue<uint32_t>(layout, "point_step");
    result.cloud.row_step = requireValue<uint32_t>(layout, "row_step");
    result.cloud.is_dense = requireValue<bool>(layout, "is_dense");

    for (const auto& field_entry : fields) {
        result.cloud.fields.push_back(parsePointField(field_entry.second));
    }
    result.cloud.data = decodeBase64(requireValue<std::string>(payload, "data_b64"));

    const ptree& odom = requireChild(payload, "odom");
    const ptree& odom_header = requireChild(odom, "header");
    const ptree& odom_stamp = requireChild(odom_header, "stamp");
    const ptree& pose = requireChild(odom, "pose");
    const ptree& position = requireChild(pose, "position");
    const ptree& orientation = requireChild(pose, "orientation");

    result.odom.header.stamp =
        ros::Time(requireValue<uint32_t>(odom_stamp, "sec"),
                  requireValue<uint32_t>(odom_stamp, "nanosec"));
    result.odom.header.frame_id = requireValue<std::string>(odom_header, "frame_id");
    result.odom.child_frame_id = requireValue<std::string>(odom, "child_frame_id");
    result.odom.pose.pose.position.x = requireValue<double>(position, "x");
    result.odom.pose.pose.position.y = requireValue<double>(position, "y");
    result.odom.pose.pose.position.z = requireValue<double>(position, "z");
    result.odom.pose.pose.orientation.x = requireValue<double>(orientation, "x");
    result.odom.pose.pose.orientation.y = requireValue<double>(orientation, "y");
    result.odom.pose.pose.orientation.z = requireValue<double>(orientation, "z");
    result.odom.pose.pose.orientation.w = requireValue<double>(orientation, "w");

    return result;
}

LidarExtrinsicsMap parseLidarExtrinsicsConfig(const std::string& config) {
    LidarExtrinsicsMap extrinsics;
    if (config.empty()) {
        return extrinsics;
    }

    for (const std::string& entry : splitString(config, ';')) {
        const size_t separator = entry.find(':');
        if (separator == std::string::npos) {
            throw std::runtime_error("Invalid extrinsic entry: " + entry);
        }

        const std::string lidar_id = entry.substr(0, separator);
        const std::vector<std::string> values = splitString(entry.substr(separator + 1), ',');
        if (values.size() != 4U) {
            throw std::runtime_error("Extrinsic entry must contain x,y,z,yaw: " + entry);
        }

        extrinsics[lidar_id] = {
            std::stod(values[0]),
            std::stod(values[1]),
            std::stod(values[2]),
            std::stod(values[3]),
        };
    }

    return extrinsics;
}

sensor_msgs::PointCloud2 fuseCloudsToVehicleFrame(
    const std::string& vehicle_id,
    const std::unordered_map<std::string, sensor_msgs::PointCloud2>& clouds,
    const LidarExtrinsicsMap& extrinsics) {
    pcl::PointCloud<SCPointType> fused_cloud;
    ros::Time latest_stamp(0, 0);

    for (const auto& entry : clouds) {
        pcl::PointCloud<SCPointType> source_cloud;
        pcl::fromROSMsg(entry.second, source_cloud);
        if (source_cloud.empty()) {
            continue;
        }

        const auto found = extrinsics.find(entry.first);
        const LidarExtrinsic transform =
            found != extrinsics.end() ? found->second : LidarExtrinsic{0.0, 0.0, 0.0, 0.0};

        const double yaw_rad = transform.yaw_deg * kPi / 180.0;
        const double cos_yaw = std::cos(yaw_rad);
        const double sin_yaw = std::sin(yaw_rad);

        for (const auto& point : source_cloud) {
            SCPointType transformed = point;
            transformed.x = static_cast<float>(point.x * cos_yaw - point.y * sin_yaw + transform.x);
            transformed.y = static_cast<float>(point.x * sin_yaw + point.y * cos_yaw + transform.y);
            transformed.z = static_cast<float>(point.z + transform.z);
            fused_cloud.push_back(transformed);
        }

        if (entry.second.header.stamp > latest_stamp) {
            latest_stamp = entry.second.header.stamp;
        }
    }

    sensor_msgs::PointCloud2 fused_msg;
    pcl::toROSMsg(fused_cloud, fused_msg);
    fused_msg.header.stamp = latest_stamp;
    fused_msg.header.frame_id = vehicle_id + "/fused";
    return fused_msg;
}

sensor_msgs::PointCloud2 transformCloudToGlobalFrame(
    const std::string& frame_id,
    const sensor_msgs::PointCloud2& cloud,
    const nav_msgs::Odometry& odom) {
    pcl::PointCloud<SCPointType> source_cloud;
    pcl::fromROSMsg(cloud, source_cloud);

    pcl::PointCloud<SCPointType> transformed_cloud;
    transformed_cloud.reserve(source_cloud.size());

    const auto& orientation = odom.pose.pose.orientation;
    Eigen::Quaterniond quaternion(
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z);
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const Eigen::Vector3d translation(
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z);

    for (const auto& point : source_cloud) {
        const Eigen::Vector3d original(point.x, point.y, point.z);
        const Eigen::Vector3d updated = rotation * original + translation;
        SCPointType transformed = point;
        transformed.x = static_cast<float>(updated.x());
        transformed.y = static_cast<float>(updated.y());
        transformed.z = static_cast<float>(updated.z());
        transformed_cloud.push_back(transformed);
    }

    sensor_msgs::PointCloud2 transformed_msg;
    pcl::toROSMsg(transformed_cloud, transformed_msg);
    transformed_msg.header.stamp = cloud.header.stamp;
    transformed_msg.header.frame_id = frame_id;
    return transformed_msg;
}

ScanContextFeatures buildScanContextFeatures(
    const sensor_msgs::PointCloud2& cloud,
    SCManager& sc_manager) {
    pcl::PointCloud<SCPointType> pcl_cloud;
    pcl::fromROSMsg(cloud, pcl_cloud);
    if (pcl_cloud.empty()) {
        throw std::runtime_error("PointCloud2 payload is empty");
    }

    ScanContextFeatures features;
    features.sc = sc_manager.makeScancontext(pcl_cloud);
    features.ring_key = sc_manager.makeRingkeyFromScancontext(features.sc);
    features.sector_key = sc_manager.makeSectorkeyFromScancontext(features.sc);
    return features;
}

std::string buildCloudTopicName(
    const std::string& base_topic,
    const std::string& vehicle_id,
    const std::string& lidar_id) {
    std::string normalized_base = base_topic;
    if (normalized_base.empty()) {
        normalized_base = "/";
    } else if (normalized_base.front() != '/') {
        normalized_base = "/" + normalized_base;
    }

    std::string topic_name = normalized_base;
    if (!vehicle_id.empty()) {
        topic_name = "/" + vehicle_id + normalized_base;
    }
    if (!lidar_id.empty()) {
        topic_name += "/" + lidar_id;
    }
    return topic_name;
}

std::string buildOptimizedGlobalTopicName(const std::string& topic_name) {
    if (topic_name.empty()) {
        return "/global_map_optimized";
    }
    if (topic_name.front() == '/') {
        return topic_name;
    }
    return "/" + topic_name;
}

}  // namespace data_listener2
