// #include <ros/ros.h>
// #include <ros/serialization.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <ros/serialization.h>

// #include "httplib.h"
// #include <nlohmann/json.hpp>
// #include <signal.h>
// #include <memory>
// #include <unordered_map>
// #include <iostream>
// #include <geometry_msgs/Pose2D.h>

// #include <tf2_ros/static_transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf2/LinearMath/Quaternion.h>

// #include "data_listener2/multi_lidar.h"

// typedef struct {
//     geometry_msgs::Pose2D gps;
//     bool gps_ready;
//     geometry_msgs::TransformStamped transStamped;
// } VehicleStatus;

// // 使用hash表保存车辆ID和对应的GPS数据
// std::unordered_map<std::string, VehicleStatus> vehicle_status_map;

// void signalHandler(int signum) {
//     ROS_INFO("Shutdown signal received (%d). Exiting...", signum);
//     ros::shutdown();
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "cloud_datahub");
//     ros::NodeHandle nh;
    
//     signal(SIGINT, signalHandler);
//     signal(SIGTERM, signalHandler);

//     // 获取车辆id参数
//     std::string cloud_server_host, pcl_pub_name;
//     int cloud_server_port;
//     nh.param<std::string>("cloud_server_host", cloud_server_host, "0.0.0.0");
//     nh.param<int>("cloud_server_port", cloud_server_port, 5000);
//     nh.param<std::string>("pcl_pub_name", pcl_pub_name, "cloud_registered");
    
//     std::cout << "Cloud server host: " << cloud_server_host << std::endl;
//     std::cout << "Cloud server port: " << cloud_server_port << std::endl;
//     std::cout << "Point cloud publisher name: " << pcl_pub_name << std::endl;

//     ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pcl_pub_name, 10);

//     tf2_ros::StaticTransformBroadcaster static_broadcaster;

//     // http_server
//     httplib::Server svr;

//     svr.Post("/sync_gps", [&](const httplib::Request& req, httplib::Response& res) {
//         nlohmann::json json_data = nlohmann::json::parse(req.body);
//         if (json_data.contains("vehicle_id") && json_data.contains("x") && json_data.contains("y") && json_data.contains("theta")) {
            
//             // 如果hash表中没有该车辆ID，则初始化
//             if (vehicle_status_map.find(json_data["vehicle_id"]) == vehicle_status_map.end()) {
//                 vehicle_status_map[json_data["vehicle_id"]] = {geometry_msgs::Pose2D(), false};
//             }
//             vehicle_status_map[json_data["vehicle_id"]].gps.x = json_data["x"];
//             vehicle_status_map[json_data["vehicle_id"]].gps.y = json_data["y"];
//             vehicle_status_map[json_data["vehicle_id"]].gps.theta = json_data["theta"];
//             vehicle_status_map[json_data["vehicle_id"]].gps_ready = true;
//             ROS_INFO("Received GPS data for vehicle %s: x=%.5f, y=%.5f, theta=%.5f",
//                      json_data["vehicle_id"].get<std::string>().c_str(),
//                      vehicle_status_map[json_data["vehicle_id"]].gps.x,
//                      vehicle_status_map[json_data["vehicle_id"]].gps.y,
//                      vehicle_status_map[json_data["vehicle_id"]].gps.theta);

//             // config tf transformation
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.header.stamp = ros::Time::now(); 
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.header.frame_id = "map";
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.child_frame_id = json_data["vehicle_id"];
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.translation.x = json_data["x"];
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.translation.y = json_data["y"];
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.translation.z = 0.0;
//             tf2::Quaternion q;
//             q.setRPY(0, 0, json_data["theta"]);
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.x = q.x();
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.y = q.y();
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.z = q.z();
//             vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.w = q.w();  
//             // send tf static transform
//             static_broadcaster.sendTransform(vehicle_status_map[json_data["vehicle_id"]].transStamped);

//             ROS_INFO("Static transform for vehicle %s published: translation=(%.2f, %.2f), rotation=(%.2f, %.2f, %.2f, %.2f)",
//                      json_data["vehicle_id"].get<std::string>().c_str(),
//                      vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.translation.x,
//                      vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.translation.y,
//                      vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.x,
//                      vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.y,
//                      vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.z,
//                      vehicle_status_map[json_data["vehicle_id"]].transStamped.transform.rotation.w);
            
//             res.status = 200;
//             res.set_content("GPS data received successfully", "text/plain");
//             return;
//         } else {
//             res.status = 400;
//             res.set_content("bad data: missing vehicle_id or gps data", "text/plain");
//             return;
//         }
//     });

//     svr.Post("/receive", [&cloud_pub](const httplib::Request& req, httplib::Response& res) {
//         data_listener2::multi_lidar cloud_data;
//         ros::serialization::IStream stream(
//                 reinterpret_cast<uint8_t*>(const_cast<char*>(req.body.data())),
//                 req.body.size());
//         ros::serialization::deserialize(stream, cloud_data);
        
//         sensor_msgs::PointCloud2 cloud_msg = cloud_data.cloud;
//         cloud_msg.header.stamp = ros::Time::now();
//         cloud_msg.header.frame_id = cloud_data.vehicle_id;
//         cloud_pub.publish(cloud_msg);
//         res.status = 200;
//         res.set_content("Point cloud data received successfully", "text/plain");
//         ROS_INFO("Received point cloud data from vehicle %s", cloud_data.vehicle_id.c_str());
//     });

//     if (!svr.listen(cloud_server_host, cloud_server_port)) {
//         ROS_FATAL("Failed to start server on %s:%d", cloud_server_host.c_str(), cloud_server_port);
//         return 1;
//     }
//     ROS_INFO("Server running on %s:%d with base frame_id=\"map\"", cloud_server_host.c_str(), cloud_server_port);

//     ros::spin();
//     return 0;
// }


#include <ros/ros.h>
#include <ros/serialization.h>
#include "Scancontext.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose2D.h>  // 添加这行


#include "httplib.h"
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <signal.h>
#include <algorithm>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <thread>
#include <chrono>
#include <queue>
#include <sstream>
#include <cerrno>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

// 引用自定义消息
#include "data_listener2/multi_lidar.h"
#include "data_listener2/cloud_recv_ros2_bridge.h"

// ScanContext相关头文件

// PCL相关
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// 类型定义
using SCPointType = pcl::PointXYZI;
using namespace std::chrono_literals;
using boost::property_tree::ptree;

// 车辆数据结构
struct LidarStreamData {
    std::string lidar_id;
    std::string cloud_topic;
    sensor_msgs::PointCloud2 latest_cloud;  // 最新点云
    Eigen::MatrixXd latest_sc;  // 最新ScanContext
    Eigen::MatrixXd latest_ring_key;  // 最新ring键
    Eigen::MatrixXd latest_sector_key;  // 最新sector键
    std::vector<Eigen::MatrixXd> sc_history;  // ScanContext历史
    std::vector<geometry_msgs::Pose2D> pose_history;  // 位姿历史
};

struct VehicleData {
    VehicleData() : accumulated_global_cloud(new pcl::PointCloud<SCPointType>()) {}

    std::string vehicle_id;
    geometry_msgs::Pose2D gps_pose;  // GPS位姿
    bool gps_ready = false;
    std::unordered_map<std::string, LidarStreamData> lidar_streams;
    nav_msgs::Odometry latest_fastlio_odom;
    std::string fastlio_cloud_topic;
    std::string fastlio_odom_topic;
    sensor_msgs::PointCloud2 latest_registered_cloud;
    sensor_msgs::PointCloud2 latest_global_cloud;
    bool fastlio_ready = false;
    pcl::PointCloud<SCPointType>::Ptr accumulated_global_cloud;
    std::mutex data_mutex;  // 数据保护锁
};

// 全局变量
std::unordered_map<std::string, VehicleData> vehicle_map;  // 车辆数据映射
std::mutex vehicle_map_mutex;  // 车辆映射锁
SCManager sc_manager;  // ScanContext管理器
ros::Publisher loop_detection_pub;  // 回环检测结果发布者
std::unordered_map<std::string, ros::Publisher> cloud_pubs;  // 分路点云发布者
std::mutex cloud_pub_mutex;
std::string base_cloud_topic;
std::string fused_cloud_topic_name = "fused";
std::string global_cloud_topic_name = "global_cloud";
std::string global_map_topic_name = "global_map";
std::string optimized_global_map_topic_name = "global_map_optimized";
std::string global_map_export_dir = "/tmp/perception_maps";
std::string global_map_export_prefix = "global_map";
std::unordered_set<std::string> fused_lidar_ids;
data_listener2::LidarExtrinsicsMap fused_lidar_extrinsics;
double global_map_leaf_size = 0.15;
double global_frame_leaf_size = 0.1;
size_t global_map_max_points = 250000;
pcl::PointCloud<SCPointType>::Ptr global_map_cloud(new pcl::PointCloud<SCPointType>());
std::mutex global_map_mutex;
pcl::PointCloud<SCPointType>::Ptr optimized_global_map_cloud(new pcl::PointCloud<SCPointType>());
std::mutex optimized_global_map_mutex;
std::unordered_map<std::string, geometry_msgs::Pose2D> optimized_vehicle_offsets;
std::mutex optimized_vehicle_offsets_mutex;

const LidarStreamData* selectPrimaryLidarStream(const VehicleData& vehicle) {
    const auto preferred = vehicle.lidar_streams.find("forward");
    if (preferred != vehicle.lidar_streams.end() && !preferred->second.sc_history.empty()) {
        return &preferred->second;
    }

    for (const auto& entry : vehicle.lidar_streams) {
        if (!entry.second.sc_history.empty()) {
            return &entry.second;
        }
    }
    return nullptr;
}

ros::Publisher getOrCreateCloudPublisher(ros::NodeHandle& nh, const std::string& topic_name) {
    std::lock_guard<std::mutex> lock(cloud_pub_mutex);

    const auto existing = cloud_pubs.find(topic_name);
    if (existing != cloud_pubs.end()) {
        return existing->second;
    }

    ros::Publisher publisher = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 10);
    cloud_pubs.emplace(topic_name, publisher);
    return publisher;
}

void publishCloudMessage(
    ros::NodeHandle& nh,
    const std::string& vehicle_id,
    const std::string& lidar_id,
    const sensor_msgs::PointCloud2& cloud,
    bool publish_vehicle_scoped) {
    const std::string compatibility_topic =
        data_listener2::buildCloudTopicName(base_cloud_topic, "", lidar_id);
    getOrCreateCloudPublisher(nh, compatibility_topic).publish(cloud);

    if (!publish_vehicle_scoped || vehicle_id.empty()) {
        return;
    }

    const std::string vehicle_scoped_topic =
        data_listener2::buildCloudTopicName(base_cloud_topic, vehicle_id, lidar_id);
    getOrCreateCloudPublisher(nh, vehicle_scoped_topic).publish(cloud);
}

void publishDirectCloudMessage(
    ros::NodeHandle& nh,
    const std::string& topic_name,
    const sensor_msgs::PointCloud2& cloud) {
    getOrCreateCloudPublisher(nh, topic_name).publish(cloud);
}

sensor_msgs::PointCloud2 downsampleCloudMessage(
    const sensor_msgs::PointCloud2& cloud,
    double leaf_size) {
    if (cloud.data.empty() || leaf_size <= 0.0) {
        return cloud;
    }

    pcl::PointCloud<SCPointType> source_cloud;
    pcl::fromROSMsg(cloud, source_cloud);
    if (source_cloud.empty()) {
        return cloud;
    }

    pcl::PointCloud<SCPointType> filtered_cloud;
    pcl::VoxelGrid<SCPointType> voxel_filter;
    voxel_filter.setLeafSize(
        static_cast<float>(leaf_size),
        static_cast<float>(leaf_size),
        static_cast<float>(leaf_size));
    voxel_filter.setInputCloud(source_cloud.makeShared());
    voxel_filter.filter(filtered_cloud);

    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(filtered_cloud, filtered_msg);
    filtered_msg.header = cloud.header;
    return filtered_msg;
}

void appendCloudToAccumulator(
    const sensor_msgs::PointCloud2& cloud,
    pcl::PointCloud<SCPointType>::Ptr& accumulator,
    double leaf_size,
    size_t max_points) {
    if (cloud.data.empty()) {
        return;
    }

    pcl::PointCloud<SCPointType> source_cloud;
    pcl::fromROSMsg(cloud, source_cloud);
    if (source_cloud.empty()) {
        return;
    }

    *accumulator += source_cloud;
    if ((max_points > 0U && accumulator->size() > max_points) || leaf_size > 0.0) {
        pcl::VoxelGrid<SCPointType> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(leaf_size),
            static_cast<float>(leaf_size),
            static_cast<float>(leaf_size));
        voxel_filter.setInputCloud(accumulator);
        pcl::PointCloud<SCPointType> filtered_cloud;
        voxel_filter.filter(filtered_cloud);
        accumulator->swap(filtered_cloud);
    }
}

sensor_msgs::PointCloud2 pointCloudToRosMessage(
    const pcl::PointCloud<SCPointType>::Ptr& cloud,
    const std::string& frame_id,
    const ros::Time& stamp) {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header.frame_id = frame_id;
    message.header.stamp = stamp;
    return message;
}

pcl::PointCloud<SCPointType>::Ptr transformAccumulatedCloud(
    const pcl::PointCloud<SCPointType>::Ptr& source,
    const geometry_msgs::Pose2D& offset) {
    pcl::PointCloud<SCPointType>::Ptr transformed(new pcl::PointCloud<SCPointType>());
    if (source == nullptr || source->empty()) {
        return transformed;
    }

    transformed->reserve(source->size());
    const double cos_yaw = std::cos(offset.theta);
    const double sin_yaw = std::sin(offset.theta);

    for (const auto& point : *source) {
        SCPointType updated = point;
        updated.x = static_cast<float>(point.x * cos_yaw - point.y * sin_yaw + offset.x);
        updated.y = static_cast<float>(point.x * sin_yaw + point.y * cos_yaw + offset.y);
        transformed->push_back(updated);
    }
    return transformed;
}

void rebuildOptimizedGlobalMap(ros::NodeHandle& nh) {
    std::vector<std::pair<std::string, pcl::PointCloud<SCPointType>::Ptr>> vehicle_clouds;
    {
        std::lock_guard<std::mutex> vehicle_lock(vehicle_map_mutex);
        vehicle_clouds.reserve(vehicle_map.size());
        for (auto& entry : vehicle_map) {
            std::lock_guard<std::mutex> data_lock(entry.second.data_mutex);
            if (!entry.second.accumulated_global_cloud ||
                entry.second.accumulated_global_cloud->empty()) {
                continue;
            }
            pcl::PointCloud<SCPointType>::Ptr snapshot(new pcl::PointCloud<SCPointType>());
            *snapshot = *entry.second.accumulated_global_cloud;
            vehicle_clouds.emplace_back(entry.first, snapshot);
        }
    }

    pcl::PointCloud<SCPointType>::Ptr rebuilt(new pcl::PointCloud<SCPointType>());
    {
        std::lock_guard<std::mutex> offset_lock(optimized_vehicle_offsets_mutex);
        for (const auto& entry : vehicle_clouds) {
            const auto found = optimized_vehicle_offsets.find(entry.first);
            const geometry_msgs::Pose2D offset =
                found != optimized_vehicle_offsets.end() ? found->second : geometry_msgs::Pose2D();
            pcl::PointCloud<SCPointType>::Ptr transformed =
                transformAccumulatedCloud(entry.second, offset);
            *rebuilt += *transformed;
        }
    }

    if (rebuilt->empty()) {
        return;
    }

    if (global_map_leaf_size > 0.0) {
        pcl::VoxelGrid<SCPointType> voxel_filter;
        voxel_filter.setLeafSize(
            static_cast<float>(global_map_leaf_size),
            static_cast<float>(global_map_leaf_size),
            static_cast<float>(global_map_leaf_size));
        voxel_filter.setInputCloud(rebuilt);
        pcl::PointCloud<SCPointType> filtered_cloud;
        voxel_filter.filter(filtered_cloud);
        rebuilt->swap(filtered_cloud);
    }

    sensor_msgs::PointCloud2 optimized_msg;
    {
        std::lock_guard<std::mutex> lock(optimized_global_map_mutex);
        optimized_global_map_cloud = rebuilt;
        optimized_msg = pointCloudToRosMessage(
            optimized_global_map_cloud,
            "map",
            ros::Time::now());
    }

    publishDirectCloudMessage(
        nh,
        data_listener2::buildOptimizedGlobalTopicName(optimized_global_map_topic_name),
        optimized_msg);
}

void updateOptimizedOffsetsFromLoop(const geometry_msgs::TransformStamped& loop_transform) {
    const std::string& parent_id = loop_transform.header.frame_id;
    const std::string& child_id = loop_transform.child_frame_id;

    geometry_msgs::Pose2D relative_pose;
    relative_pose.x = loop_transform.transform.translation.x;
    relative_pose.y = loop_transform.transform.translation.y;

    tf2::Quaternion q(
        loop_transform.transform.rotation.x,
        loop_transform.transform.rotation.y,
        loop_transform.transform.rotation.z,
        loop_transform.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    relative_pose.theta = yaw;

    std::lock_guard<std::mutex> offset_lock(optimized_vehicle_offsets_mutex);
    const auto parent = optimized_vehicle_offsets.find(parent_id);
    if (parent == optimized_vehicle_offsets.end()) {
        optimized_vehicle_offsets[parent_id] = geometry_msgs::Pose2D();
    }

    const geometry_msgs::Pose2D& parent_pose = optimized_vehicle_offsets[parent_id];
    geometry_msgs::Pose2D child_pose;
    child_pose.x =
        parent_pose.x + relative_pose.x * std::cos(parent_pose.theta) -
        relative_pose.y * std::sin(parent_pose.theta);
    child_pose.y =
        parent_pose.y + relative_pose.x * std::sin(parent_pose.theta) +
        relative_pose.y * std::cos(parent_pose.theta);
    child_pose.theta = parent_pose.theta + relative_pose.theta;
    optimized_vehicle_offsets[child_id] = child_pose;
}

bool shouldIncludeInFusion(const std::string& lidar_id) {
    return fused_lidar_ids.empty() || fused_lidar_ids.count(lidar_id) > 0;
}

bool createDirectoryIfMissing(const std::string& path, std::string* error_message) {
    if (path.empty()) {
        if (error_message != nullptr) {
            *error_message = "Export directory is empty";
        }
        return false;
    }

    if (path == "/") {
        return true;
    }

    std::string normalized = path;
    while (normalized.size() > 1U && normalized.back() == '/') {
        normalized.pop_back();
    }

    std::string current = normalized.front() == '/' ? "/" : "";
    std::stringstream stream(normalized);
    std::string part;
    while (std::getline(stream, part, '/')) {
        if (part.empty()) {
            continue;
        }
        if (!current.empty() && current.back() != '/') {
            current += "/";
        }
        current += part;

        struct stat info;
        if (stat(current.c_str(), &info) == 0) {
            if (!S_ISDIR(info.st_mode)) {
                if (error_message != nullptr) {
                    *error_message = current + " exists but is not a directory";
                }
                return false;
            }
            continue;
        }

        if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
            if (error_message != nullptr) {
                *error_message =
                    "Failed to create directory " + current + ": " + std::strerror(errno);
            }
            return false;
        }
    }

    return true;
}

std::string buildDefaultGlobalMapExportPath() {
    const std::time_t now = std::time(nullptr);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now);
#else
    localtime_r(&now, &local_tm);
#endif

    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &local_tm);

    std::string path = global_map_export_dir;
    if (!path.empty() && path.back() != '/') {
        path += "/";
    }
    path += global_map_export_prefix + "_" + timestamp + ".pcd";
    return path;
}

std::string normalizeExportPath(const std::string& requested_path) {
    if (requested_path.empty()) {
        return buildDefaultGlobalMapExportPath();
    }

    if (requested_path.back() == '/') {
        std::string path = requested_path;
        path += global_map_export_prefix + ".pcd";
        return path;
    }

    if (requested_path.size() >= 4U &&
        requested_path.substr(requested_path.size() - 4U) == ".pcd") {
        return requested_path;
    }

    return requested_path + ".pcd";
}

bool saveGlobalMapSnapshot(
    const std::string& requested_path,
    std::string* saved_path,
    size_t* point_count,
    std::string* error_message) {
    pcl::PointCloud<SCPointType>::Ptr snapshot(new pcl::PointCloud<SCPointType>());
    {
        std::lock_guard<std::mutex> lock(global_map_mutex);
        if (global_map_cloud->empty()) {
            if (error_message != nullptr) {
                *error_message = "Global map is empty";
            }
            return false;
        }
        *snapshot = *global_map_cloud;
    }

    const std::string resolved_path = normalizeExportPath(requested_path);
    const size_t last_separator = resolved_path.find_last_of('/');
    const std::string directory =
        last_separator == std::string::npos ? "." : resolved_path.substr(0, last_separator);
    if (!createDirectoryIfMissing(directory, error_message)) {
        return false;
    }

    if (pcl::io::savePCDFileBinary(resolved_path, *snapshot) != 0) {
        if (error_message != nullptr) {
            *error_message = "Failed to write PCD file: " + resolved_path;
        }
        return false;
    }

    if (saved_path != nullptr) {
        *saved_path = resolved_path;
    }
    if (point_count != nullptr) {
        *point_count = snapshot->size();
    }
    return true;
}

sensor_msgs::PointCloud2 buildFusedCloudForVehicle(const VehicleData& vehicle) {
    std::unordered_map<std::string, sensor_msgs::PointCloud2> clouds;
    for (const auto& entry : vehicle.lidar_streams) {
        if (!shouldIncludeInFusion(entry.first)) {
            continue;
        }
        clouds.emplace(entry.first, entry.second.latest_cloud);
    }

    return data_listener2::fuseCloudsToVehicleFrame(
        vehicle.vehicle_id,
        clouds,
        fused_lidar_extrinsics);
}

// 工具函数：Float64MultiArray转Eigen矩阵
Eigen::MatrixXd float64MultiArrayToEigen(const std_msgs::Float64MultiArray& arr) {
    if (arr.layout.dim.size() < 2) {
        ROS_ERROR("Invalid Float64MultiArray dimension");
        return Eigen::MatrixXd();
    }
    int rows = arr.layout.dim[0].size;
    int cols = arr.layout.dim[1].size;
    Eigen::MatrixXd mat(rows, cols);
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat(i, j) = arr.data[i * cols + j];
        }
    }
    return mat;
}

// 回环检测工作线程
void loopDetectionWorker() {
    while (ros::ok()) {
        std::this_thread::sleep_for(1s);  // 每秒检测一次
        
        std::lock_guard<std::mutex> lock(vehicle_map_mutex);
        if (vehicle_map.size() < 2) continue;  // 至少需要两辆车
        
        // 遍历所有车辆对进行回环检测
        for (auto& entry1 : vehicle_map) {
            auto& id1 = entry1.first;
            auto& data1 = entry1.second;
            for (auto& entry2 : vehicle_map) {
                auto& id2 = entry2.first;
                auto& data2 = entry2.second;
                if (id1 >= id2) continue;  // 避免重复检测
                
                std::lock_guard<std::mutex> lock1(data1.data_mutex);
                std::lock_guard<std::mutex> lock2(data2.data_mutex);
                
                // 检查数据有效性
                if (!data1.gps_ready || !data2.gps_ready) continue;

                const LidarStreamData* stream1 = selectPrimaryLidarStream(data1);
                const LidarStreamData* stream2 = selectPrimaryLidarStream(data2);
                if (stream1 == nullptr || stream2 == nullptr) continue;
                
                // 取主路或首个可用路的最新ScanContext进行匹配
                Eigen::MatrixXd sc1 = stream1->latest_sc;
                Eigen::MatrixXd sc2 = stream2->latest_sc;
                
                // 计算ScanContext距离
                double sc_dist = sc_manager.distDirectSC(sc1, sc2);
                const double SC_DIST_THRESHOLD = 0.5;  // 距离阈值，可调整
                
                if (sc_dist < SC_DIST_THRESHOLD) {
                    ROS_INFO("Loop detected between %s and %s! Distance: %.4f",
                             id1.c_str(), id2.c_str(), sc_dist);
                    
                    // 发布回环检测结果（可根据需要定义专门的消息类型）
                    geometry_msgs::TransformStamped loop_transform;
                    loop_transform.header.stamp = ros::Time::now();
                    loop_transform.header.frame_id = id1;
                    loop_transform.child_frame_id = id2;
                    
                    // 计算相对位姿（这里简化处理，实际应根据匹配结果优化）
                    loop_transform.transform.translation.x = 
                        data2.gps_pose.x - data1.gps_pose.x;
                    loop_transform.transform.translation.y = 
                        data2.gps_pose.y - data1.gps_pose.y;
                    loop_transform.transform.translation.z = 0.0;
                    
                    tf2::Quaternion q;
                    q.setRPY(0, 0, data2.gps_pose.theta - data1.gps_pose.theta);
                    loop_transform.transform.rotation.x = q.x();
                    loop_transform.transform.rotation.y = q.y();
                    loop_transform.transform.rotation.z = q.z();
                    loop_transform.transform.rotation.w = q.w();
                    
                    loop_detection_pub.publish(loop_transform);
                    updateOptimizedOffsetsFromLoop(loop_transform);

                    // 这里可以发布tf或自定义消息
                    static tf2_ros::TransformBroadcaster br;
                    br.sendTransform(loop_transform);
                }
            }
        }
    }
}

// 信号处理函数
void signalHandler(int signum) {
    ROS_INFO("Shutdown signal received (%d). Exiting...", signum);
    ros::shutdown();
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "cloud_loop_detector");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 参数配置
    std::string server_host;
    std::string legacy_server_host;
    std::string pcl_pub_name;
    std::string fused_lidar_ids_config;
    std::string fused_lidar_extrinsics_config;
    std::string global_map_export_dir_param;
    std::string global_map_export_prefix_param;
    double global_map_leaf_size_param;
    double global_frame_leaf_size_param;
    int global_map_max_points_param;
    int server_port;
    private_nh.param<std::string>("server_host", server_host, "");
    private_nh.param<std::string>("cloud_server_host", legacy_server_host, "0.0.0.0");
    if (server_host.empty()) {
        server_host = legacy_server_host;
    }
    if (!private_nh.getParam("server_port", server_port)) {
        private_nh.param<int>("cloud_server_port", server_port, 5000);
    }
    private_nh.param<std::string>("pcl_pub_name", pcl_pub_name, "cloud_registered");
    private_nh.param<std::string>("fused_cloud_topic_name", fused_cloud_topic_name, "fused");
    private_nh.param<std::string>("global_cloud_topic_name", global_cloud_topic_name, "global_cloud");
    private_nh.param<std::string>("global_map_topic_name", global_map_topic_name, "global_map");
    private_nh.param<std::string>(
        "optimized_global_map_topic_name",
        optimized_global_map_topic_name,
        "global_map_optimized");
    private_nh.param<std::string>("global_map_export_dir", global_map_export_dir_param, "/tmp/perception_maps");
    private_nh.param<std::string>("global_map_export_prefix", global_map_export_prefix_param, "global_map");
    private_nh.param<std::string>("fused_lidar_ids", fused_lidar_ids_config, "forward,left,right");
    private_nh.param<std::string>(
        "fused_lidar_extrinsics",
        fused_lidar_extrinsics_config,
        "forward:0.0,0.0,0.0,0;"
        "left:-0.8,-0.6,0.0,90;"
        "right:0.8,-0.6,0.0,-90");
    private_nh.param<double>("global_map_leaf_size", global_map_leaf_size_param, 0.15);
    private_nh.param<double>("global_frame_leaf_size", global_frame_leaf_size_param, 0.1);
    private_nh.param<int>("global_map_max_points", global_map_max_points_param, 250000);

    for (std::stringstream stream(fused_lidar_ids_config); stream.good();) {
        std::string item;
        std::getline(stream, item, ',');
        if (!item.empty()) {
            fused_lidar_ids.insert(item);
        }
    }
    fused_lidar_extrinsics =
        data_listener2::parseLidarExtrinsicsConfig(fused_lidar_extrinsics_config);
    global_map_export_dir = global_map_export_dir_param;
    global_map_export_prefix = global_map_export_prefix_param;
    global_map_leaf_size = global_map_leaf_size_param;
    global_frame_leaf_size = global_frame_leaf_size_param;
    global_map_max_points = static_cast<size_t>(std::max(global_map_max_points_param, 0));
    
    // 初始化发布者
    loop_detection_pub = nh.advertise<geometry_msgs::TransformStamped>("/loop_detections", 10);
    base_cloud_topic = pcl_pub_name;

    // 启动回环检测线程
    std::thread detection_thread(loopDetectionWorker);

    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // HTTP服务器
    httplib::Server svr;

    // 处理GPS同步请求
    svr.Post("/sync_gps", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            std::istringstream stream(req.body);
            ptree json_data;
            boost::property_tree::read_json(stream, json_data);

            std::string vehicle_id = json_data.get<std::string>("vehicle_id");
            std::lock_guard<std::mutex> lock(vehicle_map_mutex);
            
            // 更新车辆GPS信息
            VehicleData& v_data = vehicle_map[vehicle_id];
            std::lock_guard<std::mutex> data_lock(v_data.data_mutex);
            
            v_data.vehicle_id = vehicle_id;
            v_data.gps_pose.x = json_data.get<double>("x");
            v_data.gps_pose.y = json_data.get<double>("y");
            v_data.gps_pose.theta = json_data.get<double>("theta");
            v_data.gps_ready = true;
            
            ROS_INFO("Updated GPS for %s: (%.2f, %.2f, %.2f)",
                     vehicle_id.c_str(), v_data.gps_pose.x, 
                     v_data.gps_pose.y, v_data.gps_pose.theta);

            res.status = 200;
            res.set_content("GPS synced", "text/plain");
        } catch (const boost::property_tree::ptree_error& e) {
            res.status = 400;
            res.set_content("Missing GPS fields: " + std::string(e.what()), "text/plain");
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("JSON parse error: " + std::string(e.what()), "text/plain");
        }
    });

    // 处理点云及特征数据
    svr.Post("/receive", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            // 反序列化消息
            data_listener2::multi_lidar cloud_data;
            ros::serialization::IStream input_stream(
                reinterpret_cast<uint8_t*>(const_cast<char*>(req.body.data())),
                req.body.size());
            ros::serialization::deserialize(input_stream, cloud_data);

            std::string vehicle_id = cloud_data.vehicle_id;
            std::lock_guard<std::mutex> lock(vehicle_map_mutex);
            
            // 更新车辆数据
            VehicleData& v_data = vehicle_map[vehicle_id];
            std::lock_guard<std::mutex> data_lock(v_data.data_mutex);
            
            // 转换特征数据
            v_data.vehicle_id = vehicle_id;
            LidarStreamData& lidar_stream = v_data.lidar_streams["legacy"];
            lidar_stream.lidar_id = "legacy";
            lidar_stream.cloud_topic = "";
            lidar_stream.latest_cloud = cloud_data.cloud;
            lidar_stream.latest_sc = float64MultiArrayToEigen(cloud_data.sc);
            lidar_stream.latest_ring_key = float64MultiArrayToEigen(cloud_data.ring_key);
            lidar_stream.latest_sector_key = float64MultiArrayToEigen(cloud_data.sector_key);

            sensor_msgs::PointCloud2 published_cloud = lidar_stream.latest_cloud;
            published_cloud.header.stamp = ros::Time::now();
            published_cloud.header.frame_id = vehicle_id;
            const std::string legacy_topic =
                data_listener2::buildCloudTopicName(base_cloud_topic, "", "");
            getOrCreateCloudPublisher(nh, legacy_topic).publish(published_cloud);
            
            // 保存历史数据（限制最大历史数量）
            const int MAX_HISTORY = 100;
            lidar_stream.sc_history.push_back(lidar_stream.latest_sc);
            lidar_stream.pose_history.push_back(v_data.gps_pose);
            if (lidar_stream.sc_history.size() > MAX_HISTORY) {
                lidar_stream.sc_history.erase(lidar_stream.sc_history.begin());
                lidar_stream.pose_history.erase(lidar_stream.pose_history.begin());
            }

            ROS_INFO("Received legacy data from %s (history size: %zu)",
                     vehicle_id.c_str(), lidar_stream.sc_history.size());

            res.status = 200;
            res.set_content("Data received", "text/plain");
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("Data processing error: " + std::string(e.what()), "text/plain");
        }
    });

    svr.Post("/receive_ros2", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            const data_listener2::Ros2CloudPayload payload =
                data_listener2::parseRos2CloudPayload(req.body);
            const std::string vehicle_id = payload.vehicle_id;
            const std::string lidar_id = payload.lidar_id;

            std::lock_guard<std::mutex> lock(vehicle_map_mutex);
            VehicleData& v_data = vehicle_map[vehicle_id];
            std::lock_guard<std::mutex> data_lock(v_data.data_mutex);

            v_data.vehicle_id = vehicle_id;
            LidarStreamData& stream = v_data.lidar_streams[lidar_id];
            stream.lidar_id = lidar_id;
            stream.cloud_topic = payload.cloud_topic;
            stream.latest_cloud = payload.cloud;
            const data_listener2::ScanContextFeatures features =
                data_listener2::buildScanContextFeatures(stream.latest_cloud, sc_manager);
            stream.latest_sc = features.sc;
            stream.latest_ring_key = features.ring_key;
            stream.latest_sector_key = features.sector_key;

            sensor_msgs::PointCloud2 published_cloud = stream.latest_cloud;
            published_cloud.header.stamp = ros::Time::now();
            published_cloud.header.frame_id = vehicle_id + "/" + lidar_id;
            publishCloudMessage(nh, vehicle_id, lidar_id, published_cloud, true);

            const sensor_msgs::PointCloud2 fused_cloud = buildFusedCloudForVehicle(v_data);
            if (!fused_cloud.data.empty()) {
                publishCloudMessage(nh, vehicle_id, fused_cloud_topic_name, fused_cloud, true);
            }

            const int MAX_HISTORY = 100;
            stream.sc_history.push_back(stream.latest_sc);
            stream.pose_history.push_back(v_data.gps_pose);
            if (stream.sc_history.size() > MAX_HISTORY) {
                stream.sc_history.erase(stream.sc_history.begin());
                stream.pose_history.erase(stream.pose_history.begin());
            }

            ROS_INFO("Received ROS2 cloud data from %s/%s topic=%s (%u x %u, history size: %zu)",
                     vehicle_id.c_str(),
                     lidar_id.c_str(),
                     stream.cloud_topic.c_str(),
                     published_cloud.height,
                     published_cloud.width,
                     stream.sc_history.size());

            res.status = 200;
            res.set_content("ROS2 cloud data received", "text/plain");
        } catch (const std::runtime_error& e) {
            res.status = 400;
            res.set_content("Bad ROS2 payload: " + std::string(e.what()), "text/plain");
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("ROS2 data processing error: " + std::string(e.what()), "text/plain");
        }
    });

    svr.Post("/receive_fastlio", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            const data_listener2::FastLioPayload payload =
                data_listener2::parseFastLioPayload(req.body);

            sensor_msgs::PointCloud2 transformed_cloud =
                data_listener2::transformCloudToGlobalFrame(
                    payload.vehicle_id + "/map",
                    payload.cloud,
                    payload.odom);
            transformed_cloud = downsampleCloudMessage(transformed_cloud, global_frame_leaf_size);

            sensor_msgs::PointCloud2 vehicle_global_msg;
            sensor_msgs::PointCloud2 global_map_msg;
            size_t vehicle_count = 1U;

            {
                std::lock_guard<std::mutex> lock(vehicle_map_mutex);
                vehicle_count = std::max<size_t>(1U, vehicle_map.size());
                VehicleData& v_data = vehicle_map[payload.vehicle_id];
                std::lock_guard<std::mutex> data_lock(v_data.data_mutex);

                v_data.vehicle_id = payload.vehicle_id;
                v_data.fastlio_cloud_topic = payload.cloud_topic;
                v_data.fastlio_odom_topic = payload.odom_topic;
                v_data.latest_fastlio_odom = payload.odom;
                v_data.latest_registered_cloud = payload.cloud;
                v_data.latest_global_cloud = transformed_cloud;
                v_data.fastlio_ready = true;

                appendCloudToAccumulator(
                    transformed_cloud,
                    v_data.accumulated_global_cloud,
                    global_map_leaf_size,
                    global_map_max_points);
                vehicle_global_msg = pointCloudToRosMessage(
                    v_data.accumulated_global_cloud,
                    "map",
                    ros::Time::now());
            }

            {
                std::lock_guard<std::mutex> lock(global_map_mutex);
                appendCloudToAccumulator(
                    transformed_cloud,
                    global_map_cloud,
                    global_map_leaf_size,
                    global_map_max_points * vehicle_count);
                global_map_msg = pointCloudToRosMessage(
                    global_map_cloud,
                    "map",
                    ros::Time::now());
            }

            publishDirectCloudMessage(
                nh,
                "/" + payload.vehicle_id + "/" + global_cloud_topic_name,
                vehicle_global_msg);
            publishDirectCloudMessage(nh, "/" + global_map_topic_name, global_map_msg);
            rebuildOptimizedGlobalMap(nh);

            ROS_INFO(
                "Received FAST-LIO data from %s topic=%s odom_topic=%s (vehicle_global_points=%u, global_map_points=%u)",
                payload.vehicle_id.c_str(),
                payload.cloud_topic.c_str(),
                payload.odom_topic.c_str(),
                vehicle_global_msg.width * vehicle_global_msg.height,
                global_map_msg.width * global_map_msg.height);

            res.status = 200;
            res.set_content("FAST-LIO data received", "text/plain");
        } catch (const std::runtime_error& e) {
            res.status = 400;
            res.set_content("Bad FAST-LIO payload: " + std::string(e.what()), "text/plain");
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("FAST-LIO data processing error: " + std::string(e.what()), "text/plain");
        }
    });

    auto handle_save_global_map = [&](const httplib::Request& req, httplib::Response& res) {
        try {
            std::string requested_path;
            if (req.has_param("path")) {
                requested_path = req.get_param_value("path");
            } else if (!req.body.empty()) {
                try {
                    std::istringstream stream(req.body);
                    ptree json_data;
                    boost::property_tree::read_json(stream, json_data);
                    requested_path = json_data.get<std::string>("path", "");
                } catch (const std::exception&) {
                    requested_path = req.body;
                }
            }

            std::string saved_path;
            std::string error_message;
            size_t point_count = 0U;
            if (!saveGlobalMapSnapshot(requested_path, &saved_path, &point_count, &error_message)) {
                res.status = 409;
                res.set_content(error_message, "text/plain");
                return;
            }

            ptree response;
            response.put("saved_path", saved_path);
            response.put("point_count", static_cast<unsigned long long>(point_count));
            response.put("frame_id", "map");
            std::ostringstream response_stream;
            boost::property_tree::write_json(response_stream, response, false);
            res.status = 200;
            res.set_content(response_stream.str(), "application/json");
            ROS_INFO("Saved /%s snapshot to %s (%zu points)",
                     global_map_topic_name.c_str(),
                     saved_path.c_str(),
                     point_count);
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content("Save global map error: " + std::string(e.what()), "text/plain");
        }
    };
    svr.Get("/save_global_map", handle_save_global_map);
    svr.Post("/save_global_map", handle_save_global_map);

    // 启动服务器
    ROS_INFO("Starting cloud loop detection server on %s:%d", 
             server_host.c_str(), server_port);
    if (!svr.listen(server_host.c_str(), server_port)) {
        ROS_FATAL("Failed to start server");
        return 1;
    }

    ros::spin();
    detection_thread.join();  // 等待工作线程结束
    return 0;
}
