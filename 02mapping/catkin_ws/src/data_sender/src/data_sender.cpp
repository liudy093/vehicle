// #include <ros/ros.h>
// #include <ros/serialization.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <nlohmann/json.hpp>
// #include <geometry_msgs/Pose2D.h>

// #include <vector>
// #include <memory>

// #include <signal.h>
// #include <unistd.h>

// // 引用data_listener包中的lidar.msg
// #include "data_listener2/multi_lidar.h"
// #include "data_listener2/update_gps.h"
// #include "data_listener2/sync_gps.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>

// #include <httplib.h>


// std::string vehicle_id;

// // GPS status
// bool GPS_ready = false;
// bool GPS_synced = false;
// geometry_msgs::Pose2D current_GPS;

// // http client
// std::string cloud_server_url;
// std::unique_ptr<httplib::Client> cli;

// unsigned long pointcloud_count = 0;

// void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
//     // 处理接收到的点云数据
//     if (pointcloud_count % 10 == 0) {
        
//         if (!GPS_ready) {
//             ROS_WARN("GPS data not ready yet, waiting...");
//             return;
//         }
//         if (!GPS_synced) {
//             ROS_WARN("GPS data not synced yet, waiting...");
//             return;
//         }
//         if (vehicle_id == "unknown") {
//             ROS_INFO("vehicle_id unknown, skipping");
//             return;
//         }

//         // fillup and serialization
//         data_listener2::multi_lidar lidar_msg;
//         lidar_msg.vehicle_id = vehicle_id;
//         lidar_msg.cloud = *cloud_msg;

//         std::vector<uint8_t> buffer;
//         uint32_t buffer_size = ros::serialization::serializationLength(lidar_msg);
//         buffer.resize(buffer_size);
//         ros::serialization::OStream stream(buffer.data(), buffer_size);
//         ros::serialization::serialize(stream, lidar_msg);

//         auto res = cli->Post(
//             "/receive",  // 云端接收接口路由
//             reinterpret_cast<const char*>(buffer.data()),
//             buffer.size(),
//             "application/octet-stream");

//         if (!res) {
//             ROS_ERROR("Failed to connect to cloud server for point cloud data");
//         } else if (res->status != 200) {
//             ROS_ERROR("Failed to send point cloud data, server responded with status %d: %s",
//                       res->status, res->body.c_str());
//         } else {
//             ROS_INFO("Point cloud data sent successfully");
//         }
//     }
//     pointcloud_count += 1;
// }

// // Point cloud downsampling function
// sensor_msgs::PointCloud2 downsamplePointCloud(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*input_cloud, *pcl_cloud);
    
//     size_t original_size = pcl_cloud->points.size();
    
//     pcl::VoxelGrid<pcl::PointXYZ> sor;
//     sor.setInputCloud(pcl_cloud);
//     double leaf_size = 0.2;  // Voxel size: 3cm
//     sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//     sor.filter(*cloud_filtered);
    
//     size_t filtered_size = cloud_filtered->points.size();
//     double reduction_ratio = static_cast<double>(filtered_size) / original_size;
//     ROS_INFO("[Downsampling] Original points: %zu, Filtered: %zu, Reduction: %.2f%%",
//              original_size, filtered_size, reduction_ratio * 100);
    
//     sensor_msgs::PointCloud2 output_cloud;
//     pcl::toROSMsg(*cloud_filtered, output_cloud);
//     output_cloud.header = input_cloud->header;
    
//     return output_cloud;
// }

// bool UpdateGPSCallback(data_listener2::update_gps::Request &req,
//                                  data_listener2::update_gps::Response &res) {
//     // 更新GPS数据
//     GPS_ready = false;
//     sleep(2);
//     GPS_ready = true;
//     ROS_INFO("GPS updated with x=%.5f, y=%.5f, theta=%.5f",
//              current_GPS.x, current_GPS.y, current_GPS.theta);
//     return true;
// }

// bool SyncGPSCallback(data_listener2::sync_gps::Request &req,
//                                  data_listener2::sync_gps::Response &res) {
//     // sync GPS to the cloud
//     if (GPS_ready == false) {
//         ROS_WARN("GPS data not ready yet, waiting...");
//         return false;
//     }

//     nlohmann::json json_data;
//     json_data["x"] = current_GPS.x;
//     json_data["y"] = current_GPS.y;
//     json_data["theta"] = current_GPS.theta;
//     json_data["vehicle_id"] = vehicle_id;
//     std::string s = json_data.dump();
//     auto http_res = cli->Post("/sync_gps", s, "application/json");
//     if (http_res) {
//         if (http_res->status == 200) {
//             ROS_INFO("GPS synced successfully");
//             GPS_synced = true;
//             return true;
//         } else {
//             ROS_ERROR("Failed to sync GPS, server responded with status %d: %s",
//                       http_res->status, http_res->body.c_str());
//         }
//     } else {
//         ROS_ERROR("Failed to connect to cloud server for GPS sync");
//     }
//     return false;
// }

// void gpsCallback(const geometry_msgs::Pose2D::ConstPtr& gps_msg) {
//     // 更新当前GPS数据
//     if (GPS_ready == false) {
//         current_GPS = *gps_msg;    
//     }
// }

// // 信号处理（优雅退出）
// void signalHandler(int signum) {
//     ROS_INFO("Shutdown signal received (%d). Exiting...", signum);
//     ros::shutdown();
// }


// int main(int argc, char**argv) {
//     ros::init(argc, argv, "lidar_data_sender");
//     ros::NodeHandle nh;

//     nh.param<std::string>("vehicle_id", vehicle_id, "unknown");
//     nh.param<std::string>("cloud_server_url", cloud_server_url, "http://127.0.0.1:5000");

//     std::cout << "Vehicle ID: " << vehicle_id << std::endl;
//     std::cout << "Cloud server URL: " << cloud_server_url << std::endl;

//     // 注册退出信号处理
//     signal(SIGINT, signalHandler);
//     signal(SIGTERM, signalHandler);

//     cli = std::make_unique<httplib::Client>(cloud_server_url);

//     ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 10, pointCloudCallback);
//     ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::Pose2D>("/gps", 10, gpsCallback);

//     ros::ServiceServer updateGPSservice = nh.advertiseService("update_gps", UpdateGPSCallback);
//     ros::ServiceServer syncGPSservice = nh.advertiseService("sync_gps", SyncGPSCallback);


//     ros::spin();
//     return 0;
// }




// #include <ros/ros.h>
// #include <ros/serialization.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <nlohmann/json.hpp>
// #include <geometry_msgs/Pose2D.h>

// #include <vector>
// #include <memory>
// #include <Eigen/Dense>

// #include <signal.h>
// #include <unistd.h>

// // 引用自定义消息（使用新的multi.msg）
// #include "data_listener2/multi_lidar.h"
// #include "data_listener2/update_gps.h"
// #include "data_listener2/sync_gps.h"

// // ScanContext相关头文件（根据实际路径调整）
// #include "Scancontext.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>

// #include <httplib.h>

// // 类型定义
// using SCPointType = pcl::PointXYZI;  // 适配ScanContext的点类型
// using json = nlohmann::json;

// // 全局变量
// std::string vehicle_id;
// bool GPS_ready = false;
// bool GPS_synced = false;
// geometry_msgs::Pose2D current_GPS;
// std::string cloud_server_url;
// std::unique_ptr<httplib::Client> cli;
// unsigned long pointcloud_count = 0;
// SCManager sc_manager;  // ScanContext管理器

// // 工具函数：Eigen矩阵转Float64MultiArray
// std_msgs::Float64MultiArray eigenToFloat64MultiArray(const Eigen::MatrixXd& mat) {
//     std_msgs::Float64MultiArray arr;
//     arr.layout.dim.resize(2);
//     arr.layout.dim[0].size = mat.rows();
//     arr.layout.dim[0].stride = mat.rows() * mat.cols();
//     arr.layout.dim[0].label = "rows";
//     arr.layout.dim[1].size = mat.cols();
//     arr.layout.dim[1].stride = mat.cols();
//     arr.layout.dim[1].label = "cols";
//     arr.data.resize(mat.rows() * mat.cols());
    
//     for (int i = 0; i < mat.rows(); ++i) {
//         for (int j = 0; j < mat.cols(); ++j) {
//             arr.data[i * mat.cols() + j] = mat(i, j);
//         }
//     }
//     return arr;
// }

// // 点云下采样函数
// sensor_msgs::PointCloud2 downsamplePointCloud(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*input_cloud, *pcl_cloud);
    
//     size_t original_size = pcl_cloud->points.size();
    
//     pcl::VoxelGrid<pcl::PointXYZ> sor;
//     sor.setInputCloud(pcl_cloud);
//     double leaf_size = 0.2;  // 体素大小
//     sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//     sor.filter(*cloud_filtered);
    
//     size_t filtered_size = cloud_filtered->points.size();
//     double reduction_ratio = static_cast<double>(filtered_size) / original_size;
//     ROS_INFO("[Downsampling] Original: %zu, Filtered: %zu, Ratio: %.2f%%",
//              original_size, filtered_size, reduction_ratio * 100);
    
//     sensor_msgs::PointCloud2 output_cloud;
//     pcl::toROSMsg(*cloud_filtered, output_cloud);
//     output_cloud.header = input_cloud->header;
    
//     return output_cloud;
// }

// // 点云回调函数
// void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
//     if (pointcloud_count % 10 == 0) {  // 每10帧处理一次
//         // 状态检查
//         if (!GPS_ready) {
//             ROS_WARN("GPS data not ready yet, waiting...");
//             return;
//         }
//         if (!GPS_synced) {
//             ROS_WARN("GPS data not synced yet, waiting...");
//             return;
//         }
//         if (vehicle_id == "unknown") {
//             ROS_INFO("vehicle_id unknown, skipping");
//             return;
//         }

//         // 点云下采样
//         sensor_msgs::PointCloud2 filtered_cloud = downsamplePointCloud(cloud_msg);

//         // 转换为XYZI点云（适配ScanContext）
//         pcl::PointCloud<SCPointType> cloud_xyzi;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(filtered_cloud, *pcl_cloud);
        
//         for (const auto& p : pcl_cloud->points) {
//             SCPointType p_xyzi;
//             p_xyzi.x = p.x;
//             p_xyzi.y = p.y;
//             p_xyzi.z = p.z;
//             p_xyzi.intensity = 0.0;  // 若有强度信息可在此赋值
//             cloud_xyzi.push_back(p_xyzi);
//         }

//         // 生成ScanContext及特征
//         Eigen::MatrixXd sc = sc_manager.makeScancontext(cloud_xyzi);
//         Eigen::MatrixXd ring_key = sc_manager.makeRingkeyFromScancontext(sc);
//         Eigen::MatrixXd sector_key = sc_manager.makeSectorkeyFromScancontext(sc);

//         // 转换为消息类型
//         std_msgs::Float64MultiArray sc_arr = eigenToFloat64MultiArray(sc);
//         std_msgs::Float64MultiArray ring_key_arr = eigenToFloat64MultiArray(ring_key);
//         std_msgs::Float64MultiArray sector_key_arr = eigenToFloat64MultiArray(sector_key);

//         // 填充消息
//         data_listener2::multi_lidar lidar_msg;
//         lidar_msg.vehicle_id = vehicle_id;
//         lidar_msg.cloud = filtered_cloud;
//         lidar_msg.sc = sc_arr;
//         lidar_msg.ring_key = ring_key_arr;
//         lidar_msg.sector_key = sector_key_arr;

//         // 序列化并发送
//         std::vector<uint8_t> buffer;
//         uint32_t buffer_size = ros::serialization::serializationLength(lidar_msg);
//         buffer.resize(buffer_size);
//         ros::serialization::OStream stream(buffer.data(), buffer_size);
//         ros::serialization::serialize(stream, lidar_msg);

//         auto res = cli->Post(
//             "/receive",
//             reinterpret_cast<const char*>(buffer.data()),
//             buffer.size(),
//             "application/octet-stream");

//         // 发送结果处理
//         if (!res) {
//             ROS_ERROR("Failed to connect to cloud server");
//         } else if (res->status != 200) {
//             ROS_ERROR("Server responded with status %d: %s", res->status, res->body.c_str());
//         } else {
//             ROS_INFO("Data sent successfully (frame: %lu)", pointcloud_count);
//         }
//     }
//     pointcloud_count++;
// }

// // GPS更新服务回调
// bool UpdateGPSCallback(data_listener2::update_gps::Request &req,
//                        data_listener2::update_gps::Response &res) {
//     GPS_ready = false;
//     sleep(2);  // 模拟处理延迟
//     GPS_ready = true;
//     ROS_INFO("GPS updated with x=%.5f, y=%.5f, theta=%.5f",
//              current_GPS.x, current_GPS.y, current_GPS.theta);
//     return true;
// }

// // GPS同步服务回调
// bool SyncGPSCallback(data_listener2::sync_gps::Request &req,
//                      data_listener2::sync_gps::Response &res) {
//     if (!GPS_ready) {
//         ROS_WARN("GPS data not ready yet");
//         return false;
//     }

//     json json_data;
//     json_data["x"] = current_GPS.x;
//     json_data["y"] = current_GPS.y;
//     json_data["theta"] = current_GPS.theta;
//     json_data["vehicle_id"] = vehicle_id;
    
//     auto http_res = cli->Post("/sync_gps", json_data.dump(), "application/json");
//     if (http_res && http_res->status == 200) {
//         ROS_INFO("GPS synced successfully");
//         GPS_synced = true;
//         return true;
//     } else {
//         ROS_ERROR("GPS sync failed (status: %d)", http_res ? http_res->status : -1);
//         return false;
//     }
// }

// // GPS消息回调
// void gpsCallback(const geometry_msgs::Pose2D::ConstPtr& gps_msg) {
//     if (!GPS_ready) {
//         current_GPS = *gps_msg;
//     }
// }

// // 信号处理函数
// void signalHandler(int signum) {
//     ROS_INFO("Shutdown signal received (%d). Exiting...", signum);
//     ros::shutdown();
// }

// int main(int argc, char**argv) {
//     ros::init(argc, argv, "lidar_data_sender");
//     ros::NodeHandle nh;

//     // 参数读取
//     nh.param<std::string>("vehicle_id", vehicle_id, "unknown");
//     nh.param<std::string>("cloud_server_url", cloud_server_url, "http://127.0.0.1:5000");

//     ROS_INFO("Vehicle ID: %s", vehicle_id.c_str());
//     ROS_INFO("Cloud server URL: %s", cloud_server_url.c_str());

//     // 信号注册
//     signal(SIGINT, signalHandler);
//     signal(SIGTERM, signalHandler);

//     // HTTP客户端初始化
//     cli = std::make_unique<httplib::Client>(cloud_server_url);

//     // 订阅与服务
//     ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 10, pointCloudCallback);
//     ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::Pose2D>("/gps", 10, gpsCallback);
//     ros::ServiceServer updateGPSservice = nh.advertiseService("update_gps", UpdateGPSCallback);
//     ros::ServiceServer syncGPSservice = nh.advertiseService("sync_gps", SyncGPSCallback);

//     ros::spin();
//     return 0;
// }

// #include <ros/ros.h>
// #include <ros/serialization.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <nlohmann/json.hpp>
// #include <geometry_msgs/Pose2D.h>

// #include <vector>
// #include <memory>
// #include <Eigen/Dense>

// #include <signal.h>
// #include <unistd.h>

// // 引用自定义消息
// #include "data_listener2/multi_lidar.h"
// #include "data_listener2/update_gps.h"
// #include "data_listener2/sync_gps.h"

// // ScanContext相关头文件
// #include "Scancontext.h"

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/filters/voxel_grid.h>

// #include <httplib.h>

// // 类型定义
// using SCPointType = pcl::PointXYZI;
// using json = nlohmann::json;

// // 自适应体素滤波参数
// struct AdaptiveVoxelParams {
//     float near_threshold = 5.0f;    // 近场距离阈值（米）
//     float far_threshold = 15.0f;    // 远场距离阈值（米）
//     float voxel_near = 0.05f;       // 近场体素大小（米）
//     float voxel_mid = 0.2f;         // 中场体素大小（米）
//     float voxel_far = 0.5f;         // 远场体素大小（米）
// } adaptive_params;

// // 全局变量
// std::string vehicle_id;
// bool GPS_ready = false;
// bool GPS_synced = false;
// geometry_msgs::Pose2D current_GPS;
// std::string cloud_server_url;
// std::unique_ptr<httplib::Client> cli;
// unsigned long pointcloud_count = 0;
// SCManager sc_manager;  // ScanContext管理器

// // 工具函数：Eigen矩阵转Float64MultiArray
// std_msgs::Float64MultiArray eigenToFloat64MultiArray(const Eigen::MatrixXd& mat) {
//     std_msgs::Float64MultiArray arr;
//     arr.layout.dim.resize(2);
//     arr.layout.dim[0].size = mat.rows();
//     arr.layout.dim[0].stride = mat.rows() * mat.cols();
//     arr.layout.dim[0].label = "rows";
//     arr.layout.dim[1].size = mat.cols();
//     arr.layout.dim[1].stride = mat.cols();
//     arr.layout.dim[1].label = "cols";
//     arr.data.resize(mat.rows() * mat.cols());
    
//     for (int i = 0; i < mat.rows(); ++i) {
//         for (int j = 0; j < mat.cols(); ++j) {
//             arr.data[i * mat.cols() + j] = mat(i, j);
//         }
//     }
//     return arr;
// }

// /**
//  * @brief 计算点到传感器的3D距离
//  */
// inline float calculateDistance(const pcl::PointXYZ& point) {
//     return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
// }

// /**
//  * @brief 根据距离获取对应的体素大小
//  */
// float getVoxelSizeByDistance(float distance) {
//     if (distance < adaptive_params.near_threshold) {
//         return adaptive_params.voxel_near;
//     } else if (distance < adaptive_params.far_threshold) {
//         // 中场区域线性过渡
//         float ratio = (distance - adaptive_params.near_threshold) / 
//                       (adaptive_params.far_threshold - adaptive_params.near_threshold);
//         return adaptive_params.voxel_near + ratio * (adaptive_params.voxel_mid - adaptive_params.voxel_near);
//     } else {
//         // 远场区域轻微非线性增长
//         return adaptive_params.voxel_mid + 0.01f * (distance - adaptive_params.far_threshold);
//     }
// }

// /**
//  * @brief 执行自适应体素滤波
//  */
// sensor_msgs::PointCloud2 adaptiveVoxelFilter(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {
//     // 转换为PCL点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromROSMsg(*input_cloud, *pcl_cloud);
    
//     if (pcl_cloud->empty()) {
//         ROS_WARN("Input cloud is empty, returning original");
//         return *input_cloud;
//     }

//     size_t original_size = pcl_cloud->points.size();

//     // 分离不同距离区域的点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr mid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//     for (const auto& point : pcl_cloud->points) {
//         float distance = calculateDistance(point);
//         if (distance < adaptive_params.near_threshold) {
//             near_cloud->push_back(point);
//         } else if (distance < adaptive_params.far_threshold) {
//             mid_cloud->push_back(point);
//         } else {
//             far_cloud->push_back(point);
//         }
//     }

//     ROS_INFO("[Adaptive Filter] Near: %zu, Mid: %zu, Far: %zu points",
//              near_cloud->size(), mid_cloud->size(), far_cloud->size());

//     // 对各区域分别进行体素滤波
//     auto filterCloud = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float voxel_size) {
//         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
//         if (cloud->empty()) return filtered;
        
//         pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
//         voxel_filter.setInputCloud(cloud);
//         voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
//         voxel_filter.filter(*filtered);
//         return filtered;
//     };

//     auto filtered_near = filterCloud(near_cloud, adaptive_params.voxel_near);
//     auto filtered_mid = filterCloud(mid_cloud, adaptive_params.voxel_mid);
//     auto filtered_far = filterCloud(far_cloud, adaptive_params.voxel_far);

//     // 合并结果
//     pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     *output_cloud = *filtered_near + *filtered_mid + *filtered_far;

//     // 统计滤波效果
//     size_t filtered_size = output_cloud->size();
//     double reduction_ratio = static_cast<double>(filtered_size) / original_size;
//     ROS_INFO("[Adaptive Filter] Original: %zu, Filtered: %zu, Ratio: %.2f%%",
//              original_size, filtered_size, reduction_ratio * 100);

//     // 转换回ROS消息
//     sensor_msgs::PointCloud2 output_msg;
//     pcl::toROSMsg(*output_cloud, output_msg);
//     output_msg.header = input_cloud->header;
    
//     return output_msg;
// }

// // 点云回调函数
// void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
//     if (pointcloud_count % 10 == 0) {  // 每10帧处理一次
//         // 状态检查
//         if (!GPS_ready) {
//             ROS_WARN("GPS data not ready yet, waiting...");
//             return;
//         }
//         if (!GPS_synced) {
//             ROS_WARN("GPS data not synced yet, waiting...");
//             return;
//         }
//         if (vehicle_id == "unknown") {
//             ROS_INFO("vehicle_id unknown, skipping");
//             return;
//         }

//         // 执行自适应体素滤波
//         sensor_msgs::PointCloud2 filtered_cloud = adaptiveVoxelFilter(cloud_msg);

//         // 转换为XYZI点云（适配ScanContext）
//         pcl::PointCloud<SCPointType> cloud_xyzi;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(filtered_cloud, *pcl_cloud);
        
//         for (const auto& p : pcl_cloud->points) {
//             SCPointType p_xyzi;
//             p_xyzi.x = p.x;
//             p_xyzi.y = p.y;
//             p_xyzi.z = p.z;
//             p_xyzi.intensity = 0.0;  // 若有强度信息可在此赋值
//             cloud_xyzi.push_back(p_xyzi);
//         }

//         // 生成ScanContext及特征
//         Eigen::MatrixXd sc = sc_manager.makeScancontext(cloud_xyzi);
//         Eigen::MatrixXd ring_key = sc_manager.makeRingkeyFromScancontext(sc);
//         Eigen::MatrixXd sector_key = sc_manager.makeSectorkeyFromScancontext(sc);

//         // 转换为消息类型
//         std_msgs::Float64MultiArray sc_arr = eigenToFloat64MultiArray(sc);
//         std_msgs::Float64MultiArray ring_key_arr = eigenToFloat64MultiArray(ring_key);
//         std_msgs::Float64MultiArray sector_key_arr = eigenToFloat64MultiArray(sector_key);

//         // 填充消息
//         data_listener2::multi_lidar lidar_msg;
//         lidar_msg.vehicle_id = vehicle_id;
//         lidar_msg.cloud = filtered_cloud;
//         lidar_msg.sc = sc_arr;
//         lidar_msg.ring_key = ring_key_arr;
//         lidar_msg.sector_key = sector_key_arr;

//         // 序列化并发送
//         std::vector<uint8_t> buffer;
//         uint32_t buffer_size = ros::serialization::serializationLength(lidar_msg);
//         buffer.resize(buffer_size);
//         ros::serialization::OStream stream(buffer.data(), buffer_size);
//         ros::serialization::serialize(stream, lidar_msg);

//         auto res = cli->Post(
//             "/receive",
//             reinterpret_cast<const char*>(buffer.data()),
//             buffer.size(),
//             "application/octet-stream");

//         // 发送结果处理
//         if (!res) {
//             ROS_ERROR("Failed to connect to cloud server");
//         } else if (res->status != 200) {
//             ROS_ERROR("Server responded with status %d: %s", res->status, res->body.c_str());
//         } else {
//             ROS_INFO("Data sent successfully (frame: %lu)", pointcloud_count);
//         }
//     }
//     pointcloud_count++;
// }

// // GPS更新服务回调
// bool UpdateGPSCallback(data_listener2::update_gps::Request &req,
//                        data_listener2::update_gps::Response &res) {
//     GPS_ready = false;
//     sleep(2);  // 模拟处理延迟
//     GPS_ready = true;
//     ROS_INFO("GPS updated with x=%.5f, y=%.5f, theta=%.5f",
//              current_GPS.x, current_GPS.y, current_GPS.theta);
//     return true;
// }

// // GPS同步服务回调
// bool SyncGPSCallback(data_listener2::sync_gps::Request &req,
//                      data_listener2::sync_gps::Response &res) {
//     if (!GPS_ready) {
//         ROS_WARN("GPS data not ready yet");
//         return false;
//     }

//     json json_data;
//     json_data["x"] = current_GPS.x;
//     json_data["y"] = current_GPS.y;
//     json_data["theta"] = current_GPS.theta;
//     json_data["vehicle_id"] = vehicle_id;
    
//     auto http_res = cli->Post("/sync_gps", json_data.dump(), "application/json");
//     if (http_res && http_res->status == 200) {
//         ROS_INFO("GPS synced successfully");
//         GPS_synced = true;
//         return true;
//     } else {
//         ROS_ERROR("GPS sync failed (status: %d)", http_res ? http_res->status : -1);
//         return false;
//     }
// }

// // GPS消息回调
// void gpsCallback(const geometry_msgs::Pose2D::ConstPtr& gps_msg) {
//     if (!GPS_ready) {
//         current_GPS = *gps_msg;
//     }
// }

// // 信号处理函数
// void signalHandler(int signum) {
//     ROS_INFO("Shutdown signal received (%d). Exiting...", signum);
//     ros::shutdown();
// }

// int main(int argc, char**argv) {
//     ros::init(argc, argv, "lidar_data_sender");
//     ros::NodeHandle nh("~");  // 使用私有命名空间以便参数配置

//     // 参数读取
//     nh.param<std::string>("vehicle_id", vehicle_id, "unknown");
//     nh.param<std::string>("cloud_server_url", cloud_server_url, "http://127.0.0.1:5000");
    
//     // 读取自适应滤波参数
//     nh.param<float>("near_threshold", adaptive_params.near_threshold, 5.0f);
//     nh.param<float>("far_threshold", adaptive_params.far_threshold, 15.0f);
//     nh.param<float>("voxel_near", adaptive_params.voxel_near, 0.05f);
//     nh.param<float>("voxel_mid", adaptive_params.voxel_mid, 0.2f);
//     nh.param<float>("voxel_far", adaptive_params.voxel_far, 0.5f);

//     ROS_INFO("Vehicle ID: %s", vehicle_id.c_str());
//     ROS_INFO("Cloud server URL: %s", cloud_server_url.c_str());
//     ROS_INFO("Adaptive Filter Params - Near: %.2fm (≤%.2fm), Mid: %.2fm (%.2f-%.2fm), Far: ≥%.2fm (>%.2fm)",
//              adaptive_params.voxel_near, adaptive_params.near_threshold,
//              adaptive_params.voxel_mid, adaptive_params.near_threshold, adaptive_params.far_threshold,
//              adaptive_params.voxel_far, adaptive_params.far_threshold);

//     // 信号注册
//     signal(SIGINT, signalHandler);
//     signal(SIGTERM, signalHandler);

//     // HTTP客户端初始化
//     cli = std::make_unique<httplib::Client>(cloud_server_url);

//     // 订阅与服务
//     ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 10, pointCloudCallback);
//     ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::Pose2D>("/gps", 10, gpsCallback);
//     ros::ServiceServer updateGPSservice = nh.advertiseService("update_gps", UpdateGPSCallback);
//     ros::ServiceServer syncGPSservice = nh.advertiseService("sync_gps", SyncGPSCallback);

//     ros::spin();
//     return 0;
// }


#include <ros/ros.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>

#include <nlohmann/json.hpp>
#include <geometry_msgs/Pose2D.h>

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include <signal.h>
#include <unistd.h>

// 引用自定义消息
#include "data_listener2/multi_lidar.h"
#include "data_listener2/update_gps.h"
#include "data_listener2/sync_gps.h"

// ScanContext相关头文件
#include "Scancontext.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <httplib.h>
#include <nav_msgs/Odometry.h>

// 类型定义
using SCPointType = pcl::PointXYZI;
using json = nlohmann::json;

// 自适应体素滤波参数
struct AdaptiveVoxelParams {
    float near_threshold = 5.0f;    // 近场距离阈值（米）
    float far_threshold = 15.0f;    // 远场距离阈值（米）
    float voxel_near = 0.05f;       // 近场体素大小（米）
    float voxel_mid = 0.2f;         // 中场体素大小（米）
    float voxel_far = 0.5f;         // 远场体素大小（米）
} adaptive_params;

// 事件触发传输参数
struct EventTriggerParams {
    float min_translation = 0.5f;   // 最小平移触发阈值（米）
    float min_rotation = 5.0f;      // 最小旋转触发阈值（度）
    float max_interval = 2.0f;      // 最大传输间隔（秒）
} trigger_params;

// 全局变量
std::string vehicle_id;
bool GPS_ready = false;
bool GPS_synced = false;
geometry_msgs::Pose2D current_GPS;
geometry_msgs::Pose2D last_sent_GPS;  // 上次发送时的GPS位置
std::string cloud_server_url;
std::string cloud_topic;
std::string odom_topic;
std::unique_ptr<httplib::Client> cli;
unsigned long pointcloud_count = 0;
SCManager sc_manager;  // ScanContext管理器
ros::Time last_sent_time;  // 上次发送时间
bool is_first_send = true;  // 是否首次发送
nav_msgs::Odometry latest_odom;
bool odom_ready = false;

// 工具函数：Eigen矩阵转Float64MultiArray
std_msgs::Float64MultiArray eigenToFloat64MultiArray(const Eigen::MatrixXd& mat) {
    std_msgs::Float64MultiArray arr;
    arr.layout.dim.resize(2);
    arr.layout.dim[0].size = mat.rows();
    arr.layout.dim[0].stride = mat.rows() * mat.cols();
    arr.layout.dim[0].label = "rows";
    arr.layout.dim[1].size = mat.cols();
    arr.layout.dim[1].stride = mat.cols();
    arr.layout.dim[1].label = "cols";
    arr.data.resize(mat.rows() * mat.cols());
    
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            arr.data[i * mat.cols() + j] = mat(i, j);
        }
    }
    return arr;
}

/**
 * @brief 计算点到传感器的3D距离
 */
inline float calculateDistance(const pcl::PointXYZ& point) {
    return sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

/**
 * @brief 根据距离获取对应的体素大小
 */
float getVoxelSizeByDistance(float distance) {
    if (distance < adaptive_params.near_threshold) {
        return adaptive_params.voxel_near;
    } else if (distance < adaptive_params.far_threshold) {
        // 中场区域线性过渡
        float ratio = (distance - adaptive_params.near_threshold) / 
                      (adaptive_params.far_threshold - adaptive_params.near_threshold);
        return adaptive_params.voxel_near + ratio * (adaptive_params.voxel_mid - adaptive_params.voxel_near);
    } else {
        // 远场区域轻微非线性增长
        return adaptive_params.voxel_mid + 0.01f * (distance - adaptive_params.far_threshold);
    }
}

/**
 * @brief 执行自适应体素滤波
 */
sensor_msgs::PointCloud2 adaptiveVoxelFilter(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {
    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);
    
    if (pcl_cloud->empty()) {
        ROS_WARN("Input cloud is empty, returning original");
        return *input_cloud;
    }

    size_t original_size = pcl_cloud->points.size();

    // 分离不同距离区域的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (const auto& point : pcl_cloud->points) {
        float distance = calculateDistance(point);
        if (distance < adaptive_params.near_threshold) {
            near_cloud->push_back(point);
        } else if (distance < adaptive_params.far_threshold) {
            mid_cloud->push_back(point);
        } else {
            far_cloud->push_back(point);
        }
    }

    ROS_DEBUG("[Adaptive Filter] Near: %zu, Mid: %zu, Far: %zu points",
             near_cloud->size(), mid_cloud->size(), far_cloud->size());

    // 对各区域分别进行体素滤波
    auto filterCloud = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float voxel_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (cloud->empty()) return filtered;
        
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxel_filter.filter(*filtered);
        return filtered;
    };

    auto filtered_near = filterCloud(near_cloud, adaptive_params.voxel_near);
    auto filtered_mid = filterCloud(mid_cloud, adaptive_params.voxel_mid);
    auto filtered_far = filterCloud(far_cloud, adaptive_params.voxel_far);

    // 合并结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *output_cloud = *filtered_near + *filtered_mid + *filtered_far;

    // 统计滤波效果
    size_t filtered_size = output_cloud->size();
    double reduction_ratio = static_cast<double>(filtered_size) / original_size;
    ROS_DEBUG("[Adaptive Filter] Original: %zu, Filtered: %zu, Ratio: %.2f%%",
             original_size, filtered_size, reduction_ratio * 100);

    // 转换回ROS消息
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*output_cloud, output_msg);
    output_msg.header = input_cloud->header;
    
    return output_msg;
}

/**
 * @brief 检查是否需要触发传输
 * @return 是否满足传输条件
 */
bool checkTriggerCondition() {
    // 首次发送直接触发
    if (is_first_send) {
        return true;
    }

    if (!odom_topic.empty()) {
        float time_interval = (ros::Time::now() - last_sent_time).toSec();
        return time_interval >= trigger_params.max_interval;
    }

    // 检查GPS状态
    if (!GPS_ready || !GPS_synced) {
        return false;
    }

    // 计算平移距离
    float dx = current_GPS.x - last_sent_GPS.x;
    float dy = current_GPS.y - last_sent_GPS.y;
    float translation = sqrt(dx*dx + dy*dy);

    // 计算旋转角度（处理角度环绕）
    float rotation = fabs(current_GPS.theta - last_sent_GPS.theta);
    rotation = std::min(rotation, 360.0f - rotation);  // 取最小角度差

    // 计算时间间隔
    float time_interval = (ros::Time::now() - last_sent_time).toSec();

    // 满足以下任一条件则触发传输
    bool translation_trigger = (translation >= trigger_params.min_translation);
    bool rotation_trigger = (rotation >= trigger_params.min_rotation);
    bool time_trigger = (time_interval >= trigger_params.max_interval);

    ROS_DEBUG("Trigger check - Translation: %.2fm (thres: %.2fm), Rotation: %.2f° (thres: %.2f°), Time: %.2fs (thres: %.2fs)",
             translation, trigger_params.min_translation,
             rotation, trigger_params.min_rotation,
             time_interval, trigger_params.max_interval);

    return translation_trigger || rotation_trigger || time_trigger;
}

json buildPointFieldJson(const sensor_msgs::PointField& field) {
    json field_json;
    field_json["name"] = field.name;
    field_json["offset"] = field.offset;
    field_json["datatype"] = field.datatype;
    field_json["count"] = field.count;
    return field_json;
}

json buildPointCloudJson(const sensor_msgs::PointCloud2& cloud_msg) {
    json payload;
    payload["header"]["stamp"]["sec"] = cloud_msg.header.stamp.sec;
    payload["header"]["stamp"]["nanosec"] = cloud_msg.header.stamp.nsec;
    payload["header"]["frame_id"] = cloud_msg.header.frame_id;
    payload["shape"]["height"] = cloud_msg.height;
    payload["shape"]["width"] = cloud_msg.width;
    payload["layout"]["is_bigendian"] = cloud_msg.is_bigendian;
    payload["layout"]["point_step"] = cloud_msg.point_step;
    payload["layout"]["row_step"] = cloud_msg.row_step;
    payload["layout"]["is_dense"] = cloud_msg.is_dense;
    payload["fields"] = json::array();
    for (const auto& field : cloud_msg.fields) {
        payload["fields"].push_back(buildPointFieldJson(field));
    }

    const std::string data_bytes(
        reinterpret_cast<const char*>(cloud_msg.data.data()),
        cloud_msg.data.size());
    payload["data_b64"] = httplib::detail::base64_encode(data_bytes);
    return payload;
}

bool sendFastLioFrame(const sensor_msgs::PointCloud2& cloud_msg) {
    if (!odom_ready) {
        ROS_WARN("FAST-LIO odometry not ready yet, waiting...");
        return false;
    }

    json payload = buildPointCloudJson(cloud_msg);
    payload["vehicle_id"] = vehicle_id;
    payload["cloud_topic"] = cloud_topic;
    payload["odom_topic"] = odom_topic;
    payload["odom"]["header"]["stamp"]["sec"] = latest_odom.header.stamp.sec;
    payload["odom"]["header"]["stamp"]["nanosec"] = latest_odom.header.stamp.nsec;
    payload["odom"]["header"]["frame_id"] = latest_odom.header.frame_id;
    payload["odom"]["child_frame_id"] = latest_odom.child_frame_id;
    payload["odom"]["pose"]["position"]["x"] = latest_odom.pose.pose.position.x;
    payload["odom"]["pose"]["position"]["y"] = latest_odom.pose.pose.position.y;
    payload["odom"]["pose"]["position"]["z"] = latest_odom.pose.pose.position.z;
    payload["odom"]["pose"]["orientation"]["x"] = latest_odom.pose.pose.orientation.x;
    payload["odom"]["pose"]["orientation"]["y"] = latest_odom.pose.pose.orientation.y;
    payload["odom"]["pose"]["orientation"]["z"] = latest_odom.pose.pose.orientation.z;
    payload["odom"]["pose"]["orientation"]["w"] = latest_odom.pose.pose.orientation.w;

    auto res = cli->Post("/receive_fastlio", payload.dump(), "application/json");
    if (!res) {
        ROS_ERROR("Failed to connect to cloud server for FAST-LIO payload");
        return false;
    }
    if (res->status != 200) {
        ROS_ERROR("FAST-LIO upload failed with status %d: %s", res->status, res->body.c_str());
        return false;
    }

    ROS_INFO("FAST-LIO frame sent successfully (frame: %lu)", pointcloud_count);
    last_sent_time = ros::Time::now();
    is_first_send = false;
    return true;
}

// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // 检查是否满足传输条件
    if (!checkTriggerCondition()) {
        pointcloud_count++;
        return;
    }

    if (vehicle_id == "unknown") {
        ROS_INFO("vehicle_id unknown, skipping");
        return;
    }

    if (odom_topic.empty()) {
        if (!GPS_ready) {
            ROS_WARN("GPS data not ready yet, waiting...");
            return;
        }
        if (!GPS_synced) {
            ROS_WARN("GPS data not synced yet, waiting...");
            return;
        }
    }

    // 执行自适应体素滤波
    sensor_msgs::PointCloud2 filtered_cloud = adaptiveVoxelFilter(cloud_msg);

    if (!odom_topic.empty()) {
        if (sendFastLioFrame(filtered_cloud)) {
            pointcloud_count++;
        }
        return;
    }

    // 转换为XYZI点云（适配ScanContext）
    pcl::PointCloud<SCPointType> cloud_xyzi;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(filtered_cloud, *pcl_cloud);
    
    for (const auto& p : pcl_cloud->points) {
        SCPointType p_xyzi;
        p_xyzi.x = p.x;
        p_xyzi.y = p.y;
        p_xyzi.z = p.z;
        p_xyzi.intensity = 0.0;  // 若有强度信息可在此赋值
        cloud_xyzi.push_back(p_xyzi);
    }

    // 生成ScanContext及特征
    Eigen::MatrixXd sc = sc_manager.makeScancontext(cloud_xyzi);
    Eigen::MatrixXd ring_key = sc_manager.makeRingkeyFromScancontext(sc);
    Eigen::MatrixXd sector_key = sc_manager.makeSectorkeyFromScancontext(sc);

    // 转换为消息类型
    std_msgs::Float64MultiArray sc_arr = eigenToFloat64MultiArray(sc);
    std_msgs::Float64MultiArray ring_key_arr = eigenToFloat64MultiArray(ring_key);
    std_msgs::Float64MultiArray sector_key_arr = eigenToFloat64MultiArray(sector_key);

    // 填充消息
    data_listener2::multi_lidar lidar_msg;
    lidar_msg.vehicle_id = vehicle_id;
    lidar_msg.cloud = filtered_cloud;
    lidar_msg.sc = sc_arr;
    lidar_msg.ring_key = ring_key_arr;
    lidar_msg.sector_key = sector_key_arr;

    // 序列化并发送
    std::vector<uint8_t> buffer;
    uint32_t buffer_size = ros::serialization::serializationLength(lidar_msg);
    buffer.resize(buffer_size);
    ros::serialization::OStream stream(buffer.data(), buffer_size);
    ros::serialization::serialize(stream, lidar_msg);

    auto res = cli->Post(
        "/receive",
        reinterpret_cast<const char*>(buffer.data()),
        buffer.size(),
        "application/octet-stream");

    // 发送结果处理
    if (!res) {
        ROS_ERROR("Failed to connect to cloud server");
    } else if (res->status != 200) {
        ROS_ERROR("Server responded with status %d: %s", res->status, res->body.c_str());
    } else {
        ROS_INFO("Data sent successfully (frame: %lu)", pointcloud_count);
        // 更新发送状态
        last_sent_GPS = current_GPS;
        last_sent_time = ros::Time::now();
        is_first_send = false;
    }
    
    pointcloud_count++;
}

// GPS更新服务回调
bool UpdateGPSCallback(data_listener2::update_gps::Request &req,
                       data_listener2::update_gps::Response &res) {
    GPS_ready = false;
    sleep(2);  // 模拟处理延迟
    GPS_ready = true;
    ROS_INFO("GPS updated with x=%.5f, y=%.5f, theta=%.5f",
             current_GPS.x, current_GPS.y, current_GPS.theta);
    return true;
}

// GPS同步服务回调
bool SyncGPSCallback(data_listener2::sync_gps::Request &req,
                     data_listener2::sync_gps::Response &res) {
    if (!GPS_ready) {
        ROS_WARN("GPS data not ready yet");
        return false;
    }

    json json_data;
    json_data["x"] = current_GPS.x;
    json_data["y"] = current_GPS.y;
    json_data["theta"] = current_GPS.theta;
    json_data["vehicle_id"] = vehicle_id;
    
    auto http_res = cli->Post("/sync_gps", json_data.dump(), "application/json");
    if (http_res && http_res->status == 200) {
        ROS_INFO("GPS synced successfully");
        GPS_synced = true;
        // 首次同步时初始化上次发送位置
        if (is_first_send) {
            last_sent_GPS = current_GPS;
            last_sent_time = ros::Time::now();
        }
        return true;
    } else {
        ROS_ERROR("GPS sync failed (status: %d)", http_res ? http_res->status : -1);
        return false;
    }
}

// GPS消息回调
void gpsCallback(const geometry_msgs::Pose2D::ConstPtr& gps_msg) {
    if (!GPS_ready) {
        current_GPS = *gps_msg;
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    latest_odom = *odom_msg;
    odom_ready = true;
}

// 信号处理函数
void signalHandler(int signum) {
    ROS_INFO("Shutdown signal received (%d). Exiting...", signum);
    ros::shutdown();
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "lidar_data_sender");
    ros::NodeHandle nh("~");  // 使用私有命名空间以便参数配置

    // 参数读取
    nh.param<std::string>("vehicle_id", vehicle_id, "unknown");
    nh.param<std::string>("cloud_server_url", cloud_server_url, "http://127.0.0.1:5000");
    nh.param<std::string>("cloud_topic", cloud_topic, "/cloud_registered");
    nh.param<std::string>("odom_topic", odom_topic, "");
    
    // 读取自适应滤波参数
    nh.param<float>("near_threshold", adaptive_params.near_threshold, 5.0f);
    nh.param<float>("far_threshold", adaptive_params.far_threshold, 15.0f);
    nh.param<float>("voxel_near", adaptive_params.voxel_near, 0.05f);
    nh.param<float>("voxel_mid", adaptive_params.voxel_mid, 0.2f);
    nh.param<float>("voxel_far", adaptive_params.voxel_far, 0.5f);
    
    // 读取事件触发参数
    nh.param<float>("min_translation", trigger_params.min_translation, 0.5f);
    nh.param<float>("min_rotation", trigger_params.min_rotation, 5.0f);
    nh.param<float>("max_interval", trigger_params.max_interval, 2.0f);

    ROS_INFO("Vehicle ID: %s", vehicle_id.c_str());
    ROS_INFO("Cloud server URL: %s", cloud_server_url.c_str());
    ROS_INFO("Cloud Topic: %s", cloud_topic.c_str());
    ROS_INFO("Odom Topic: %s", odom_topic.c_str());
    ROS_INFO("Adaptive Filter Params - Near: %.2fm (≤%.2fm), Mid: %.2fm (%.2f-%.2fm), Far: ≥%.2fm (>%.2fm)",
             adaptive_params.voxel_near, adaptive_params.near_threshold,
             adaptive_params.voxel_mid, adaptive_params.near_threshold, adaptive_params.far_threshold,
             adaptive_params.voxel_far, adaptive_params.far_threshold);
    ROS_INFO("Event Trigger Params - Min translation: %.2fm, Min rotation: %.2f°, Max interval: %.2fs",
             trigger_params.min_translation, trigger_params.min_rotation, trigger_params.max_interval);

    // 信号注册
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // HTTP客户端初始化
    cli = std::make_unique<httplib::Client>(cloud_server_url);

    // 订阅与服务
    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 10, pointCloudCallback);
    ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::Pose2D>("/gps", 10, gpsCallback);
    ros::Subscriber odom_sub;
    if (!odom_topic.empty()) {
        odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, odomCallback);
    }
    ros::ServiceServer updateGPSservice = nh.advertiseService("update_gps", UpdateGPSCallback);
    ros::ServiceServer syncGPSservice = nh.advertiseService("sync_gps", SyncGPSCallback);

    ros::spin();
    return 0;
}
