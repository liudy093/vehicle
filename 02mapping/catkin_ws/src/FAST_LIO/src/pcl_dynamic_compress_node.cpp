// // pcl_dynamic_compress_node.cpp
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/ByteMultiArray.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/compression/octree_pointcloud_compression.h>
// #include <pcl/io/impl/octree_pointcloud_compression.hpp>
// #include <mutex>
// #include <sstream>

// using Float = float;

// // Global encoder pointer (we recreate when params change)
// pcl::io::OctreePointCloudCompression<pcl::PointXYZI>* encoder = nullptr;
// std::mutex enc_mutex;

// // Current desired retention ratio r ∈ [0.1, 1.0] (from RL)
// Float current_r = 1.0f;
// std::mutex r_mutex;

// // Last applied r to encoder (to decide rebuild)
// Float last_applied_r = -1.0f;

// // Threshold to rebuild encoder
// const Float REBUILD_THRESHOLD = 0.02f; // rebuild only when r changes > 0.02

// // Topic publishers
// ros::Publisher pub_compressed;            // ByteMultiArray with compressed bytes
// ros::Publisher pub_compressed_size;       // optional, publish size for reward calc

// // Map retention ratio r to encoder parameters.
// void r_to_params(Float r, double &pointResolution, double &octreeResolution, bool &doVoxelGridDownSampling)
// {
//     double pointRes_min = 0.001; // best
//     double pointRes_max = 0.05;  // coarse
//     double octRes_min   = 0.01;
//     double octRes_max   = 0.2;

//     pointResolution  = pointRes_min  + (pointRes_max  - pointRes_min)  * (1.0 - r);
//     octreeResolution = octRes_min    + (octRes_max    - octRes_min)    * (1.0 - r);
//     doVoxelGridDownSampling = (r < 0.3f);
// }

// void rebuild_encoder_if_needed(Float r)
// {
//     std::lock_guard<std::mutex> lk(enc_mutex);
//     if (encoder != nullptr && std::abs(r - last_applied_r) < REBUILD_THRESHOLD) {
//         return; // small change, reuse
//     }

//     if (encoder) { delete encoder; encoder = nullptr; }

//     double pointResolution, octreeResolution;
//     bool doVoxelGridDownSampling;
//     r_to_params(r, pointResolution, octreeResolution, doVoxelGridDownSampling);

//     pcl::io::compression_Profiles_e profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

//     encoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>(
//         profile,
//         true,               // showStatistics
//         pointResolution,
//         octreeResolution,
//         doVoxelGridDownSampling,
//         0,                  // frameRate (not used)
//         false,              // doColorEncoding
//         6                   // colorBitResolution
//     );

//     last_applied_r = r;
//     ROS_INFO("Rebuilt encoder: r=%.3f pointRes=%.4f octRes=%.4f voxDown=%d", r, pointResolution, octreeResolution, (int)doVoxelGridDownSampling);
// }

// void compress_and_publish(const sensor_msgs::PointCloud2ConstPtr& msg)
// {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*msg, *cloud);

//     Float r;
//     {
//         std::lock_guard<std::mutex> lk(r_mutex);
//         r = current_r;
//     }

//     rebuild_encoder_if_needed(r);

//     std::stringstream compressed;
//     {
//         std::lock_guard<std::mutex> lk(enc_mutex);
//         if (!encoder) {
//             ROS_ERROR_THROTTLE(5, "encoder is null");
//             return;
//         }
//         encoder->encodePointCloud(cloud, compressed);
//     }

//     std::string s = compressed.str();
//     std_msgs::ByteMultiArray out;
//     out.data.insert(out.data.end(), s.begin(), s.end());
//     pub_compressed.publish(out);

//     std_msgs::Float32 size_msg;
//     size_msg.data = static_cast<float>(s.size());
//     pub_compressed_size.publish(size_msg);

//     ROS_DEBUG("Compressed frame pts=%lu -> %lu bytes (r=%.3f)", cloud->size(), s.size(), r);
// }

// // RL publishes desired rate to /compress_rate (Float32)
// void compress_rate_cb(const std_msgs::Float32::ConstPtr& msg)
// {
//     Float r = msg->data;
//     if (r < 0.05f) r = 0.05f;
//     if (r > 1.0f)  r = 1.0f;
//     {
//         std::lock_guard<std::mutex> lk(r_mutex);
//         current_r = r;
//     }
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "pcl_dynamic_compress_node");
//     ros::NodeHandle nh("~");

//     std::string input_cloud_topic;
//     nh.param<std::string>("input_cloud", input_cloud_topic, "/cloud_registered");

//     // 定义变量存放 param 的结果
//     std::string compressed_topic;
//     std::string compressed_size_topic;

//     nh.param<std::string>("compressed_topic", compressed_topic, input_cloud_topic + std::string("_compressed"));
//     nh.param<std::string>("compressed_size_topic", compressed_size_topic, std::string("compressed_size"));

//     // Subscribe to RL commands
//     ros::Subscriber sub_rate = nh.subscribe<std_msgs::Float32>("/compress_rate", 10, compress_rate_cb);

//     // Subscribe to point cloud
//     ros::Subscriber sub_cloud = nh.subscribe(input_cloud_topic, 2, compress_and_publish);

//     // Publishers
//     pub_compressed = nh.advertise<std_msgs::ByteMultiArray>(compressed_topic, 5);
//     pub_compressed_size = nh.advertise<std_msgs::Float32>(compressed_size_topic, 30);

//     // init encoder with default r=1.0 (no compression)
//     {
//         std::lock_guard<std::mutex> lk(r_mutex);
//         current_r = 1.0f;
//     }
//     rebuild_encoder_if_needed(current_r);

//     ros::spin();

//     if (encoder) {
//         delete encoder;
//         encoder = nullptr;
//     }

//     return 0;
// }

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/compression/octree_pointcloud_compression.h>

// class DynamicPCLCompressor
// {
// public:
//     DynamicPCLCompressor()
//     {
//         // 设置高压缩率参数
//         bool showStatistics = true;

//         compressor_ = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(
//             pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR,
//             showStatistics,
//             0.03f,    // pointResolution
//             0.2f,      // octreeResolution
//             false,      // doVoxelGridDownSampling
//             0.1f,      // voxelGridSize
//             true        // doColorEncoding
//         );

//         sub_ = nh_.subscribe("/cloud_registered", 5, &DynamicPCLCompressor::cloudCallback, this);
//     }

//     ~DynamicPCLCompressor()
//     {
//         delete compressor_;
//     }

//     void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
//     {
//         // ROS -> PCL
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(*msg, *cloud);

//         std::stringstream compressedData;

//         // 压缩
//         compressor_->encodePointCloud(cloud, compressedData);

//         ROS_INFO("[Compressed] pts=%d bytes=%ld",
//                  (int)cloud->size(),
//                  (long)compressedData.str().size());
//     }

// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber sub_;
//     pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* compressor_;
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "pcl_dynamic_compress_node");
//     DynamicPCLCompressor compressor;
//     ros::spin();
//     return 0;
// }


// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/compression/octree_pointcloud_compression.h>
// #include <pcl/io/impl/octree_pointcloud_compression.hpp>

// int frame_id = 0;

// boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZI>> encoder;
// boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZI>> decoder;

// // 最近邻 RMSE 计算函数
// double computeRMSE_NN(
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr& original,
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr& decoded,
//     double &max_error)
// {
//     pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//     kdtree.setInputCloud(original);

//     double sum_sq = 0.0;
//     max_error = 0.0;

//     std::vector<int> nearest_indices(1);
//     std::vector<float> nearest_distances(1);

//     for (size_t i = 0; i < decoded->points.size(); ++i)
//     {
//         const pcl::PointXYZI &p = decoded->points[i];

//         if (kdtree.nearestKSearch(p, 1, nearest_indices, nearest_distances) > 0)
//         {
//             double dist = std::sqrt(nearest_distances[0]);
//             sum_sq += dist * dist;

//             if (dist > max_error)
//                 max_error = dist;
//         }
//     }

//     if (decoded->points.empty())
//         return 0.0;

//     return std::sqrt(sum_sq / decoded->points.size());
// }

// void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
// {
//     frame_id++;

//     // --- 原始点云 ---
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*msg, *cloud);
//     pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_const(cloud);

//     size_t original_bytes = msg->data.size();
//     size_t point_num = cloud->size();

//     // --- 压缩 ---
//     std::stringstream compressedData;
//     encoder->encodePointCloud(cloud_const, compressedData);
//     size_t compressed_bytes = compressedData.str().size();

//     // --- 解压 ---
//     pcl::PointCloud<pcl::PointXYZI>::Ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     decoder->decodePointCloud(compressedData, decompressed_cloud);

//     size_t decompressed_points = decompressed_cloud->size();

//     // --- 使用最近邻 RMSE ---
//     double max_error = 0.0;
//     double rmse = computeRMSE_NN(cloud, decompressed_cloud, max_error);

//     // --- 压缩率 ---
//     double ratio = (double)original_bytes / (double)compressed_bytes;
//     double percent = (double)compressed_bytes / (double)original_bytes * 100.0;
//     double xyz_bpp = (double)compressed_bytes / (double)point_num;

//     // ===== 输出结果 =====
//     ROS_INFO("\n----------------------------------------");
//     ROS_INFO("Frame ID: %d", frame_id);
//     ROS_INFO("Original points:     %zu", point_num);
//     ROS_INFO("Decompressed points: %zu", decompressed_points);
//     ROS_INFO("Point count change:  %ld", (long)decompressed_points - (long)point_num);

//     ROS_INFO("Nearest Neighbor RMSE: %.6f m", rmse);
//     ROS_INFO("Nearest Neighbor Max error: %.6f m", max_error);

//     ROS_INFO("Uncompressed size: %.2f kBytes", original_bytes / 1024.0);
//     ROS_INFO("Compressed size:   %.2f kBytes", compressed_bytes / 1024.0);

//     ROS_INFO("Compression ratio: %.3f", ratio);
//     ROS_INFO("Compressed percentage: %.3f %%", percent);
//     ROS_INFO("Bytes per point: %.6f", xyz_bpp);

//     ROS_INFO("----------------------------------------\n");
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "pcl_dynamic_compress_node");
//     ros::NodeHandle nh;

//     bool showStatistics = false;

//     pcl::io::compression_Profiles_e compressionProfile =
//         pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

//     float pointResolution = 0.001f;
//     float octreeResolution = 0.01f;

//     encoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>(
//         compressionProfile,
//         showStatistics,
//         pointResolution,
//         octreeResolution,
//         false,  // 不压缩颜色
//         0.01f
//     ));

//     decoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>());

//     ros::Subscriber sub = nh.subscribe("/cloud_registered", 1, cloudCallback);

//     ROS_INFO("PCL Octree Compression Node Started.");
//     ros::spin();
//     return 0;
// }


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/impl/octree_pointcloud_compression.hpp>

int frame_id = 0;

// 动态速度
double linear_v = 0.0;
double angular_v = 0.0;

// 阈值（你可调整）
double low_thresh = 0.5;    // 低速阈值
double high_thresh = 1.5;   // 高速阈值

// 当前压缩等级
int current_profile = -1; // -1 表示未初始化

boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZI>> encoder;
boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZI>> decoder;

// 创建压缩器（可动态调用）
void createEncoder(int mode)
{
    pcl::io::compression_Profiles_e profile;

    if (mode == 0)
        profile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    else if (mode == 1)
        profile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
    else
        profile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

    float pointResolution = 0.001f;
    float octreeResolution = 0.01f;

    encoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>(
        profile,
        false,
        pointResolution,
        octreeResolution,
        false,
        0.01f
    ));
}

// 最近邻 RMSE
double computeRMSE_NN(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& original,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& decoded,
    double &max_error)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(original);

    double sum_sq = 0.0;
    max_error = 0.0;

    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_distances(1);

    for (size_t i = 0; i < decoded->points.size(); ++i)
    {
        const pcl::PointXYZI &p = decoded->points[i];

        if (kdtree.nearestKSearch(p, 1, nearest_indices, nearest_distances) > 0)
        {
            double dist = std::sqrt(nearest_distances[0]);
            sum_sq += dist * dist;

            if (dist > max_error)
                max_error = dist;
        }
    }

    if (decoded->points.empty())
        return 0.0;

    return std::sqrt(sum_sq / decoded->points.size());
}

// --- 速度回调 ---
void twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
    linear_v = msg->linear.x;
    angular_v = msg->angular.z;
}

// --- 点云回调 ---
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    frame_id++;

    // --- 速度判断 ---
    double speed = std::sqrt(linear_v * linear_v + angular_v * angular_v);

    int desired_profile = 0; // default LOW

    if (speed < low_thresh)
        desired_profile = 0;
    else if (speed < high_thresh)
        desired_profile = 1;
    else
        desired_profile = 2;

    // --- 检测压缩等级变化 ---
    if (desired_profile != current_profile)
    {
        current_profile = desired_profile;
        createEncoder(current_profile);

        if (current_profile == 0)
            ROS_WARN("Switch to LOW compression (speed=%.2f)", speed);
        else if (current_profile == 1)
            ROS_WARN("Switch to MED compression (speed=%.2f)", speed);
        else
            ROS_WARN("Switch to HIGH compression (speed=%.2f)", speed);
    }

    // --- 原始点云 ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_const(cloud);

    size_t original_bytes = msg->data.size();
    size_t point_num = cloud->size();

    // --- 压缩 ---
    std::stringstream compressedData;
    encoder->encodePointCloud(cloud_const, compressedData);
    size_t compressed_bytes = compressedData.str().size();

    // --- 解压 ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    decoder->decodePointCloud(compressedData, decompressed_cloud);

    // --- 质量评估（最近邻 RMSE） ---
    double max_error = 0.0;
    double rmse = computeRMSE_NN(cloud, decompressed_cloud, max_error);

    double ratio = (double)original_bytes / (double)compressed_bytes;
    double percent = (double)compressed_bytes / (double)original_bytes * 100.0;
    double xyz_bpp = (double)compressed_bytes / (double)point_num;

    // --- 输出 ---
    ROS_INFO("\n----------------------------------------");
    ROS_INFO("Frame ID: %d (Profile: %d)", frame_id, current_profile);
    ROS_INFO("Speed: %.3f  (v=%.3f, w=%.3f)", speed, linear_v, angular_v);

    ROS_INFO("NN RMSE: %.6f m", rmse);
    ROS_INFO("NN Max error: %.6f m", max_error);

    ROS_INFO("Original: %.2f kB   Compressed: %.2f kB", 
             original_bytes / 1024.0, compressed_bytes / 1024.0);

    ROS_INFO("Compression ratio: %.3f", ratio);
    ROS_INFO("Compressed percentage: %.2f %%", percent);
    ROS_INFO("Bytes per point: %.6f", xyz_bpp);
    ROS_INFO("----------------------------------------\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_dynamic_compress_node");
    ros::NodeHandle nh;

    decoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>());

    ros::Subscriber sub_twist = nh.subscribe("/cmd_vel", 1, twistCallback);
    ros::Subscriber sub_cloud = nh.subscribe("/cloud_registered", 1, cloudCallback);

    ROS_INFO("PCL Dynamic Compression Node Started.");
    ros::spin();
    return 0;
}


// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>

// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/compression/octree_pointcloud_compression.h>
// #include <pcl/io/impl/octree_pointcloud_compression.hpp>

// int frame_id = 0;

// // 压缩器
// boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZI>> encoder;
// boost::shared_ptr<pcl::io::OctreePointCloudCompression<pcl::PointXYZI>> decoder;

// // --- 最近邻 RMSE 计算 ---
// double computeRMSE_NN(
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr& original,
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr& decoded,
//     double &max_error)
// {
//     pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
//     kdtree.setInputCloud(original);

//     double sum_sq = 0.0;
//     max_error = 0.0;

//     std::vector<int> nearest_indices(1);
//     std::vector<float> nearest_distances(1);

//     for (size_t i = 0; i < decoded->points.size(); ++i)
//     {
//         const pcl::PointXYZI &p = decoded->points[i];
//         if (kdtree.nearestKSearch(p, 1, nearest_indices, nearest_distances) > 0)
//         {
//             double dist = std::sqrt(nearest_distances[0]);
//             sum_sq += dist * dist;
//             if (dist > max_error) max_error = dist;
//         }
//     }

//     if (decoded->points.empty())
//         return 0.0;

//     return std::sqrt(sum_sq / decoded->points.size());
// }

// // --- 根据速度或 RL 动作设置压缩率 ---
// void setCompressionProfile(int action)
// {
//     pcl::io::compression_Profiles_e profile =
//         pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

//     float pointResolution;
//     float octreeResolution;

//     switch(action){
//         case 0: // High compression -> 粗压缩
//             pointResolution = 0.005f;
//             octreeResolution = 0.05f;
//             break;
//         case 1: // Med compression
//             pointResolution = 0.002f;
//             octreeResolution = 0.02f;
//             break;
//         case 2: // Low compression -> 精压缩
//         default:
//             pointResolution = 0.001f;
//             octreeResolution = 0.01f;
//             break;
//     }

//     encoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>(
//         profile,
//         false,
//         pointResolution,
//         octreeResolution,
//         false,
//         0.01f
//     ));
// }

// void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
// {
//     frame_id++;

//     // --- 原始点云 ---
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::fromROSMsg(*msg, *cloud);

//     pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_const(cloud);

//     size_t original_bytes = msg->data.size();
//     size_t point_num = cloud->size();

//     // --- 动态选择压缩率：这里示例用简单规则，可替换为 RL 动作 ---
//     // 例如速度阈值判断（假设从topic获得线速度 linear_vel）
//     float linear_vel = 0.0; // TODO: 替换为你的速度订阅/计算
//     int action = 2; // 默认 low compression
//     if(linear_vel > 3.0f) action = 0; // 高速 -> 高压缩
//     else if(linear_vel > 1.0f) action = 1; // 中速 -> 中压缩

//     setCompressionProfile(action);

//     // --- 压缩 ---
//     std::stringstream compressedData;
//     encoder->encodePointCloud(cloud_const, compressedData);
//     size_t compressed_bytes = compressedData.str().size();

//     // --- 解压 ---
//     pcl::PointCloud<pcl::PointXYZI>::Ptr decompressed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     decoder->decodePointCloud(compressedData, decompressed_cloud);
//     size_t decompressed_points = decompressed_cloud->size();

//     // --- RMSE ---
//     double max_error = 0.0;
//     double rmse = computeRMSE_NN(cloud, decompressed_cloud, max_error);

//     // --- 压缩率 ---
//     double ratio = (double)original_bytes / (double)compressed_bytes;
//     double percent = (double)compressed_bytes / (double)original_bytes * 100.0;
//     double xyz_bpp = (double)compressed_bytes / (double)point_num;

//     // ===== 输出结果 =====
//     ROS_INFO("\n----------------------------------------");
//     ROS_INFO("Frame ID: %d", frame_id);
//     ROS_INFO("Original points:     %zu", point_num);
//     ROS_INFO("Decompressed points: %zu", decompressed_points);
//     ROS_INFO("Point count change:  %ld", (long)decompressed_points - (long)point_num);

//     ROS_INFO("Nearest Neighbor RMSE: %.6f m", rmse);
//     ROS_INFO("Nearest Neighbor Max error: %.6f m", max_error);

//     ROS_INFO("Uncompressed size: %.2f kBytes", original_bytes / 1024.0);
//     ROS_INFO("Compressed size:   %.2f kBytes", compressed_bytes / 1024.0);

//     ROS_INFO("Compression ratio: %.3f", ratio);
//     ROS_INFO("Compressed percentage: %.3f %%", percent);
//     ROS_INFO("Bytes per point: %.6f", xyz_bpp);
//     ROS_INFO("----------------------------------------\n");
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "pcl_dynamic_compress_node");
//     ros::NodeHandle nh;

//     // 统一初始化压缩器
//     pcl::io::compression_Profiles_e profile =
//         pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

//     encoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>(
//         profile, false, 0.001f, 0.01f, false, 0.01f));

//     decoder.reset(new pcl::io::OctreePointCloudCompression<pcl::PointXYZI>());

//     ros::Subscriber sub = nh.subscribe("/cloud_registered", 1, cloudCallback);

//     ROS_INFO("PCL Octree Compression Node Started.");
//     ros::spin();
//     return 0;
// }


// // 文件: pcl_predictive_residual_compress_node.cpp
// // 说明: Predictive residual + PCL Octree compression prototype
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_listener.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl_conversions/pcl_conversions.h>

// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/compression/octree_pointcloud_compression.h>
// #include <pcl/io/impl/octree_pointcloud_compression.hpp>

// #include <sstream>
// #include <vector>
// #include <cmath>
// #include <chrono>

// using PointT = pcl::PointXYZI;

// boost::shared_ptr<pcl::io::OctreePointCloudCompression<PointT>> encoder;
// boost::shared_ptr<pcl::io::OctreePointCloudCompression<PointT>> decoder;

// pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>());
// bool have_prev = false;

// double current_speed = 0.0;
// tf::TransformListener *tf_listener = nullptr;
// std::string map_frame = "map";      // 可改为 "odom" / "base_link" 视你TF树
// std::string sensor_frame = "velodyne"; // 你点云的frame_id

// int frame_id = 0;

// // 参数（可调）
// double eps_base = 0.15;       // 基础残差阈值 (m) — 越大压缩越强
// double eps_min = 0.02;        // 最小阈值 (m)
// double eps_max = 0.5;         // 最大阈值 (m)
// double speed_norm_max = 5.0;  // 用于归一化速度（m/s）
// double speed_influence = 0.8; // speed对eps的影响系数 (0~1)
// double voxel_leaf = 0.02;     // 可选体素下采样大小（m）用于残差点云

// // 将 prev_cloud 变换到 target_frame（使用 TF）
// bool transformPointCloudToFrame(const pcl::PointCloud<PointT>::Ptr &in,
//                                 pcl::PointCloud<PointT>::Ptr &out,
//                                 const std::string &target_frame,
//                                 const std::string &source_frame,
//                                 const ros::Time &stamp)
// {
//     if (!tf_listener) return false;
//     tf::StampedTransform transform;
//     try {
//         tf_listener->waitForTransform(target_frame, source_frame, stamp, ros::Duration(0.05));
//         tf_listener->lookupTransform(target_frame, source_frame, stamp, transform);
//     } catch (tf::TransformException &ex) {
//         ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
//         return false;
//     }

//     out->clear();
//     out->header = in->header;
//     out->is_dense = in->is_dense;

//     tf::Vector3 t = transform.getOrigin();
//     tf::Matrix3x3 R = transform.getBasis();

//     for (const auto &p : in->points) {
//         tf::Vector3 v(p.x, p.y, p.z);
//         tf::Vector3 vt = R * v + t;
//         PointT q;
//         q.x = vt.x(); q.y = vt.y(); q.z = vt.z(); q.intensity = p.intensity;
//         out->points.push_back(q);
//     }
//     out->width = out->points.size();
//     out->height = 1;
//     return true;
// }

// double compute_speed_norm(double speed) {
//     double s = std::min(std::abs(speed), speed_norm_max);
//     return s / speed_norm_max; // 0..1
// }

// void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg) {
//     current_speed = std::abs(msg->linear.x);
// }

// void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
//     frame_id++;
//     ros::Time stamp = msg->header.stamp;

//     pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//     pcl::fromROSMsg(*msg, *cloud);

//     // 可选：体素下采样以降低计算（保持足够密度）
//     // TODO: add voxel filter if needed

//     // 1) 预测：用 prev_cloud （若存在）变换到当前frame（使用TF），否则直接使用 prev as-is
//     pcl::PointCloud<PointT>::Ptr pred_cloud(new pcl::PointCloud<PointT>());
//     bool have_prediction = false;
//     if (have_prev) {
//         // try TF transform prev to current sensor frame
//         if (transformPointCloudToFrame(prev_cloud, pred_cloud, msg->header.frame_id, prev_cloud->header.frame_id, stamp)) {
//             have_prediction = true;
//         } else {
//             // fallback: use prev_cloud as-is (possible if frames are same)
//             *pred_cloud = *prev_cloud;
//             have_prediction = true;
//         }
//     }

//     // 2) compute residuals: for each point in cloud, find nearest in pred_cloud; if dist > eps => residual
//     pcl::PointCloud<PointT>::Ptr residual_cloud(new pcl::PointCloud<PointT>());
//     if (have_prediction && pred_cloud->size() > 5) {
//         pcl::KdTreeFLANN<PointT> kdtree;
//         kdtree.setInputCloud(pred_cloud);

//         // adapt eps by speed: 当速度大时，eps更小（保留更多）
//         double speed_norm = compute_speed_norm(current_speed); // 0..1
//         double eps = eps_base * (1.0 - speed_influence * speed_norm);
//         if (eps < eps_min) eps = eps_min;
//         if (eps > eps_max) eps = eps_max;

//         // iterate through points
//         std::vector<int> idx(1);
//         std::vector<float> dist2(1);
//         for (size_t i = 0; i < cloud->points.size(); ++i) {
//             const PointT &p = cloud->points[i];
//             if (kdtree.nearestKSearch(p, 1, idx, dist2) > 0) {
//                 double d = std::sqrt(dist2[0]);
//                 if (d > eps) {
//                     residual_cloud->points.push_back(p);
//                 }
//             } else {
//                 // no match -> residual
//                 residual_cloud->points.push_back(p);
//             }
//         }
//     } else {
//         // no prediction available -> treat entire cloud as residual
//         *residual_cloud = *cloud;
//     }

//     residual_cloud->width = residual_cloud->points.size();
//     residual_cloud->height = 1;

//     // 3) optional: voxel downsample residual to reduce bandwidth
//     if (voxel_leaf > 0.0 && residual_cloud->size() > 0) {
//         pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
//         std::vector<int> indices;
//         pcl::VoxelGrid<PointT> vg;
//         vg.setInputCloud(residual_cloud);
//         vg.setLeafSize(voxel_leaf, voxel_leaf, voxel_leaf);
//         vg.filter(*tmp);
//         residual_cloud.swap(tmp);
//     }

//     // 4) compress residual_cloud using PCL Octree compression
//     std::stringstream compressedData;
//     pcl::PointCloud<PointT>::ConstPtr residual_const(residual_cloud);

//     // encode
//     encoder->encodePointCloud(residual_const, compressedData);
//     size_t compressed_bytes = compressedData.str().size();

//     // For evaluation: original raw size (approx msg->data) and residual raw (estimate)
//     size_t original_bytes = msg->data.size();
//     size_t residual_raw_bytes = residual_cloud->size() * sizeof(PointT); // rough estimate

//     // decode for RMSE evaluation
//     pcl::PointCloud<PointT>::Ptr decoded_residual(new pcl::PointCloud<PointT>());
//     // decoder expects pointer to PointCloud Ptr
//     // Note: PCL decoder API requires PointCloudPtr&, so we create one
//     boost::shared_ptr<pcl::PointCloud<PointT>> decoded_ptr(new pcl::PointCloud<PointT>());
//     std::stringstream in(compressedData.str());
//     decoder->decodePointCloud(in, decoded_ptr);
//     decoded_residual = decoded_ptr;

//     // reconstruct predicted->reconstructed = pred + decoded_residual approximate:
//     // For nearest matching we skip vector addition; instead compute NN-RMSE between original cloud and reconstructed
//     // For simplicity compute NN-RMSE between original cloud and (pred + decoded_residual) approximate by directly using decoded_residual + pred points
//     // Here we will compute NN-RMSE between original cloud and (pred union decoded_residual)
//     pcl::PointCloud<PointT>::Ptr recon_cloud(new pcl::PointCloud<PointT>());
//     if (have_prediction) {
//         // merge pred_cloud and decoded_residual (simple union)
//         *recon_cloud = *pred_cloud;
//         recon_cloud->insert(recon_cloud->end(), decoded_residual->begin(), decoded_residual->end());
//     } else {
//         recon_cloud = decoded_residual;
//     }

//     // compute NN-RMSE: for each point in original cloud find nearest in recon_cloud
//     double sum_sq = 0.0;
//     double max_err = 0.0;
//     pcl::KdTreeFLANN<PointT> kdt_recon;
//     if (recon_cloud->size() > 0) {
//         kdt_recon.setInputCloud(recon_cloud);
//         std::vector<int> nidx(1);
//         std::vector<float> ndist(1);
//         for (size_t i = 0; i < cloud->points.size(); ++i) {
//             if (kdt_recon.nearestKSearch(cloud->points[i], 1, nidx, ndist) > 0) {
//                 double d = std::sqrt(ndist[0]);
//                 sum_sq += d*d;
//                 if (d > max_err) max_err = d;
//             } else {
//                 // no neighbor -> treat large error
//                 double d = 5.0;
//                 sum_sq += d*d;
//                 if (d > max_err) max_err = d;
//             }
//         }
//     } else {
//         // recon empty -> huge error
//         sum_sq = 1e6;
//         max_err = 100.0;
//     }
//     double rmse = 0.0;
//     if (cloud->size() > 0) rmse = std::sqrt(sum_sq / (double)cloud->size());

//     // compute compression ratio
//     double ratio = (double)original_bytes / (double)compressed_bytes;
//     double percent = (double)compressed_bytes / (double)original_bytes * 100.0;
//     double bytes_per_point = (residual_cloud->size() > 0) ? ((double)compressed_bytes / (double)residual_cloud->size()) : 0.0;

//     // log
//     ROS_INFO("\n---------------- Predictive Residual Compression ----------------");
//     ROS_INFO("Frame: %d, original pts: %zu, residual pts: %zu", frame_id, cloud->size(), residual_cloud->size());
//     ROS_INFO("Original bytes (msg): %zu, residual raw est: %zu bytes", original_bytes, residual_raw_bytes);
//     ROS_INFO("Compressed bytes: %zu, ratio: %.3f, pct: %.3f%%, bytes/pt(residual): %.3f",
//              compressed_bytes, ratio, percent, bytes_per_point);
//     ROS_INFO("RMSE to reconstructed: %.4f m, max err: %.4f m, eps_used: %.4f m", rmse, max_err, 
//              std::max(eps_min, std::min(eps_max, eps_base * (1.0 - speed_influence * compute_speed_norm(current_speed)))));
//     ROS_INFO("----------------------------------------------------------------\n");

//     // update prev_cloud (store current cloud for next frame prediction)
//     *prev_cloud = *cloud;
//     prev_cloud->header.frame_id = msg->header.frame_id;
//     have_prev = true;
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "pcl_predictive_residual_compress_node");
//     ros::NodeHandle nh("~");

//     // params (can be set via rosparam)
//     nh.param("eps_base", eps_base, eps_base);
//     nh.param("eps_min", eps_min, eps_min);
//     nh.param("eps_max", eps_max, eps_max);
//     nh.param("speed_norm_max", speed_norm_max, speed_norm_max);
//     nh.param("speed_influence", speed_influence, speed_influence);
//     nh.param("voxel_leaf", voxel_leaf, voxel_leaf);
//     nh.param("sensor_frame", sensor_frame, sensor_frame);

//     tf_listener = new tf::TransformListener();

//     bool showStatistics = false;
//     pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
//     float pointResolution = 0.001f;
//     float octreeResolution = 0.01f;

//     encoder.reset(new pcl::io::OctreePointCloudCompression<PointT>(
//         compressionProfile, showStatistics, pointResolution, octreeResolution, false, 0.01f));
//     decoder.reset(new pcl::io::OctreePointCloudCompression<PointT>());

//     ros::Subscriber sub_cloud = nh.subscribe("/cloud_registered", 2, cloudCallback);
//     ros::Subscriber sub_cmd = nh.subscribe("/cmd_vel", 5, cmdVelCallback);

//     ROS_INFO("pcl_predictive_residual_compress_node started.");
//     ros::spin();

//     delete tf_listener;
//     return 0;
// }











