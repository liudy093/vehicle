#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <httplib.h>

#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

namespace {

using boost::property_tree::ptree;

std::string vehicle_id;
std::string cloud_server_url;
std::string cloud_topic;
std::string odom_topic;
double send_interval_sec = 0.5;

std::unique_ptr<httplib::Client> client;
nav_msgs::Odometry latest_odom;
bool odom_ready = false;
ros::Time last_send_time;
bool first_send = true;
unsigned long sent_count = 0;

ptree makeStampTree(const ros::Time& stamp) {
    ptree stamp_tree;
    stamp_tree.put("sec", static_cast<uint32_t>(stamp.sec));
    stamp_tree.put("nanosec", static_cast<uint32_t>(stamp.nsec));
    return stamp_tree;
}

ptree makePointFieldTree(const sensor_msgs::PointField& field) {
    ptree field_tree;
    field_tree.put("name", field.name);
    field_tree.put("offset", field.offset);
    field_tree.put("datatype", static_cast<uint32_t>(field.datatype));
    field_tree.put("count", field.count);
    return field_tree;
}

std::string encodeCloudData(const sensor_msgs::PointCloud2& cloud_msg) {
    const std::string bytes(
        reinterpret_cast<const char*>(cloud_msg.data.data()),
        cloud_msg.data.size());
    return httplib::detail::base64_encode(bytes);
}

bool shouldSendNow() {
    if (first_send) {
        return true;
    }
    return (ros::Time::now() - last_send_time).toSec() >= send_interval_sec;
}

std::string buildFastLioPayload(const sensor_msgs::PointCloud2& cloud_msg) {
    ptree root;
    root.put("vehicle_id", vehicle_id);
    root.put("cloud_topic", cloud_topic);
    root.put("odom_topic", odom_topic);

    ptree header;
    header.add_child("stamp", makeStampTree(cloud_msg.header.stamp));
    header.put("frame_id", cloud_msg.header.frame_id);
    root.add_child("header", header);

    ptree shape;
    shape.put("height", cloud_msg.height);
    shape.put("width", cloud_msg.width);
    root.add_child("shape", shape);

    ptree fields;
    for (const auto& field : cloud_msg.fields) {
        fields.push_back(std::make_pair("", makePointFieldTree(field)));
    }
    root.add_child("fields", fields);

    ptree layout;
    layout.put("is_bigendian", cloud_msg.is_bigendian);
    layout.put("point_step", cloud_msg.point_step);
    layout.put("row_step", cloud_msg.row_step);
    layout.put("is_dense", cloud_msg.is_dense);
    root.add_child("layout", layout);
    root.put("data_b64", encodeCloudData(cloud_msg));

    ptree odom;
    ptree odom_header;
    odom_header.add_child("stamp", makeStampTree(latest_odom.header.stamp));
    odom_header.put("frame_id", latest_odom.header.frame_id);
    odom.add_child("header", odom_header);
    odom.put("child_frame_id", latest_odom.child_frame_id);

    ptree pose;
    ptree position;
    position.put("x", latest_odom.pose.pose.position.x);
    position.put("y", latest_odom.pose.pose.position.y);
    position.put("z", latest_odom.pose.pose.position.z);
    pose.add_child("position", position);

    ptree orientation;
    orientation.put("x", latest_odom.pose.pose.orientation.x);
    orientation.put("y", latest_odom.pose.pose.orientation.y);
    orientation.put("z", latest_odom.pose.pose.orientation.z);
    orientation.put("w", latest_odom.pose.pose.orientation.w);
    pose.add_child("orientation", orientation);
    odom.add_child("pose", pose);
    root.add_child("odom", odom);

    std::ostringstream stream;
    boost::property_tree::write_json(stream, root, false);
    return stream.str();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    latest_odom = *odom_msg;
    odom_ready = true;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (!odom_ready) {
        ROS_WARN_THROTTLE(5.0, "FAST-LIO odometry not ready yet; skipping cloud send");
        return;
    }
    if (!shouldSendNow()) {
        return;
    }

    const std::string payload = buildFastLioPayload(*cloud_msg);
    auto response = client->Post("/receive_fastlio", payload, "application/json");
    if (!response) {
        ROS_ERROR_THROTTLE(5.0, "Failed to connect to cloud server for FAST-LIO payload");
        return;
    }
    if (response->status != 200) {
        ROS_ERROR_THROTTLE(
            5.0,
            "FAST-LIO upload failed with status %d: %s",
            response->status,
            response->body.c_str());
        return;
    }

    last_send_time = ros::Time::now();
    first_send = false;
    ++sent_count;
    ROS_INFO(
        "FAST-LIO frame sent successfully (count=%lu, stamp=%u)",
        sent_count,
        cloud_msg->header.stamp.sec);
}

}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "fastlio_http_sender");
    ros::NodeHandle nh("~");

    nh.param<std::string>("vehicle_id", vehicle_id, "vehicle2");
    nh.param<std::string>("cloud_server_url", cloud_server_url, "http://127.0.0.1:5000");
    nh.param<std::string>("cloud_topic", cloud_topic, "/cloud_registered");
    nh.param<std::string>("odom_topic", odom_topic, "/Odometry");
    nh.param<double>("send_interval_sec", send_interval_sec, 0.5);

    ROS_INFO("vehicle_id=%s", vehicle_id.c_str());
    ROS_INFO("cloud_server_url=%s", cloud_server_url.c_str());
    ROS_INFO("cloud_topic=%s", cloud_topic.c_str());
    ROS_INFO("odom_topic=%s", odom_topic.c_str());
    ROS_INFO("send_interval_sec=%.2f", send_interval_sec);

    client = std::make_unique<httplib::Client>(cloud_server_url);

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 10, odomCallback);
    ros::Subscriber cloud_sub =
        nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 10, cloudCallback);

    ros::spin();
    return 0;
}
