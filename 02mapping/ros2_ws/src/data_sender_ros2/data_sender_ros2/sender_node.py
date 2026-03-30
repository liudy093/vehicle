import http.client
import json
import socket
import threading
import urllib.error
import urllib.request

import rclpy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from data_sender_ros2.payload_builder import build_cloud_payload, build_fastlio_payload
from data_sender_ros2.topic_config import build_lidar_topic_map, resolve_configured_topics


class SenderNode(Node):
    def __init__(self):
        super().__init__("sender_node")
        self.declare_parameter("vehicle_id", "vehicle2")
        self.declare_parameter("cloud_server_url", "http://127.0.0.1:5000")
        self.declare_parameter("cloud_topic", "/forward/rslidar_points")
        self.declare_parameter("cloud_topics", "")
        self.declare_parameter("gps_topic", "/gps")
        self.declare_parameter("send_interval_sec", 1.0)
        self.declare_parameter("http_timeout_sec", 10.0)
        self.declare_parameter("fastlio_cloud_topic", "")
        self.declare_parameter("fastlio_odom_topic", "/Odometry")
        self.declare_parameter("fastlio_send_interval_sec", 0.5)

        self.vehicle_id = self.get_parameter("vehicle_id").value
        self.cloud_server_url = self.get_parameter("cloud_server_url").value.rstrip("/")
        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.cloud_topics = self.get_parameter("cloud_topics").value
        self.gps_topic = self.get_parameter("gps_topic").value
        self.send_interval_sec = float(self.get_parameter("send_interval_sec").value)
        self.http_timeout_sec = float(self.get_parameter("http_timeout_sec").value)
        self.fastlio_cloud_topic = self.get_parameter("fastlio_cloud_topic").value
        self.fastlio_odom_topic = self.get_parameter("fastlio_odom_topic").value
        self.fastlio_send_interval_sec = float(
            self.get_parameter("fastlio_send_interval_sec").value
        )
        configured_topics = resolve_configured_topics(self.cloud_topic, self.cloud_topics)
        self.lidar_topics = build_lidar_topic_map(configured_topics)

        self._gps_lock = threading.Lock()
        self._gps_pose = None
        self._odom_lock = threading.Lock()
        self._fastlio_odom = None
        self._last_send_times = {}

        for lidar_id, topic in self.lidar_topics.items():
            self.create_subscription(
                PointCloud2,
                topic,
                lambda msg, lidar_id=lidar_id, topic=topic: self.cloud_callback(
                    msg, lidar_id, topic
                ),
                10,
            )
        self.create_subscription(Pose2D, self.gps_topic, self.gps_callback, 10)
        if self.fastlio_cloud_topic:
            self.create_subscription(
                PointCloud2,
                self.fastlio_cloud_topic,
                self.fastlio_cloud_callback,
                10,
            )
            self.create_subscription(Odometry, self.fastlio_odom_topic, self.fastlio_odom_callback, 10)

        self.get_logger().info(f"vehicle_id={self.vehicle_id}")
        self.get_logger().info(f"cloud_server_url={self.cloud_server_url}")
        self.get_logger().info(f"cloud_topics={self.lidar_topics}")
        self.get_logger().info(f"gps_topic={self.gps_topic}")
        self.get_logger().info(f"http_timeout_sec={self.http_timeout_sec}")
        self.get_logger().info(f"fastlio_cloud_topic={self.fastlio_cloud_topic}")
        self.get_logger().info(f"fastlio_odom_topic={self.fastlio_odom_topic}")

    def gps_callback(self, msg):
        with self._gps_lock:
            self._gps_pose = msg

    def fastlio_odom_callback(self, msg):
        with self._odom_lock:
            self._fastlio_odom = msg

    def cloud_callback(self, msg, lidar_id, cloud_topic):
        now = self.get_clock().now()
        last_send_time = self._last_send_times.get(lidar_id)
        if last_send_time is not None:
            elapsed = (now - last_send_time).nanoseconds / 1e9
            if elapsed < self.send_interval_sec:
                return

        with self._gps_lock:
            gps_pose = self._gps_pose

        if gps_pose is None:
            self.get_logger().warn("GPS pose not available yet; skipping cloud send")
            return

        payload = build_cloud_payload(
            vehicle_id=self.vehicle_id,
            lidar_id=lidar_id,
            cloud_msg=msg,
            gps_pose=gps_pose,
            cloud_topic=cloud_topic,
        )

        gps_payload = dict(payload["gps"])
        gps_payload["vehicle_id"] = self.vehicle_id
        if self.post_json("/sync_gps", gps_payload):
            self.get_logger().debug("GPS sync sent")

        if self.post_json("/receive_ros2", payload):
            self._last_send_times[lidar_id] = now
            self.get_logger().info(
                f"Sent cloud frame sec={msg.header.stamp.sec} lidar_id={lidar_id} topic={cloud_topic}"
            )

    def fastlio_cloud_callback(self, msg):
        now = self.get_clock().now()
        last_send_time = self._last_send_times.get("fastlio")
        if last_send_time is not None:
            elapsed = (now - last_send_time).nanoseconds / 1e9
            if elapsed < self.fastlio_send_interval_sec:
                return

        with self._odom_lock:
            odom_msg = self._fastlio_odom

        if odom_msg is None:
            self.get_logger().warn("FAST-LIO odom not available yet; skipping registered cloud send")
            return

        payload = build_fastlio_payload(
            vehicle_id=self.vehicle_id,
            cloud_msg=msg,
            odom_msg=odom_msg,
            cloud_topic=self.fastlio_cloud_topic,
            odom_topic=self.fastlio_odom_topic,
        )

        if self.post_json("/receive_fastlio", payload):
            self._last_send_times["fastlio"] = now
            self.get_logger().info(
                "Sent FAST-LIO frame sec=%s topic=%s odom_topic=%s"
                % (msg.header.stamp.sec, self.fastlio_cloud_topic, self.fastlio_odom_topic)
            )

    def post_json(self, path, payload):
        body = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(
            self.cloud_server_url + path,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.http_timeout_sec) as response:
                return 200 <= response.status < 300
        except (
            urllib.error.URLError,
            socket.timeout,
            TimeoutError,
            http.client.RemoteDisconnected,
            ConnectionResetError,
            ConnectionAbortedError,
            BrokenPipeError,
        ) as exc:
            self.get_logger().error(f"POST {path} failed: {exc}")
            return False


def main():
    rclpy.init()
    node = SenderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
