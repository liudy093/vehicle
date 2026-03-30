import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node

from data_sender_ros2.autodrive_bridge import fill_pose2d_message_from_gps


def load_gps_interface():
    try:
        from car_interfaces.msg import GpsInterface
    except ImportError as exc:  # pragma: no cover - exercised only in ROS runtime
        raise RuntimeError(
            "car_interfaces.msg.GpsInterface is required. "
            "Source the AutoDrive workspace before running gps_pose_bridge_node."
        ) from exc
    return GpsInterface


class GpsPoseBridgeNode(Node):
    def __init__(self):
        super().__init__("gps_pose_bridge_node")
        self.declare_parameter("input_topic", "/gps_data")
        self.declare_parameter("output_topic", "/gps")
        self.declare_parameter("yaw_in_degrees", True)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.yaw_in_degrees = bool(self.get_parameter("yaw_in_degrees").value)

        gps_type = load_gps_interface()
        self.publisher = self.create_publisher(Pose2D, output_topic, 10)
        self.create_subscription(gps_type, input_topic, self.gps_callback, 10)

        self.get_logger().info(f"input_topic={input_topic}")
        self.get_logger().info(f"output_topic={output_topic}")
        self.get_logger().info(f"yaw_in_degrees={self.yaw_in_degrees}")

    def gps_callback(self, msg):
        pose_msg = Pose2D()
        fill_pose2d_message_from_gps(
            pose_msg,
            msg,
            yaw_in_degrees=self.yaw_in_degrees,
        )
        self.publisher.publish(pose_msg)


def main():
    rclpy.init()
    node = GpsPoseBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
