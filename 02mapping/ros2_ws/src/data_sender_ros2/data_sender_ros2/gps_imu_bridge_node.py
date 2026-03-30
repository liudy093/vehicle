import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from data_sender_ros2.autodrive_bridge import fill_imu_message_from_gps


def _as_float_tuple(node, parameter_name, length):
    values = tuple(float(value) for value in node.get_parameter(parameter_name).value)
    if len(values) != length:
        raise ValueError(f"{parameter_name} must contain {length} values")
    return values


def _as_string_tuple(node, parameter_name, length):
    values = tuple(str(value) for value in node.get_parameter(parameter_name).value)
    if len(values) != length:
        raise ValueError(f"{parameter_name} must contain {length} values")
    return values


def load_gps_interface():
    try:
        from car_interfaces.msg import GpsInterface
    except ImportError as exc:  # pragma: no cover - exercised only in ROS runtime
        raise RuntimeError(
            "car_interfaces.msg.GpsInterface is required. "
            "Source the AutoDrive workspace before running gps_imu_bridge_node."
        ) from exc
    return GpsInterface


class GpsImuBridgeNode(Node):
    def __init__(self):
        super().__init__("gps_imu_bridge_node")
        self.declare_parameter("input_topic", "/gps_data")
        self.declare_parameter("output_topic", "/imu")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("angles_in_degrees", True)
        self.declare_parameter("angular_velocity_in_degrees", True)
        self.declare_parameter("angular_velocity_axes", ["x", "y", "z"])
        self.declare_parameter("angular_velocity_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("linear_acceleration_axes", ["x", "y", "z"])
        self.declare_parameter("linear_acceleration_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("linear_acceleration_scale", 9.8)
        self.declare_parameter(
            "orientation_covariance",
            [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.1],
        )
        self.declare_parameter(
            "angular_velocity_covariance",
            [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02],
        )
        self.declare_parameter(
            "linear_acceleration_covariance",
            [0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5],
        )

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.angles_in_degrees = bool(self.get_parameter("angles_in_degrees").value)
        self.angular_velocity_in_degrees = bool(
            self.get_parameter("angular_velocity_in_degrees").value
        )
        self.angular_velocity_axes = _as_string_tuple(self, "angular_velocity_axes", 3)
        self.angular_velocity_signs = _as_float_tuple(self, "angular_velocity_signs", 3)
        self.linear_acceleration_axes = _as_string_tuple(
            self, "linear_acceleration_axes", 3
        )
        self.linear_acceleration_signs = _as_float_tuple(
            self, "linear_acceleration_signs", 3
        )
        self.linear_acceleration_scale = float(
            self.get_parameter("linear_acceleration_scale").value
        )
        self.orientation_covariance = _as_float_tuple(
            self, "orientation_covariance", 9
        )
        self.angular_velocity_covariance = _as_float_tuple(
            self, "angular_velocity_covariance", 9
        )
        self.linear_acceleration_covariance = _as_float_tuple(
            self, "linear_acceleration_covariance", 9
        )

        gps_type = load_gps_interface()
        self.publisher = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(gps_type, input_topic, self.gps_callback, 10)

        self.get_logger().info(f"input_topic={input_topic}")
        self.get_logger().info(f"output_topic={output_topic}")
        self.get_logger().info(f"frame_id={self.frame_id}")
        self.get_logger().info(f"angles_in_degrees={self.angles_in_degrees}")
        self.get_logger().info(
            f"angular_velocity_in_degrees={self.angular_velocity_in_degrees}"
        )
        self.get_logger().info(f"angular_velocity_axes={self.angular_velocity_axes}")
        self.get_logger().info(f"angular_velocity_signs={self.angular_velocity_signs}")
        self.get_logger().info(
            f"linear_acceleration_axes={self.linear_acceleration_axes}"
        )
        self.get_logger().info(
            f"linear_acceleration_signs={self.linear_acceleration_signs}"
        )
        self.get_logger().info(
            f"linear_acceleration_scale={self.linear_acceleration_scale}"
        )

    def gps_callback(self, msg):
        imu_msg = Imu()
        fill_imu_message_from_gps(
            imu_msg,
            msg,
            frame_id=self.frame_id,
            angles_in_degrees=self.angles_in_degrees,
            angular_velocity_in_degrees=self.angular_velocity_in_degrees,
            angular_velocity_axes=self.angular_velocity_axes,
            angular_velocity_signs=self.angular_velocity_signs,
            linear_acceleration_axes=self.linear_acceleration_axes,
            linear_acceleration_signs=self.linear_acceleration_signs,
            linear_acceleration_scale=self.linear_acceleration_scale,
            orientation_covariance=self.orientation_covariance,
            angular_velocity_covariance=self.angular_velocity_covariance,
            linear_acceleration_covariance=self.linear_acceleration_covariance,
        )
        self.publisher.publish(imu_msg)


def main():
    rclpy.init()
    node = GpsImuBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
