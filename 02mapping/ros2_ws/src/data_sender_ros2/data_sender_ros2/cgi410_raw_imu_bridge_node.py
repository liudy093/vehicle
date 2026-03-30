from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace


@dataclass(frozen=True)
class ParsedGTIMU:
    gps_week: int
    gps_seconds: float
    angular_velocity: SimpleNamespace
    linear_acceleration: SimpleNamespace
    temperature_celsius: float


def _parse_nmea_checksum(line: str) -> tuple[str, int] | None:
    sentence = line.strip()
    if not sentence.startswith("$") or "*" not in sentence:
        return None

    payload, checksum_text = sentence[1:].split("*", 1)
    try:
        expected_checksum = int(checksum_text[:2], 16)
    except ValueError:
        return None

    actual_checksum = 0
    for char in payload:
        actual_checksum ^= ord(char)

    if actual_checksum != expected_checksum:
        return None

    return payload, expected_checksum


def parse_gtimu_sentence(line: str) -> ParsedGTIMU | None:
    parsed = _parse_nmea_checksum(line)
    if parsed is None:
        return None

    payload, _ = parsed
    fields = payload.split(",")
    if len(fields) != 10 or fields[0] != "GTIMU":
        return None

    try:
        gps_week = int(fields[1])
        gps_seconds = float(fields[2])
        wx = float(fields[3])
        wy = float(fields[4])
        wz = float(fields[5])
        ax = float(fields[6])
        ay = float(fields[7])
        az = float(fields[8])
        temperature_celsius = float(fields[9])
    except ValueError:
        return None

    return ParsedGTIMU(
        gps_week=gps_week,
        gps_seconds=gps_seconds,
        angular_velocity=SimpleNamespace(x=wx, y=wy, z=wz),
        linear_acceleration=SimpleNamespace(x=ax, y=ay, z=az),
        temperature_celsius=temperature_celsius,
    )


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


def _pick_components(values, axes, signs):
    result = {}
    for target_axis, source_axis, sign in zip(("x", "y", "z"), axes, signs):
        result[target_axis] = getattr(values, source_axis) * sign
    return result


class Cgi410RawImuBridgeNode:
    def __init__(self):
        import serial
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Imu

        class _NodeImpl(Node):
            def __init__(self_outer):
                super().__init__("cgi410_raw_imu_bridge_node")
                self_outer.declare_parameter("port", "/dev/ttyUART_232_C")
                self_outer.declare_parameter("baudrate", 230400)
                self_outer.declare_parameter("frame_id", "imu_link")
                self_outer.declare_parameter("output_topic", "/imu_raw")
                self_outer.declare_parameter("angular_velocity_axes", ["x", "y", "z"])
                self_outer.declare_parameter("angular_velocity_signs", [1.0, 1.0, 1.0])
                self_outer.declare_parameter("linear_acceleration_axes", ["x", "y", "z"])
                self_outer.declare_parameter("linear_acceleration_signs", [1.0, 1.0, 1.0])
                self_outer.declare_parameter(
                    "angular_velocity_covariance",
                    [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02],
                )
                self_outer.declare_parameter(
                    "linear_acceleration_covariance",
                    [0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5],
                )

                self_outer.frame_id = self_outer.get_parameter("frame_id").value
                self_outer.angular_velocity_axes = _as_string_tuple(
                    self_outer, "angular_velocity_axes", 3
                )
                self_outer.angular_velocity_signs = _as_float_tuple(
                    self_outer, "angular_velocity_signs", 3
                )
                self_outer.linear_acceleration_axes = _as_string_tuple(
                    self_outer, "linear_acceleration_axes", 3
                )
                self_outer.linear_acceleration_signs = _as_float_tuple(
                    self_outer, "linear_acceleration_signs", 3
                )
                self_outer.angular_velocity_covariance = list(
                    _as_float_tuple(self_outer, "angular_velocity_covariance", 9)
                )
                self_outer.linear_acceleration_covariance = list(
                    _as_float_tuple(self_outer, "linear_acceleration_covariance", 9)
                )

                port = self_outer.get_parameter("port").value
                baudrate = int(self_outer.get_parameter("baudrate").value)
                output_topic = self_outer.get_parameter("output_topic").value
                self_outer.publisher = self_outer.create_publisher(Imu, output_topic, 20)
                self_outer.serial = serial.Serial(port, baudrate, timeout=0.1)
                self_outer.create_timer(0.005, self_outer._poll_serial)
                self_outer.get_logger().info(f"port={port}")
                self_outer.get_logger().info(f"baudrate={baudrate}")
                self_outer.get_logger().info(f"output_topic={output_topic}")

            def _poll_serial(self_outer):
                while True:
                    raw = self_outer.serial.readline()
                    if not raw:
                        return
                    parsed = parse_gtimu_sentence(raw.decode("utf-8", errors="ignore"))
                    if parsed is None:
                        continue

                    msg = Imu()
                    msg.header.frame_id = self_outer.frame_id
                    msg.header.stamp = self_outer.get_clock().now().to_msg()
                    msg.orientation_covariance[0] = -1.0

                    angular_velocity = _pick_components(
                        parsed.angular_velocity,
                        self_outer.angular_velocity_axes,
                        self_outer.angular_velocity_signs,
                    )
                    linear_acceleration = _pick_components(
                        parsed.linear_acceleration,
                        self_outer.linear_acceleration_axes,
                        self_outer.linear_acceleration_signs,
                    )

                    msg.angular_velocity.x = angular_velocity["x"]
                    msg.angular_velocity.y = angular_velocity["y"]
                    msg.angular_velocity.z = angular_velocity["z"]
                    msg.linear_acceleration.x = linear_acceleration["x"]
                    msg.linear_acceleration.y = linear_acceleration["y"]
                    msg.linear_acceleration.z = linear_acceleration["z"]
                    msg.angular_velocity_covariance = (
                        self_outer.angular_velocity_covariance
                    )
                    msg.linear_acceleration_covariance = (
                        self_outer.linear_acceleration_covariance
                    )
                    self_outer.publisher.publish(msg)

        self.rclpy = rclpy
        self.node = _NodeImpl()

    def spin(self):
        self.rclpy.spin(self.node)

    def shutdown(self):
        try:
            self.node.serial.close()
        except Exception:
            pass
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    import rclpy

    rclpy.init()
    wrapper = Cgi410RawImuBridgeNode()
    try:
        wrapper.spin()
    finally:
        wrapper.shutdown()
