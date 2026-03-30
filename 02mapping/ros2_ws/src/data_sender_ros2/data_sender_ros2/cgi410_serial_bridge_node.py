from __future__ import annotations

import re
import sys
import time
from dataclasses import dataclass
from types import SimpleNamespace

from data_sender_ros2.autodrive_bridge import fill_imu_message_from_gps
from data_sender_ros2.cgi410_raw_imu_bridge_node import parse_gtimu_sentence


@dataclass(frozen=True)
class ParsedGPCHC:
    yaw: float
    pitch: float
    roll: float
    wx: float
    wy: float
    wz: float
    ax: float
    ay: float
    az: float
    latitude: float
    longitude: float
    height: float
    eastvelocity: float
    northvelocity: float
    skyvelocity: float


NMEA_PATTERN = re.compile(r"\$(?:GTIMU|GPCHC),[^\r\n$]*\*[0-9A-Fa-f]{2}")


def extract_nmea_sentences(buffer: str) -> tuple[list[str], str]:
    sentences = []
    last_end = 0
    for match in NMEA_PATTERN.finditer(buffer):
        sentences.append(match.group(0))
        last_end = match.end()
    remainder = buffer[last_end:]
    if "$" in remainder:
        remainder = remainder[remainder.rfind("$") :]
    elif len(remainder) > 1024:
        remainder = ""
    return sentences, remainder


def parse_gpchc_sentence(line: str) -> ParsedGPCHC | None:
    sentence = line.strip()
    if not sentence.startswith("$GPCHC,"):
        return None

    if "*" in sentence:
        payload = sentence[1:].split("*", 1)[0]
    else:
        payload = sentence[1:]

    fields = payload.split(",")
    if len(fields) != 24 or fields[0] != "GPCHC":
        return None

    try:
        heading = float(fields[3])
        yaw = 90.0 - heading
        if yaw >= 360.0:
            yaw -= 360.0
        if yaw < 0.0:
            yaw += 360.0

        return ParsedGPCHC(
            yaw=yaw,
            pitch=float(fields[4]),
            roll=float(fields[5]),
            wx=float(fields[6]),
            wy=float(fields[7]),
            wz=float(fields[8]),
            ax=float(fields[9]) * 9.8,
            ay=float(fields[10]) * 9.8,
            az=float(fields[11]) * 9.8,
            latitude=float(fields[12]),
            longitude=float(fields[13]),
            height=float(fields[14]),
            eastvelocity=float(fields[15]),
            northvelocity=float(fields[16]),
            skyvelocity=float(fields[17]),
        )
    except ValueError:
        return None


def load_runtime_types():
    try:
        from car_interfaces.msg import GpsInterface
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "car_interfaces.msg.GpsInterface is required. "
            "Source the AutoDrive workspace before running cgi410_serial_bridge_node."
        ) from exc

    try:
        import tjitools
    except ImportError as exc:  # pragma: no cover
        utils_path = "/home/nvidia/AutoDrive/src/utils"
        if utils_path not in sys.path:
            sys.path.append(utils_path)
        try:
            import tjitools
        except ImportError as inner_exc:
            raise RuntimeError(
                "tjitools is required. Source /home/nvidia/AutoDrive and ensure src/utils is on PYTHONPATH."
            ) from inner_exc

    return GpsInterface, tjitools


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


def fill_imu_message_from_gtimu(
    imu_msg,
    latest_gps,
    parsed_gtimu,
    *,
    frame_id="imu_link",
    angular_velocity_axes=("x", "y", "z"),
    angular_velocity_signs=(1.0, 1.0, 1.0),
    linear_acceleration_axes=("x", "y", "z"),
    linear_acceleration_signs=(1.0, 1.0, 1.0),
    orientation_covariance=(0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.1),
    angular_velocity_covariance=(0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02),
    linear_acceleration_covariance=(0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5),
):
    gps_like = SimpleNamespace(
        timestamp=latest_gps.timestamp,
        yaw=latest_gps.yaw,
        pitch=latest_gps.pitch,
        roll=latest_gps.roll,
        wx=parsed_gtimu.angular_velocity.x,
        wy=parsed_gtimu.angular_velocity.y,
        wz=parsed_gtimu.angular_velocity.z,
        ax=parsed_gtimu.linear_acceleration.x,
        ay=parsed_gtimu.linear_acceleration.y,
        az=parsed_gtimu.linear_acceleration.z,
    )
    fill_imu_message_from_gps(
        imu_msg,
        gps_like,
        frame_id=frame_id,
        angles_in_degrees=True,
        angular_velocity_in_degrees=True,
        angular_velocity_axes=angular_velocity_axes,
        angular_velocity_signs=angular_velocity_signs,
        linear_acceleration_axes=linear_acceleration_axes,
        linear_acceleration_signs=linear_acceleration_signs,
        linear_acceleration_scale=9.8,
        orientation_covariance=orientation_covariance,
        angular_velocity_covariance=angular_velocity_covariance,
        linear_acceleration_covariance=linear_acceleration_covariance,
    )


class Cgi410SerialBridgeNode:
    def __init__(self):
        import numpy as np
        import rclpy
        import serial
        from rclpy.node import Node
        from sensor_msgs.msg import Imu

        GpsInterface, tjitools = load_runtime_types()

        class _NodeImpl(Node):
            def __init__(self_outer):
                super().__init__("cgi410_serial_bridge_node")
                self_outer.declare_parameter("port", "/dev/ttyUART_232_C")
                self_outer.declare_parameter("baudrate", 230400)
                self_outer.declare_parameter("gps_output_topic", "/gps_data")
                self_outer.declare_parameter("imu_output_topic", "/imu_raw")
                self_outer.declare_parameter("frame_id", "imu_link")
                self_outer.declare_parameter("angular_velocity_axes", ["x", "y", "z"])
                self_outer.declare_parameter("angular_velocity_signs", [1.0, 1.0, 1.0])
                self_outer.declare_parameter("linear_acceleration_axes", ["x", "y", "z"])
                self_outer.declare_parameter("linear_acceleration_signs", [1.0, 1.0, 1.0])
                self_outer.declare_parameter(
                    "orientation_covariance",
                    [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.1],
                )
                self_outer.declare_parameter(
                    "angular_velocity_covariance",
                    [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02],
                )
                self_outer.declare_parameter(
                    "linear_acceleration_covariance",
                    [0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5],
                )

                port = self_outer.get_parameter("port").value
                baudrate = int(self_outer.get_parameter("baudrate").value)
                gps_output_topic = self_outer.get_parameter("gps_output_topic").value
                imu_output_topic = self_outer.get_parameter("imu_output_topic").value
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
                self_outer.orientation_covariance = _as_float_tuple(
                    self_outer, "orientation_covariance", 9
                )
                self_outer.angular_velocity_covariance = _as_float_tuple(
                    self_outer, "angular_velocity_covariance", 9
                )
                self_outer.linear_acceleration_covariance = _as_float_tuple(
                    self_outer, "linear_acceleration_covariance", 9
                )

                self_outer.gps_publisher = self_outer.create_publisher(
                    GpsInterface, gps_output_topic, 20
                )
                self_outer.imu_publisher = self_outer.create_publisher(
                    Imu, imu_output_topic, 50
                )
                self_outer.serial = serial.Serial(port, baudrate, timeout=0.1)
                self_outer.buffer = ""
                self_outer.latest_gps = None
                self_outer.latest_gtimu = None
                self_outer.tjitools = tjitools
                self_outer.np = np
                self_outer.create_timer(0.01, self_outer._poll_serial)

                self_outer.get_logger().info(f"port={port}")
                self_outer.get_logger().info(f"baudrate={baudrate}")
                self_outer.get_logger().info(f"gps_output_topic={gps_output_topic}")
                self_outer.get_logger().info(f"imu_output_topic={imu_output_topic}")

            def _publish_gps(self_outer, parsed_gpchc):
                gps_msg = GpsInterface()
                gps_msg.timestamp = time.time()
                gps_msg.id = 0x01
                gps_msg.yaw = parsed_gpchc.yaw
                gps_msg.pitch = parsed_gpchc.pitch
                gps_msg.roll = parsed_gpchc.roll
                gps_msg.wx = parsed_gpchc.wx
                gps_msg.wy = parsed_gpchc.wy
                gps_msg.wz = parsed_gpchc.wz
                gps_msg.ax = parsed_gpchc.ax
                gps_msg.ay = parsed_gpchc.ay
                gps_msg.az = parsed_gpchc.az
                gps_msg.latitude = parsed_gpchc.latitude
                gps_msg.longitude = parsed_gpchc.longitude
                gps_msg.height = parsed_gpchc.height
                gps_msg.eastvelocity = parsed_gpchc.eastvelocity
                gps_msg.northvelocity = parsed_gpchc.northvelocity
                gps_msg.skyvelocity = parsed_gpchc.skyvelocity
                enu = self_outer.tjitools.gps_to_enu(
                    self_outer.np.array([gps_msg.longitude, gps_msg.latitude, 0.0])
                )
                gps_msg.x = float(enu[0])
                gps_msg.y = float(enu[1])
                gps_msg.process_time = 0.0
                self_outer.latest_gps = gps_msg
                self_outer.gps_publisher.publish(gps_msg)

            def _publish_imu(self_outer, parsed_gtimu):
                if self_outer.latest_gps is None:
                    return
                imu_msg = Imu()
                fill_imu_message_from_gtimu(
                    imu_msg,
                    self_outer.latest_gps,
                    parsed_gtimu,
                    frame_id=self_outer.frame_id,
                    angular_velocity_axes=self_outer.angular_velocity_axes,
                    angular_velocity_signs=self_outer.angular_velocity_signs,
                    linear_acceleration_axes=self_outer.linear_acceleration_axes,
                    linear_acceleration_signs=self_outer.linear_acceleration_signs,
                    orientation_covariance=self_outer.orientation_covariance,
                    angular_velocity_covariance=self_outer.angular_velocity_covariance,
                    linear_acceleration_covariance=self_outer.linear_acceleration_covariance,
                )
                self_outer.imu_publisher.publish(imu_msg)

            def _poll_serial(self_outer):
                chunk = self_outer.serial.read(self_outer.serial.in_waiting or 1)
                if not chunk:
                    return
                self_outer.buffer += chunk.decode("utf-8", errors="ignore")
                sentences, self_outer.buffer = extract_nmea_sentences(self_outer.buffer)
                for sentence in sentences:
                    parsed_gpchc = parse_gpchc_sentence(sentence)
                    if parsed_gpchc is not None:
                        self_outer._publish_gps(parsed_gpchc)
                        continue

                    parsed_gtimu = parse_gtimu_sentence(sentence)
                    if parsed_gtimu is not None:
                        self_outer.latest_gtimu = parsed_gtimu
                        self_outer._publish_imu(parsed_gtimu)

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
    wrapper = Cgi410SerialBridgeNode()
    try:
        wrapper.spin()
    finally:
        wrapper.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
