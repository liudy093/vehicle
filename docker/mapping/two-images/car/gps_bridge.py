#!/usr/bin/env python3

import argparse
import math
import sys

import pyproj
import rospy
import serial
from geometry_msgs.msg import Pose2D


class GPSToXYConverter:
    def __init__(self, lat0: float, lon0: float):
        self.projection = pyproj.Proj(
            f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 "
            "+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
        )

    def gps_to_xy(self, lat: float, lon: float):
        return self.projection(lon, lat)


def parse_args():
    parser = argparse.ArgumentParser(description="Container-only GPS bridge for /gps")
    parser.add_argument("--serial-port", default="/dev/ttyUSB1")
    parser.add_argument("--baud-rate", type=int, default=115200)
    parser.add_argument("--lat0", type=float, required=True)
    parser.add_argument("--lon0", type=float, required=True)
    parser.add_argument("--topic", default="/gps")
    parser.add_argument("--rate", type=float, default=50.0)
    return parser.parse_args()


def parse_line(raw: str):
    parts = raw.strip().split(",")
    if len(parts) < 6:
        raise ValueError("not enough fields")
    lon = float(parts[2])
    lat = float(parts[3])
    theta = float(parts[5])
    return lat, lon, theta


def heading_deg_to_rad(theta_deg: float) -> float:
    if theta_deg - 90 > 180:
        theta_deg = 360 - theta_deg + 90
    else:
        theta_deg = -theta_deg + 90
    return theta_deg * math.pi / 180.0


def main():
    args = parse_args()

    try:
        ser = serial.Serial(args.serial_port, args.baud_rate, timeout=1.0)
    except serial.SerialException as exc:
        print(f"Failed to open serial port {args.serial_port}: {exc}", file=sys.stderr)
        return 1

    rospy.init_node("container_gps_bridge", anonymous=False)
    pub = rospy.Publisher(args.topic, Pose2D, queue_size=10)
    rate = rospy.Rate(args.rate)
    converter = GPSToXYConverter(args.lat0, args.lon0)

    while not rospy.is_shutdown():
        try:
            raw = ser.readline().decode("utf-8", errors="ignore")
            if not raw:
                rate.sleep()
                continue
            lat, lon, theta_deg = parse_line(raw)
            x, y = converter.gps_to_xy(lat, lon)
            msg = Pose2D()
            msg.x = x
            msg.y = y
            msg.theta = heading_deg_to_rad(theta_deg)
            pub.publish(msg)
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, f"GPS bridge parse error: {exc}")
        rate.sleep()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
