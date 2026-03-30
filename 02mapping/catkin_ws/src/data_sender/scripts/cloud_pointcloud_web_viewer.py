#!/usr/bin/env python3
import json
import os
import threading
from copy import deepcopy
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from cloud_pointcloud_viewer_state import build_snapshot, empty_cloud_payload, resolve_lidar_topics


class MultiLatestCloudStore:
    def __init__(self, max_points, vehicle_topics):
        self._max_points = max_points
        self._vehicle_topics = {
            vehicle_id: dict(topic_map)
            for vehicle_id, topic_map in vehicle_topics.items()
        }
        self._lock = threading.Lock()
        self._payloads = {
            vehicle_id: {
                lidar_id: empty_cloud_payload() for lidar_id in topic_map
            }
            for vehicle_id, topic_map in self._vehicle_topics.items()
        }

    def update(self, vehicle_id, lidar_id, msg):
        total_points = max(1, msg.width * msg.height)
        stride = max(1, total_points // max(1, self._max_points))
        sampled_points = []
        min_xyz = [float("inf"), float("inf"), float("inf")]
        max_xyz = [float("-inf"), float("-inf"), float("-inf")]

        for index, point in enumerate(
            pc2.read_points(
                msg,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        ):
            if index % stride != 0:
                continue

            x, y, z, intensity = point
            sampled_points.append([x, y, z, intensity])

            min_xyz[0] = min(min_xyz[0], x)
            min_xyz[1] = min(min_xyz[1], y)
            min_xyz[2] = min(min_xyz[2], z)
            max_xyz[0] = max(max_xyz[0], x)
            max_xyz[1] = max(max_xyz[1], y)
            max_xyz[2] = max(max_xyz[2], z)

        bounds = None
        if sampled_points:
            bounds = {
                "min": {"x": min_xyz[0], "y": min_xyz[1], "z": min_xyz[2]},
                "max": {"x": max_xyz[0], "y": max_xyz[1], "z": max_xyz[2]},
            }

        payload = {
            "frame_id": msg.header.frame_id,
            "stamp": {"secs": msg.header.stamp.secs, "nsecs": msg.header.stamp.nsecs},
            "point_count": len(sampled_points),
            "points": sampled_points,
            "bounds": bounds,
        }

        with self._lock:
            self._payloads.setdefault(vehicle_id, {})[lidar_id] = payload

    def snapshot(self):
        with self._lock:
            return build_snapshot(self._vehicle_topics, deepcopy(self._payloads))


def make_handler(store, html_path):
    class RequestHandler(BaseHTTPRequestHandler):
        def _write_json(self, payload):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _write_html(self):
            with open(html_path, "rb") as handle:
                body = handle.read()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._write_html()
                return
            if self.path == "/api/cloud":
                self._write_json(store.snapshot())
                return

            self.send_error(HTTPStatus.NOT_FOUND)

        def log_message(self, format_string, *args):
            rospy.loginfo("viewer_http " + format_string, *args)

    return RequestHandler


def main():
    rospy.init_node("cloud_pointcloud_web_viewer", anonymous=False, disable_signals=True)

    topic_name = rospy.get_param("~topic_name", "")
    topic_names = rospy.get_param("~topic_names", "")
    bind_host = rospy.get_param("~bind_host", "0.0.0.0")
    bind_port = int(rospy.get_param("~bind_port", 18080))
    max_points = int(rospy.get_param("~max_points", 8000))

    script_dir = os.path.dirname(os.path.abspath(__file__))
    html_path = os.path.normpath(os.path.join(script_dir, "..", "web", "cloud_pointcloud_viewer.html"))

    vehicle_topics = resolve_lidar_topics(topic_names or topic_name)
    store = MultiLatestCloudStore(max_points=max_points, vehicle_topics=vehicle_topics)
    for vehicle_id, lidar_map in vehicle_topics.items():
        for lidar_id, resolved_topic in lidar_map.items():
            rospy.Subscriber(
                resolved_topic,
                PointCloud2,
                lambda msg, vehicle_id=vehicle_id, lidar_id=lidar_id: store.update(
                    vehicle_id, lidar_id, msg
                ),
                queue_size=1,
            )

    server = ThreadingHTTPServer((bind_host, bind_port), make_handler(store, html_path))
    rospy.loginfo(
        "cloud_pointcloud_web_viewer serving %s on http://%s:%d",
        json.dumps(vehicle_topics, ensure_ascii=False),
        bind_host,
        bind_port,
    )

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
