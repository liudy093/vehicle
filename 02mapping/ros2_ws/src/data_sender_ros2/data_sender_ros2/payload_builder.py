"""Helpers for converting ROS messages into HTTP-friendly payloads."""

import base64


def serialize_fields(fields):
    """Convert PointField-like objects into plain dictionaries."""
    return [
        {
            "name": field.name,
            "offset": field.offset,
            "datatype": field.datatype,
            "count": field.count,
        }
        for field in fields
    ]


def serialize_pointcloud(cloud_msg):
    """Convert a PointCloud2-like message into a JSON-friendly structure."""
    return {
        "header": {
            "stamp": {
                "sec": cloud_msg.header.stamp.sec,
                "nanosec": cloud_msg.header.stamp.nanosec,
            },
            "frame_id": cloud_msg.header.frame_id,
        },
        "shape": {
            "height": cloud_msg.height,
            "width": cloud_msg.width,
        },
        "fields": serialize_fields(cloud_msg.fields),
        "layout": {
            "is_bigendian": cloud_msg.is_bigendian,
            "point_step": cloud_msg.point_step,
            "row_step": cloud_msg.row_step,
            "is_dense": cloud_msg.is_dense,
        },
        "data_b64": base64.b64encode(bytes(cloud_msg.data)).decode("ascii"),
    }


def build_cloud_payload(vehicle_id, lidar_id, cloud_msg, gps_pose, cloud_topic):
    """Build a JSON-serializable payload from PointCloud2-like input."""
    payload = serialize_pointcloud(cloud_msg)
    payload.update(
        {
            "vehicle_id": vehicle_id,
            "lidar_id": lidar_id,
            "cloud_topic": cloud_topic,
            "gps": {
                "x": gps_pose.x,
                "y": gps_pose.y,
                "theta": gps_pose.theta,
            },
        }
    )
    return payload


def build_fastlio_payload(
    vehicle_id,
    cloud_msg,
    odom_msg,
    cloud_topic,
    odom_topic,
):
    """Build a payload that carries FAST-LIO odom and registered cloud."""
    payload = serialize_pointcloud(cloud_msg)
    payload.update(
        {
            "vehicle_id": vehicle_id,
            "cloud_topic": cloud_topic,
            "odom_topic": odom_topic,
            "odom": {
                "header": {
                    "stamp": {
                        "sec": odom_msg.header.stamp.sec,
                        "nanosec": odom_msg.header.stamp.nanosec,
                    },
                    "frame_id": odom_msg.header.frame_id,
                },
                "child_frame_id": odom_msg.child_frame_id,
                "pose": {
                    "position": {
                        "x": odom_msg.pose.pose.position.x,
                        "y": odom_msg.pose.pose.position.y,
                        "z": odom_msg.pose.pose.position.z,
                    },
                    "orientation": {
                        "x": odom_msg.pose.pose.orientation.x,
                        "y": odom_msg.pose.pose.orientation.y,
                        "z": odom_msg.pose.pose.orientation.z,
                        "w": odom_msg.pose.pose.orientation.w,
                    },
                },
            },
        }
    )
    return payload
