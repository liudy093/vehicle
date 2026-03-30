import base64
from types import SimpleNamespace

from data_sender_ros2.payload_builder import build_cloud_payload, build_fastlio_payload
from data_sender_ros2.topic_config import derive_lidar_id_from_topic


def test_build_cloud_payload_serializes_pointcloud_and_gps():
    cloud_msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=123, nanosec=456),
            frame_id="rslidar",
        ),
        height=2,
        width=3,
        fields=[
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            SimpleNamespace(name="intensity", offset=16, datatype=7, count=1),
        ],
        is_bigendian=False,
        point_step=26,
        row_step=78,
        data=bytes([1, 2, 3, 4]),
        is_dense=False,
    )
    gps_pose = SimpleNamespace(x=1.5, y=-2.5, theta=90.0)

    payload = build_cloud_payload(
        vehicle_id="vehicle2",
        lidar_id="forward",
        cloud_msg=cloud_msg,
        gps_pose=gps_pose,
        cloud_topic="/forward/rslidar_points",
    )

    assert payload["vehicle_id"] == "vehicle2"
    assert payload["lidar_id"] == "forward"
    assert payload["cloud_topic"] == "/forward/rslidar_points"
    assert payload["header"]["frame_id"] == "rslidar"
    assert payload["header"]["stamp"] == {"sec": 123, "nanosec": 456}
    assert payload["shape"] == {"height": 2, "width": 3}
    assert payload["layout"]["point_step"] == 26
    assert payload["layout"]["row_step"] == 78
    assert payload["layout"]["is_bigendian"] is False
    assert payload["layout"]["is_dense"] is False
    assert payload["fields"] == [
        {"name": "x", "offset": 0, "datatype": 7, "count": 1},
        {"name": "intensity", "offset": 16, "datatype": 7, "count": 1},
    ]
    assert payload["gps"] == {"x": 1.5, "y": -2.5, "theta": 90.0}
    assert payload["data_b64"] == base64.b64encode(bytes([1, 2, 3, 4])).decode("ascii")


def test_derive_lidar_id_from_topic_uses_first_path_segment():
    assert derive_lidar_id_from_topic("/forward/rslidar_points") == "forward"
    assert derive_lidar_id_from_topic("/left/rslidar_points") == "left"
    assert derive_lidar_id_from_topic("/right/rslidar_points") == "right"


def test_derive_lidar_id_from_topic_rejects_invalid_paths():
    for topic in ("", "/", "rslidar_points", "/rslidar_points"):
        try:
            derive_lidar_id_from_topic(topic)
        except ValueError as exc:
            assert "lidar id" in str(exc)
        else:
            raise AssertionError(f"expected ValueError for topic={topic!r}")


def test_build_fastlio_payload_serializes_registered_cloud_and_odom():
    cloud_msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=321, nanosec=654),
            frame_id="body",
        ),
        height=1,
        width=2,
        fields=[
            SimpleNamespace(name="x", offset=0, datatype=7, count=1),
        ],
        is_bigendian=False,
        point_step=16,
        row_step=32,
        data=bytes([9, 8, 7, 6]),
        is_dense=True,
    )
    odom_msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=111, nanosec=222),
            frame_id="camera_init",
        ),
        child_frame_id="body",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.8660254),
            )
        ),
    )

    payload = build_fastlio_payload(
        vehicle_id="vehicle2",
        cloud_msg=cloud_msg,
        odom_msg=odom_msg,
        cloud_topic="/cloud_registered",
        odom_topic="/Odometry",
    )

    assert payload["vehicle_id"] == "vehicle2"
    assert payload["cloud_topic"] == "/cloud_registered"
    assert payload["odom_topic"] == "/Odometry"
    assert payload["header"]["frame_id"] == "body"
    assert payload["data_b64"] == base64.b64encode(bytes([9, 8, 7, 6])).decode("ascii")
    assert payload["odom"]["header"]["frame_id"] == "camera_init"
    assert payload["odom"]["child_frame_id"] == "body"
    assert payload["odom"]["pose"]["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert payload["odom"]["pose"]["orientation"] == {
        "x": 0.0,
        "y": 0.0,
        "z": 0.5,
        "w": 0.8660254,
    }
