from data_sender_ros2.topic_config import (
    build_lidar_topic_map,
    resolve_configured_topics,
)


def test_resolve_configured_topics_prefers_multi_topic_parameter():
    configured = resolve_configured_topics(
        cloud_topic="/forward/rslidar_points",
        cloud_topics=["/left/rslidar_points", "/right/rslidar_points"],
    )

    assert configured == ["/left/rslidar_points", "/right/rslidar_points"]


def test_resolve_configured_topics_falls_back_to_single_topic():
    configured = resolve_configured_topics(
        cloud_topic="/forward/rslidar_points",
        cloud_topics=[],
    )

    assert configured == ["/forward/rslidar_points"]


def test_build_lidar_topic_map_uses_stable_lidar_ids():
    lidar_topics = build_lidar_topic_map(
        ["/forward/rslidar_points", "/left/rslidar_points", "/right/rslidar_points"]
    )

    assert lidar_topics == {
        "forward": "/forward/rslidar_points",
        "left": "/left/rslidar_points",
        "right": "/right/rslidar_points",
    }


def test_build_lidar_topic_map_rejects_duplicate_lidar_ids():
    try:
        build_lidar_topic_map(
            ["/forward/rslidar_points", "/forward/secondary_points"]
        )
    except ValueError as exc:
        assert "duplicate" in str(exc)
    else:
        raise AssertionError("expected duplicate lidar id error")
