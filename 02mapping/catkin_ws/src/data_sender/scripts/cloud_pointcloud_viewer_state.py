DEFAULT_TOPIC_ORDER = ["fused", "global_cloud", "global_map", "global_map_optimized"]

DEFAULT_VEHICLE_TOPICS = {
    "vehicle1": {
        "fused": "/vehicle1/cloud_sent/fused",
        "global_cloud": "/vehicle1/global_cloud",
    },
    "vehicle2": {
        "fused": "/vehicle2/cloud_sent/fused",
        "global_cloud": "/vehicle2/global_cloud",
    },
    "global": {
        "global_map": "/global_map",
        "global_map_optimized": "/global_map_optimized",
    },
}


def infer_lidar_id(topic_name):
    normalized = (topic_name or "").strip().rstrip("/")
    if not normalized:
        return "cloud"
    return normalized.split("/")[-1] or "cloud"


def empty_cloud_payload():
    return {
        "frame_id": "",
        "stamp": {"secs": 0, "nsecs": 0},
        "point_count": 0,
        "points": [],
        "bounds": None,
    }


def _clone_default_topics():
    return {
        vehicle_id: dict(topic_map)
        for vehicle_id, topic_map in DEFAULT_VEHICLE_TOPICS.items()
    }


def _normalize_vehicle_lidar_key(key, topic_name):
    normalized_key = (key or "").strip().strip("/")
    normalized_topic = (topic_name or "").strip()

    if "/" in normalized_key:
        vehicle_id, lidar_id = normalized_key.split("/", 1)
        return vehicle_id.strip(), lidar_id.strip()

    normalized_topic_parts = normalized_topic.strip("/").split("/")
    if len(normalized_topic_parts) >= 3:
        return normalized_topic_parts[0], normalized_topic_parts[-1]

    return "default", normalized_key or infer_lidar_id(normalized_topic)


def resolve_lidar_topics(raw_topics):
    raw_topics = (raw_topics or "").strip()
    if not raw_topics:
        return _clone_default_topics()

    vehicle_topics = {}
    for entry in raw_topics.split(","):
        item = entry.strip()
        if not item:
            continue

        if "=" in item:
            key, topic_name = item.split("=", 1)
            vehicle_id, lidar_id = _normalize_vehicle_lidar_key(key, topic_name)
            vehicle_topics.setdefault(vehicle_id, {})[lidar_id] = topic_name.strip()
            continue

        topic_name = item
        normalized = topic_name.strip("/").split("/")
        if len(normalized) >= 3:
            vehicle_id = normalized[0]
            lidar_id = normalized[-1]
        else:
            vehicle_id = "default"
            lidar_id = infer_lidar_id(topic_name)
        vehicle_topics.setdefault(vehicle_id, {})[lidar_id] = topic_name

    return vehicle_topics or _clone_default_topics()


def build_snapshot(vehicle_topics, payloads=None):
    payloads = payloads or {}
    vehicle_order = list(vehicle_topics.keys())

    topic_order = []
    for lidar_id in DEFAULT_TOPIC_ORDER:
        if any(lidar_id in topic_map for topic_map in vehicle_topics.values()):
            topic_order.append(lidar_id)
    for topic_map in vehicle_topics.values():
        for lidar_id in topic_map.keys():
            if lidar_id not in topic_order:
                topic_order.append(lidar_id)

    topics_snapshot = {}
    clouds_snapshot = {}
    for vehicle_id in vehicle_order:
        topics_snapshot[vehicle_id] = {}
        clouds_snapshot[vehicle_id] = {}
        for lidar_id in topic_order:
            if lidar_id in vehicle_topics[vehicle_id]:
                topics_snapshot[vehicle_id][lidar_id] = vehicle_topics[vehicle_id][lidar_id]
            clouds_snapshot[vehicle_id][lidar_id] = dict(
                payloads.get(vehicle_id, {}).get(lidar_id, empty_cloud_payload())
            )

    return {
        "vehicle_order": vehicle_order,
        "topic_order": topic_order,
        "topics": topics_snapshot,
        "clouds": clouds_snapshot,
    }
