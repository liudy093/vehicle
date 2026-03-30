def derive_lidar_id_from_topic(topic):
    normalized = (topic or "").strip()
    if not normalized.startswith("/"):
        raise ValueError(f"unable to derive lidar id from topic: {topic!r}")

    parts = [part for part in normalized.split("/") if part]
    if len(parts) < 2:
        raise ValueError(f"unable to derive lidar id from topic: {topic!r}")

    return parts[0]


def resolve_configured_topics(cloud_topic, cloud_topics):
    if isinstance(cloud_topics, str):
        cloud_topics = [part.strip() for part in cloud_topics.split(",")]
    filtered_topics = [topic for topic in cloud_topics if topic]
    if filtered_topics:
        return filtered_topics
    return [cloud_topic]


def build_lidar_topic_map(cloud_topics):
    lidar_topics = {}
    for topic in cloud_topics:
        lidar_id = derive_lidar_id_from_topic(topic)
        if lidar_id in lidar_topics:
            raise ValueError(f"duplicate lidar id configured: {lidar_id}")
        lidar_topics[lidar_id] = topic
    return lidar_topics
