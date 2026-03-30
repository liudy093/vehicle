"""Conversion helpers for bridging AutoDrive messages to standard ROS types."""

import math


def _coerce_vector_config(values, length, cast=float):
    coerced = tuple(cast(value) for value in values)
    if len(coerced) != length:
        raise ValueError(f"Expected {length} values, got {len(coerced)}")
    return coerced


def _resolve_axis_triplet(components, axes, signs):
    resolved = []
    for axis_name, sign in zip(axes, signs):
        resolved.append(float(components[axis_name]) * float(sign))
    return tuple(resolved)


def _degrees_to_radians_if_needed(value, enabled):
    if enabled:
        return math.radians(value)
    return value


def normalize_heading_radians(theta):
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(theta), math.cos(theta))


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles in radians to quaternion components."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def fill_pose2d_message_from_gps(pose_msg, gps_msg, yaw_in_degrees=True):
    """Populate a Pose2D-like object from an AutoDrive GPS-like object."""
    yaw = _degrees_to_radians_if_needed(gps_msg.yaw, yaw_in_degrees)
    pose_msg.x = float(gps_msg.x)
    pose_msg.y = float(gps_msg.y)
    pose_msg.theta = normalize_heading_radians(yaw)
    return pose_msg


def apply_epoch_timestamp(stamp, timestamp_sec):
    """Populate a ROS Time-like object from epoch seconds."""
    sec = int(timestamp_sec)
    nanosec = int(round((float(timestamp_sec) - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


def fill_imu_message_from_gps(
    imu_msg,
    gps_msg,
    frame_id="imu_link",
    angles_in_degrees=True,
    angular_velocity_in_degrees=True,
    angular_velocity_axes=("x", "y", "z"),
    angular_velocity_signs=(1.0, 1.0, 1.0),
    linear_acceleration_axes=("x", "y", "z"),
    linear_acceleration_signs=(1.0, 1.0, 1.0),
    linear_acceleration_scale=1.0,
    orientation_covariance=(0.0,) * 9,
    angular_velocity_covariance=(0.0,) * 9,
    linear_acceleration_covariance=(0.0,) * 9,
):
    """Populate an Imu-like object from an AutoDrive GPS/IMU-like object."""
    roll = _degrees_to_radians_if_needed(gps_msg.roll, angles_in_degrees)
    pitch = _degrees_to_radians_if_needed(gps_msg.pitch, angles_in_degrees)
    yaw = _degrees_to_radians_if_needed(gps_msg.yaw, angles_in_degrees)
    angular_velocity_axes = _coerce_vector_config(angular_velocity_axes, 3, str)
    angular_velocity_signs = _coerce_vector_config(angular_velocity_signs, 3, float)
    linear_acceleration_axes = _coerce_vector_config(linear_acceleration_axes, 3, str)
    linear_acceleration_signs = _coerce_vector_config(linear_acceleration_signs, 3, float)
    orientation_covariance = list(_coerce_vector_config(orientation_covariance, 9, float))
    angular_velocity_covariance = list(
        _coerce_vector_config(angular_velocity_covariance, 9, float)
    )
    linear_acceleration_covariance = list(
        _coerce_vector_config(linear_acceleration_covariance, 9, float)
    )

    angular_velocity_components = {
        "x": _degrees_to_radians_if_needed(gps_msg.wx, angular_velocity_in_degrees),
        "y": _degrees_to_radians_if_needed(gps_msg.wy, angular_velocity_in_degrees),
        "z": _degrees_to_radians_if_needed(gps_msg.wz, angular_velocity_in_degrees),
    }
    linear_acceleration_components = {
        "x": float(gps_msg.ax) * float(linear_acceleration_scale),
        "y": float(gps_msg.ay) * float(linear_acceleration_scale),
        "z": float(gps_msg.az) * float(linear_acceleration_scale),
    }
    wx, wy, wz = _resolve_axis_triplet(
        angular_velocity_components, angular_velocity_axes, angular_velocity_signs
    )
    ax, ay, az = _resolve_axis_triplet(
        linear_acceleration_components,
        linear_acceleration_axes,
        linear_acceleration_signs,
    )

    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

    imu_msg.header.frame_id = frame_id
    if hasattr(gps_msg, "timestamp"):
        apply_epoch_timestamp(imu_msg.header.stamp, gps_msg.timestamp)

    imu_msg.orientation.x = qx
    imu_msg.orientation.y = qy
    imu_msg.orientation.z = qz
    imu_msg.orientation.w = qw
    imu_msg.orientation_covariance = orientation_covariance
    imu_msg.angular_velocity.x = wx
    imu_msg.angular_velocity.y = wy
    imu_msg.angular_velocity.z = wz
    imu_msg.angular_velocity_covariance = angular_velocity_covariance
    imu_msg.linear_acceleration.x = ax
    imu_msg.linear_acceleration.y = ay
    imu_msg.linear_acceleration.z = az
    imu_msg.linear_acceleration_covariance = linear_acceleration_covariance
    return imu_msg
