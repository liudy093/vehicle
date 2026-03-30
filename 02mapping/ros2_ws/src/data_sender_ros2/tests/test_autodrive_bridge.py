import math
import unittest
from types import SimpleNamespace

from data_sender_ros2.autodrive_bridge import (
    apply_epoch_timestamp,
    fill_imu_message_from_gps,
    fill_pose2d_message_from_gps,
)


def make_pose2d():
    return SimpleNamespace(x=None, y=None, theta=None)


def make_imu():
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=None,
            stamp=SimpleNamespace(sec=None, nanosec=None),
        ),
        orientation=SimpleNamespace(x=None, y=None, z=None, w=None),
        orientation_covariance=[None, None, None, None, None, None, None, None, None],
        angular_velocity=SimpleNamespace(x=None, y=None, z=None),
        angular_velocity_covariance=[None, None, None, None, None, None, None, None, None],
        linear_acceleration=SimpleNamespace(x=None, y=None, z=None),
        linear_acceleration_covariance=[None, None, None, None, None, None, None, None, None],
    )


def make_gps_msg(**overrides):
    base = {
        "timestamp": 1712345678.25,
        "x": 12.5,
        "y": -3.0,
        "yaw": 90.0,
        "pitch": 0.0,
        "roll": 0.0,
        "wx": 180.0,
        "wy": 0.0,
        "wz": -90.0,
        "ax": 1.0,
        "ay": 2.0,
        "az": 3.0,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


class AutodriveBridgeTests(unittest.TestCase):
    def test_fill_pose2d_message_from_gps_converts_degrees_to_radians(self):
        pose = make_pose2d()
        gps_msg = make_gps_msg(yaw=90.0, x=4.0, y=5.0)

        fill_pose2d_message_from_gps(pose, gps_msg)

        self.assertEqual(pose.x, 4.0)
        self.assertEqual(pose.y, 5.0)
        self.assertAlmostEqual(pose.theta, math.pi / 2.0)

    def test_fill_pose2d_message_from_gps_normalizes_wrapped_heading(self):
        pose = make_pose2d()
        gps_msg = make_gps_msg(yaw=350.0)

        fill_pose2d_message_from_gps(pose, gps_msg)

        self.assertAlmostEqual(pose.theta, -math.radians(10.0))

    def test_fill_imu_message_from_gps_populates_orientation_and_motion(self):
        imu = make_imu()
        gps_msg = make_gps_msg(yaw=90.0, pitch=0.0, roll=0.0, wx=180.0, wz=-90.0)

        fill_imu_message_from_gps(
            imu,
            gps_msg,
            frame_id="imu_link",
            angles_in_degrees=True,
            angular_velocity_in_degrees=True,
        )

        self.assertEqual(imu.header.frame_id, "imu_link")
        self.assertAlmostEqual(imu.orientation.x, 0.0)
        self.assertAlmostEqual(imu.orientation.y, 0.0)
        self.assertAlmostEqual(imu.orientation.z, math.sqrt(0.5))
        self.assertAlmostEqual(imu.orientation.w, math.sqrt(0.5))
        self.assertAlmostEqual(imu.angular_velocity.x, math.pi)
        self.assertAlmostEqual(imu.angular_velocity.y, 0.0)
        self.assertAlmostEqual(imu.angular_velocity.z, -math.pi / 2.0)
        self.assertAlmostEqual(imu.linear_acceleration.x, 1.0)
        self.assertAlmostEqual(imu.linear_acceleration.y, 2.0)
        self.assertAlmostEqual(imu.linear_acceleration.z, 3.0)

    def test_fill_imu_message_from_gps_supports_axis_remap_and_sign_flip(self):
        imu = make_imu()
        gps_msg = make_gps_msg(wx=1.0, wy=2.0, wz=3.0, ax=4.0, ay=5.0, az=6.0)

        fill_imu_message_from_gps(
            imu,
            gps_msg,
            angular_velocity_in_degrees=False,
            angular_velocity_axes=("z", "x", "y"),
            angular_velocity_signs=(-1.0, 1.0, -1.0),
            linear_acceleration_axes=("y", "z", "x"),
            linear_acceleration_signs=(1.0, -1.0, 1.0),
            linear_acceleration_scale=9.8,
        )

        self.assertAlmostEqual(imu.angular_velocity.x, -3.0)
        self.assertAlmostEqual(imu.angular_velocity.y, 1.0)
        self.assertAlmostEqual(imu.angular_velocity.z, -2.0)
        self.assertAlmostEqual(imu.linear_acceleration.x, 5.0 * 9.8)
        self.assertAlmostEqual(imu.linear_acceleration.y, -6.0 * 9.8)
        self.assertAlmostEqual(imu.linear_acceleration.z, 4.0 * 9.8)

    def test_fill_imu_message_from_gps_populates_configured_covariances(self):
        imu = make_imu()
        gps_msg = make_gps_msg()

        fill_imu_message_from_gps(
            imu,
            gps_msg,
            orientation_covariance=(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0),
            angular_velocity_covariance=(4.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 6.0),
            linear_acceleration_covariance=(7.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 9.0),
        )

        self.assertEqual(imu.orientation_covariance, [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0])
        self.assertEqual(
            imu.angular_velocity_covariance,
            [4.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 6.0],
        )
        self.assertEqual(
            imu.linear_acceleration_covariance,
            [7.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 9.0],
        )

    def test_apply_epoch_timestamp_splits_seconds_and_nanoseconds(self):
        stamp = SimpleNamespace(sec=None, nanosec=None)

        apply_epoch_timestamp(stamp, 1712345678.25)

        self.assertEqual(stamp.sec, 1712345678)
        self.assertEqual(stamp.nanosec, 250000000)


if __name__ == "__main__":
    unittest.main()
