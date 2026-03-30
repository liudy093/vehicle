import unittest
from types import SimpleNamespace

from data_sender_ros2.cgi410_serial_bridge_node import (
    fill_imu_message_from_gtimu,
    parse_gpchc_sentence,
)


class Cgi410SerialBridgeTests(unittest.TestCase):
    def make_imu(self):
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

    def test_parse_gpchc_sentence_maps_current_vehicle_fields(self):
        parsed = parse_gpchc_sentence(
            "$GPCHC,2411,288621.45,266.43,-0.02,0.47,-0.01,0.03,-0.05,-0.0036,0.0310,0.9994,34.58676538,113.67966691,150.80,-0.001,0.006,-0.004,0.006,41,39,42,1,00a2*00"
        )

        self.assertIsNotNone(parsed)
        self.assertAlmostEqual(parsed.yaw, 183.57)
        self.assertAlmostEqual(parsed.pitch, -0.02)
        self.assertAlmostEqual(parsed.roll, 0.47)
        self.assertAlmostEqual(parsed.wx, -0.01)
        self.assertAlmostEqual(parsed.wy, 0.03)
        self.assertAlmostEqual(parsed.wz, -0.05)
        self.assertAlmostEqual(parsed.ax, -0.0036 * 9.8)
        self.assertAlmostEqual(parsed.ay, 0.0310 * 9.8)
        self.assertAlmostEqual(parsed.az, 0.9994 * 9.8)
        self.assertAlmostEqual(parsed.latitude, 34.58676538)
        self.assertAlmostEqual(parsed.longitude, 113.67966691)
        self.assertAlmostEqual(parsed.height, 150.80)
        self.assertAlmostEqual(parsed.eastvelocity, -0.001)
        self.assertAlmostEqual(parsed.northvelocity, 0.006)
        self.assertAlmostEqual(parsed.skyvelocity, -0.004)

    def test_parse_gpchc_sentence_rejects_non_gpchc_input(self):
        parsed = parse_gpchc_sentence(
            "$GTIMU,2411,288621.420,-0.2133,-0.0368,-0.0950,-0.0056,0.0147,0.9988,44.5*6F"
        )

        self.assertIsNone(parsed)

    def test_fill_imu_message_from_gtimu_converts_expected_units(self):
        latest_gps = SimpleNamespace(
            timestamp=1712345678.25,
            yaw=180.0,
            pitch=0.0,
            roll=0.5,
        )
        parsed_gtimu = SimpleNamespace(
            angular_velocity=SimpleNamespace(x=-0.2418, y=-0.022, z=-0.0248),
            linear_acceleration=SimpleNamespace(x=-0.01, y=0.0176, z=0.9991),
        )
        imu = self.make_imu()

        fill_imu_message_from_gtimu(imu, latest_gps, parsed_gtimu)

        self.assertAlmostEqual(imu.angular_velocity.x, -0.2418 * 3.141592653589793 / 180.0)
        self.assertAlmostEqual(imu.angular_velocity.y, -0.022 * 3.141592653589793 / 180.0)
        self.assertAlmostEqual(imu.angular_velocity.z, -0.0248 * 3.141592653589793 / 180.0)
        self.assertAlmostEqual(imu.linear_acceleration.x, -0.01 * 9.8)
        self.assertAlmostEqual(imu.linear_acceleration.y, 0.0176 * 9.8)
        self.assertAlmostEqual(imu.linear_acceleration.z, 0.9991 * 9.8)


if __name__ == "__main__":
    unittest.main()
