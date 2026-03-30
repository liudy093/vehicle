import math
import unittest

from data_sender_ros2.cgi410_raw_imu_bridge_node import parse_gtimu_sentence


class Cgi410RawImuBridgeTests(unittest.TestCase):
    def test_parse_gtimu_sentence_returns_measurements(self):
        parsed = parse_gtimu_sentence(
            "$GTIMU,2411,288621.420,-0.2133,-0.0368,-0.0950,-0.0056,0.0147,0.9988,44.5*6F"
        )

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.gps_week, 2411)
        self.assertAlmostEqual(parsed.gps_seconds, 288621.420)
        self.assertAlmostEqual(parsed.angular_velocity.x, -0.2133)
        self.assertAlmostEqual(parsed.angular_velocity.y, -0.0368)
        self.assertAlmostEqual(parsed.angular_velocity.z, -0.0950)
        self.assertAlmostEqual(parsed.linear_acceleration.x, -0.0056)
        self.assertAlmostEqual(parsed.linear_acceleration.y, 0.0147)
        self.assertAlmostEqual(parsed.linear_acceleration.z, 0.9988)
        self.assertAlmostEqual(parsed.temperature_celsius, 44.5)

    def test_parse_gtimu_sentence_rejects_bad_checksum(self):
        parsed = parse_gtimu_sentence(
            "$GTIMU,2411,288621.420,-0.2133,-0.0368,-0.0950,-0.0056,0.0147,0.9988,44.5*00"
        )

        self.assertIsNone(parsed)

    def test_parse_gtimu_sentence_rejects_other_nmea_types(self):
        parsed = parse_gtimu_sentence(
            "$GPCHC,2411,288621.45,266.43,-0.02,0.47,-0.01,0.03,-0.05,-0.0036,0.0310,0.9994,34.58676538,113.67966691,150.80,-0.001,0.006,-0.004,0.006,41,39,42,1,00a2*00"
        )

        self.assertIsNone(parsed)


if __name__ == "__main__":
    unittest.main()
