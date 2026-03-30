import unittest
from pathlib import Path


FAST_LIO_ROOT = Path(__file__).resolve().parents[1]


class RoboSenseTimestampFieldSupportTest(unittest.TestCase):
    def test_preprocess_header_declares_robosense_timestamp_point_type(self) -> None:
        header_path = FAST_LIO_ROOT / "src" / "preprocess.h"
        content = header_path.read_text()

        self.assertIn("namespace robosense_ros", content)
        self.assertIn("double timestamp;", content)
        self.assertIn("(double, timestamp, timestamp)", content)

    def test_preprocess_cpp_detects_timestamp_field_and_uses_it(self) -> None:
        source_path = FAST_LIO_ROOT / "src" / "preprocess.cpp"
        content = source_path.read_text()

        self.assertIn('field.name == "timestamp"', content)
        self.assertIn("robosense_ros::Point", content)
        self.assertIn("point.timestamp", content)
        self.assertIn("first_point_time", content)
        self.assertIn("(point.timestamp - first_point_time) * 1.e3f", content)


if __name__ == "__main__":
    unittest.main()
