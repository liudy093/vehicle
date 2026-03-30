import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
SETUP_PY = ROOT / "setup.py"


class Cgi410RawImuEntrypointTest(unittest.TestCase):
    def test_setup_registers_cgi410_raw_imu_bridge_node(self):
        content = SETUP_PY.read_text()
        self.assertIn(
            "cgi410_raw_imu_bridge_node = data_sender_ros2.cgi410_raw_imu_bridge_node:main",
            content,
        )

    def test_setup_registers_cgi410_serial_bridge_node(self):
        content = SETUP_PY.read_text()
        self.assertIn(
            "cgi410_serial_bridge_node = data_sender_ros2.cgi410_serial_bridge_node:main",
            content,
        )


if __name__ == "__main__":
    unittest.main()
