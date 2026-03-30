import unittest
from pathlib import Path


FAST_LIO_ROOT = Path(__file__).resolve().parents[1]


class RoboSenseHeliosEntrypointTest(unittest.TestCase):
    def test_robosense_helios_config_exists_with_expected_defaults(self) -> None:
        config_path = FAST_LIO_ROOT / "config" / "robosense_helios.yaml"
        self.assertTrue(config_path.exists(), "robosense_helios.yaml should exist")

        content = config_path.read_text()
        self.assertIn('lid_topic:  "/forward/rslidar_points"', content)
        self.assertIn('imu_topic:  "/imu"', content)
        self.assertIn("lidar_type: 2", content)
        self.assertIn("scan_line: 16", content)

    def test_mapping_robosense_launch_exists_and_defaults_to_helios(self) -> None:
        launch_path = FAST_LIO_ROOT / "launch" / "mapping_robosense.launch"
        self.assertTrue(launch_path.exists(), "mapping_robosense.launch should exist")

        content = launch_path.read_text()
        self.assertIn(
            'default="$(find fast_lio2)/config/robosense_helios.yaml"',
            content,
        )
        self.assertIn('<arg name="lid_topic" default="/forward/rslidar_points" />', content)
        self.assertIn('<arg name="imu_topic" default="/imu" />', content)
        self.assertIn('<param name="common/lid_topic" value="$(arg lid_topic)" />', content)
        self.assertIn('<param name="common/imu_topic" value="$(arg imu_topic)" />', content)


if __name__ == "__main__":
    unittest.main()
