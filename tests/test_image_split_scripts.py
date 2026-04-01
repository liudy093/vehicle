import pathlib
import subprocess
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


def run_command(command: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", "-lc", command],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


class ImageSplitScriptTests(unittest.TestCase):
    def test_build_orchestration_help_mentions_image_profile(self) -> None:
        result = run_command("./scripts/shared/build_vehicle_orchestration_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("IMAGE_PROFILE             Default: all (all, autodrive, or mapping)", result.stdout)

    def test_transfer_orchestration_help_mentions_image_profile(self) -> None:
        result = run_command("./scripts/shared/transfer_vehicle_orchestration_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("IMAGE_PROFILE             Default: all (all, autodrive, or mapping)", result.stdout)

    def test_build_autodrive_wrapper_uses_autodrive_profile(self) -> None:
        result = run_command("IMAGE_PROFILE=unexpected ./scripts/shared/build_vehicle_autodrive_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("IMAGE_PROFILE             Default: autodrive (all, autodrive, or mapping)", result.stdout)

    def test_transfer_mapping_wrapper_uses_mapping_profile(self) -> None:
        result = run_command("IMAGE_PROFILE=unexpected ./scripts/shared/transfer_vehicle_mapping_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("IMAGE_PROFILE             Default: mapping (all, autodrive, or mapping)", result.stdout)


if __name__ == "__main__":
    unittest.main()
