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
    def test_legacy_build_orchestration_entrypoint_is_removed(self) -> None:
        self.assertFalse((ROOT / "scripts" / "shared" / "build_vehicle_orchestration_images.sh").exists())

    def test_legacy_transfer_orchestration_entrypoint_is_removed(self) -> None:
        self.assertFalse((ROOT / "scripts" / "shared" / "transfer_vehicle_orchestration_images.sh").exists())

    def test_build_autodrive_script_uses_autodrive_profile(self) -> None:
        result = run_command("IMAGE_PROFILE=unexpected ./scripts/shared/build_vehicle_autodrive_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:\n  ./scripts/shared/build_vehicle_autodrive_images.sh", result.stdout)
        self.assertIn("IMAGE_PROFILE             Default: autodrive (all, autodrive, or mapping)", result.stdout)

    def test_build_mapping_script_uses_mapping_profile(self) -> None:
        result = run_command("IMAGE_PROFILE=unexpected ./scripts/shared/build_vehicle_mapping_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:\n  ./scripts/shared/build_vehicle_mapping_images.sh", result.stdout)
        self.assertIn("IMAGE_PROFILE             Default: mapping (all, autodrive, or mapping)", result.stdout)

    def test_transfer_autodrive_script_uses_autodrive_profile(self) -> None:
        result = run_command("IMAGE_PROFILE=unexpected ./scripts/shared/transfer_vehicle_autodrive_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:\n  ./scripts/shared/transfer_vehicle_autodrive_images.sh", result.stdout)
        self.assertIn("IMAGE_PROFILE             Default: autodrive (all, autodrive, or mapping)", result.stdout)

    def test_transfer_mapping_script_uses_mapping_profile(self) -> None:
        result = run_command("IMAGE_PROFILE=unexpected ./scripts/shared/transfer_vehicle_mapping_images.sh --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:\n  ./scripts/shared/transfer_vehicle_mapping_images.sh", result.stdout)
        self.assertIn("IMAGE_PROFILE             Default: mapping (all, autodrive, or mapping)", result.stdout)


if __name__ == "__main__":
    unittest.main()
