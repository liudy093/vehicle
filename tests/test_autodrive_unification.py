import pathlib
import subprocess
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
WORKFLOW_PATH = ROOT / "deploy" / "autodrive" / "argo" / "vehicle-autodrive-workflow.yaml"


def run_command(command: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", "-lc", command],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )


class AutodriveUnificationTests(unittest.TestCase):
    def test_workflow_uses_unified_template_names(self) -> None:
        workflow_text = WORKFLOW_PATH.read_text()

        self.assertIn("- name: stop-autodrive", workflow_text)
        self.assertIn("- name: run-autodrive", workflow_text)
        self.assertNotIn("- name: stop-vehicle1-autodrive", workflow_text)
        self.assertNotIn("- name: stop-vehicle2-autodrive", workflow_text)
        self.assertNotIn("- name: run-vehicle1-autodrive", workflow_text)
        self.assertNotIn("- name: run-vehicle2-autodrive", workflow_text)

    def test_vehicle_specific_wrapper_scripts_are_removed(self) -> None:
        self.assertFalse((ROOT / "scripts" / "autodrive" / "start_vehicle1_autodrive.sh").exists())
        self.assertFalse((ROOT / "scripts" / "autodrive" / "start_vehicle2_autodrive.sh").exists())
        self.assertFalse((ROOT / "scripts" / "autodrive" / "stop_vehicle1_autodrive.sh").exists())
        self.assertFalse((ROOT / "scripts" / "autodrive" / "stop_vehicle2_autodrive.sh").exists())

    def test_vehicle_specific_common_wrappers_are_removed(self) -> None:
        self.assertFalse((ROOT / "scripts" / "shared" / "lib" / "vehicle1_autodrive_common.sh").exists())
        self.assertFalse((ROOT / "scripts" / "shared" / "lib" / "vehicle2_autodrive_common.sh").exists())

    def test_vehicle2_default_host_comes_from_inventory(self) -> None:
        result = run_command(
            "source scripts/shared/lib/vehicle_autodrive_common.sh && "
            "vehicle_autodrive_default_host vehicle2"
        )

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertEqual(result.stdout.strip(), "nvidia@192.168.3.6")

    def test_vehicle_autodrive_common_rejects_unknown_vehicle(self) -> None:
        result = run_command(
            "source scripts/shared/lib/vehicle_autodrive_common.sh && "
            "vehicle_autodrive_default_host vehicle9"
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Unsupported vehicle id", result.stderr)

    def test_start_help_uses_unified_entrypoint_and_inventory_default_host(self) -> None:
        result = run_command("./scripts/autodrive/start_vehicle_autodrive.sh vehicle2 --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn(
            "Usage:\n  ./scripts/autodrive/start_vehicle_autodrive.sh vehicle2 [end_1|end_2|end_3]",
            result.stdout,
        )
        self.assertIn(
            "VEHICLE_HOST         Default: config/shared/vehicle_hosts.env -> VEHICLE2_HOST",
            result.stdout,
        )

    def test_stop_help_uses_unified_entrypoint_and_inventory_default_host(self) -> None:
        result = run_command("./scripts/autodrive/stop_vehicle_autodrive.sh vehicle2 --help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn(
            "Usage:\n  ./scripts/autodrive/stop_vehicle_autodrive.sh vehicle2",
            result.stdout,
        )
        self.assertIn(
            "VEHICLE_HOST         Default: config/shared/vehicle_hosts.env -> VEHICLE2_HOST",
            result.stdout,
        )


if __name__ == "__main__":
    unittest.main()
