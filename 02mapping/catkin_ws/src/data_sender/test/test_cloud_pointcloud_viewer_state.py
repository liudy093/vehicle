import importlib.util
import pathlib
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "cloud_pointcloud_viewer_state.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "cloud_pointcloud_viewer_state", MODULE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class CloudPointcloudViewerStateTest(unittest.TestCase):
    def test_builds_default_dual_vehicle_snapshot(self):
        module = load_module()

        topics = module.resolve_lidar_topics("")
        snapshot = module.build_snapshot(topics)

        self.assertEqual(
            topics,
            {
                "vehicle1": {
                    "fused": "/vehicle1/cloud_sent/fused",
                    "global_cloud": "/vehicle1/global_cloud",
                },
                "vehicle2": {
                    "fused": "/vehicle2/cloud_sent/fused",
                    "global_cloud": "/vehicle2/global_cloud",
                },
                "global": {
                    "global_map": "/global_map",
                    "global_map_optimized": "/global_map_optimized",
                },
            },
        )
        self.assertEqual(snapshot["vehicle_order"], ["vehicle1", "vehicle2", "global"])
        self.assertEqual(
            snapshot["topic_order"],
            ["fused", "global_cloud", "global_map", "global_map_optimized"],
        )
        self.assertEqual(
            set(snapshot["clouds"].keys()), {"vehicle1", "vehicle2", "global"}
        )
        self.assertEqual(snapshot["clouds"]["vehicle1"]["fused"]["point_count"], 0)
        self.assertIsNone(snapshot["clouds"]["vehicle2"]["global_cloud"]["bounds"])
        self.assertEqual(
            snapshot["clouds"]["global"]["global_map_optimized"]["point_count"], 0
        )

    def test_parses_explicit_topic_mapping(self):
        module = load_module()

        topics = module.resolve_lidar_topics(
            "vehicle1/forward=/vehicle1/cloud_sent/forward,"
            "vehicle1/fused=/vehicle1/cloud_sent/fused,"
            "vehicle2/fused=/vehicle2/cloud_sent/fused,"
            "vehicle2/global_cloud=/vehicle2/global_cloud"
        )

        self.assertEqual(topics["vehicle1"]["forward"], "/vehicle1/cloud_sent/forward")
        self.assertEqual(topics["vehicle1"]["fused"], "/vehicle1/cloud_sent/fused")
        self.assertEqual(topics["vehicle2"]["fused"], "/vehicle2/cloud_sent/fused")
        self.assertEqual(topics["vehicle2"]["global_cloud"], "/vehicle2/global_cloud")

    def test_build_snapshot_keeps_nested_payloads(self):
        module = load_module()

        topics = module.resolve_lidar_topics(
            "vehicle1/forward=/vehicle1/cloud_sent/forward,"
            "vehicle1/fused=/vehicle1/cloud_sent/fused,"
            "vehicle2/forward=/vehicle2/cloud_sent/forward,"
            "vehicle2/fused=/vehicle2/cloud_sent/fused"
        )
        payloads = {
            "vehicle1": {
                "forward": {
                    "frame_id": "vehicle1/forward",
                    "stamp": {"secs": 1, "nsecs": 2},
                    "point_count": 3,
                    "points": [[1, 2, 3, 4]],
                    "bounds": None,
                }
            },
            "vehicle2": {
                "fused": {
                    "frame_id": "vehicle2/fused",
                    "stamp": {"secs": 5, "nsecs": 6},
                    "point_count": 7,
                    "points": [[7, 8, 9, 10]],
                    "bounds": None,
                }
            },
        }

        snapshot = module.build_snapshot(topics, payloads)

        self.assertEqual(snapshot["clouds"]["vehicle1"]["forward"]["frame_id"], "vehicle1/forward")
        self.assertEqual(snapshot["clouds"]["vehicle2"]["fused"]["frame_id"], "vehicle2/fused")
        self.assertEqual(snapshot["clouds"]["vehicle1"]["fused"]["point_count"], 0)
        self.assertEqual(snapshot["clouds"]["vehicle2"]["forward"]["point_count"], 0)


if __name__ == "__main__":
    unittest.main()
