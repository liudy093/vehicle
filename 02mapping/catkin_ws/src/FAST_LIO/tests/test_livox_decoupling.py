import re
import unittest
from pathlib import Path


FAST_LIO_ROOT = Path(__file__).resolve().parents[1]


class LivoxDecouplingTest(unittest.TestCase):
    def test_preprocess_header_guards_livox_types(self) -> None:
        content = (FAST_LIO_ROOT / "src" / "preprocess.h").read_text()

        self.assertIn("FAST_LIO_HAS_LIVOX", content)
        self.assertRegex(
            content,
            re.compile(
                r"#if\s+FAST_LIO_HAS_LIVOX\s+"
                r"#include <livox_ros_driver2/CustomMsg.h>",
                re.MULTILINE,
            ),
        )
        self.assertRegex(
            content,
            re.compile(
                r"#if\s+FAST_LIO_HAS_LIVOX[\s\S]*"
                r"void process\(const livox_ros_driver2::CustomMsg::ConstPtr &msg, "
                r"PointCloudXYZI::Ptr &pcl_out\);",
                re.MULTILINE,
            ),
        )

    def test_laser_mapping_guards_livox_callback(self) -> None:
        content = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()

        self.assertIn("FAST_LIO_HAS_LIVOX", content)
        self.assertRegex(
            content,
            re.compile(
                r"#if\s+FAST_LIO_HAS_LIVOX\s+"
                r"#include <livox_ros_driver2/CustomMsg.h>",
                re.MULTILINE,
            ),
        )
        self.assertRegex(
            content,
            re.compile(
                r"#if\s+FAST_LIO_HAS_LIVOX[\s\S]*"
                r"void livox_pcl_cbk\(const livox_ros_driver2::CustomMsg::ConstPtr &msg\)",
                re.MULTILINE,
            ),
        )

    def test_cmakelists_detects_livox_optionally(self) -> None:
        content = (FAST_LIO_ROOT / "CMakeLists.txt").read_text()

        self.assertIn("find_package(livox_ros_driver2 QUIET)", content)
        self.assertIn("FAST_LIO_HAS_LIVOX=1", content)
        self.assertIn("FAST_LIO_HAS_LIVOX=0", content)

    def test_package_xml_drops_hard_livox_dependency(self) -> None:
        content = (FAST_LIO_ROOT / "package.xml").read_text()

        self.assertNotIn("<build_depend>livox_ros_driver2</build_depend>", content)
        self.assertNotIn("<run_depend>livox_ros_driver2</run_depend>", content)


if __name__ == "__main__":
    unittest.main()
