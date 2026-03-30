import unittest
from pathlib import Path


FAST_LIO_ROOT = Path(__file__).resolve().parents[1]


class EigenConversionsDependencyTest(unittest.TestCase):
    def test_package_xml_drops_eigen_conversions_dependency(self) -> None:
        content = (FAST_LIO_ROOT / "package.xml").read_text()

        self.assertNotIn("<build_depend>eigen_conversions</build_depend>", content)
        self.assertNotIn("<run_depend>eigen_conversions</run_depend>", content)

    def test_cmakelists_drops_eigen_conversions_component(self) -> None:
        content = (FAST_LIO_ROOT / "CMakeLists.txt").read_text()

        self.assertNotIn("eigen_conversions", content)

    def test_headers_drop_unused_eigen_conversions_include(self) -> None:
        imu_processing = (FAST_LIO_ROOT / "src" / "IMU_Processing.hpp").read_text()
        common_lib = (FAST_LIO_ROOT / "include" / "common_lib.h").read_text()

        self.assertNotIn("#include <eigen_conversions/eigen_msg.h>", imu_processing)
        self.assertNotIn("#include <eigen_conversions/eigen_msg.h>", common_lib)


if __name__ == "__main__":
    unittest.main()
