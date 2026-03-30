import unittest
from pathlib import Path


FAST_LIO_ROOT = Path(__file__).resolve().parents[1]


class RuntimeLogGatingTest(unittest.TestCase):
    def test_laser_mapping_gates_debug_file_setup(self) -> None:
        content = (FAST_LIO_ROOT / "src" / "laserMapping.cpp").read_text()

        self.assertIn("bool debug_log_ready = false;", content)
        self.assertIn("if (runtime_pos_log)", content)
        self.assertNotIn('cout << "~~~~"<<ROOT_DIR<<" doesn\'t exist"', content)
        self.assertNotIn('cout << "~~~~"<<ROOT_DIR<<" file opened"', content)


if __name__ == "__main__":
    unittest.main()
