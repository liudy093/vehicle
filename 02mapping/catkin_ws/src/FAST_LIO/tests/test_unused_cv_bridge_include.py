import unittest
from pathlib import Path


FAST_LIO_ROOT = Path(__file__).resolve().parents[1]


class CvBridgeIncludeTest(unittest.TestCase):
    def test_scancontext_header_drops_unused_cv_bridge_include(self) -> None:
        content = (FAST_LIO_ROOT / "include" / "Scancontext.h").read_text()

        self.assertNotIn("#include <cv_bridge/cv_bridge.h>", content)


if __name__ == "__main__":
    unittest.main()
