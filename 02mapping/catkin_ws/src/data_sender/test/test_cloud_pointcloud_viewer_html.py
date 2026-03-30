import unittest
from pathlib import Path


HTML_PATH = (
    Path(__file__).resolve().parents[1]
    / "web"
    / "cloud_pointcloud_viewer.html"
)


class CloudPointcloudViewerHtmlTest(unittest.TestCase):
    def test_viewer_includes_rotation_toggle(self):
        html = HTML_PATH.read_text(encoding="utf-8")
        self.assertIn("自动旋转", html)
        self.assertIn('id="auto-rotate-toggle"', html)

    def test_viewer_uses_on_demand_rendering(self):
        html = HTML_PATH.read_text(encoding="utf-8")
        self.assertIn("scheduleRender()", html)
        self.assertIn("function renderFrame(timestamp)", html)
        self.assertIn("if (animationFrameId !== null)", html)


if __name__ == "__main__":
    unittest.main()
