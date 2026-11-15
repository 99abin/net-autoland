from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "perception"))
try:
    import cv2
    from perception.detector import detect_rectangle
    from perception.pose_estimation import estimate_pose
except ImportError:
    cv2 = None


@unittest.skipIf(cv2 is None, "OpenCV is required for perception tests")
class PerceptionTests(unittest.TestCase):
    def test_user_reference_blue_rectangle(self):
        image = cv2.imread(str(ROOT / "tests/data/net_reference.png"))
        result, _ = detect_rectangle(image)
        self.assertIsNotNone(result)
        corners, score = result
        self.assertGreater(score, 0.9)
        expected = np.array([[160, 134], [653, 131], [647, 302], [167, 317]])
        self.assertLess(np.max(np.linalg.norm(corners - expected, axis=1)), 8)

    def test_blank_and_cyan_distractor(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (50, 50), (500, 300), (255, 255, 0), -1)
        self.assertIsNone(detect_rectangle(image)[0])

    def test_perspective_color_panel(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.fillConvexPoly(
            image, np.array([[80, 100], [550, 130], [500, 350], [100, 320]]), (255, 0, 0)
        )
        self.assertIsNotNone(detect_rectangle(image)[0])

    def projection(self, rvec=None, tvec=None):
        obj = np.array(
            [[-2.75, -1.0, 0], [2.75, -1.0, 0], [2.75, 1.0, 0], [-2.75, 1.0, 0]], dtype=float
        )
        k = np.array([[700, 0, 320], [0, 700, 240], [0, 0, 1]], dtype=float)
        rvec = np.array([0.04, -0.12, 0.01]) if rvec is None else np.array(rvec, dtype=float)
        tvec = np.array([1.0, -0.5, 25.0]) if tvec is None else np.array(tvec, dtype=float)
        pixels, _ = cv2.projectPoints(obj, rvec, tvec, k, np.zeros(5))
        return pixels.reshape(4, 2), k, tvec

    def test_metric_pose_and_body_transform(self):
        uv, k, t = self.projection()
        transform = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        p, q, cov, error = estimate_pose(uv, k, np.zeros(5), 5.5, 2, transform, [0, 0, 0])
        np.testing.assert_allclose(p, transform @ t, atol=1e-5)
        self.assertLess(error, 1e-5)
        self.assertAlmostEqual(np.linalg.norm(q), 1)
        self.assertTrue(np.isfinite(cov).all())
        self.assertTrue(np.all(np.diag(cov) >= 0))

    def test_invalid_calibration_and_rotation(self):
        uv, k, _ = self.projection()
        with self.assertRaises(ValueError):
            estimate_pose(uv, np.zeros((3, 3)), [], 5.5, 2, np.eye(3), [0, 0, 0])
        with self.assertRaises(ValueError):
            estimate_pose(uv, k, [], 5.5, 2, -np.eye(3), [0, 0, 0])

    def test_corrupt_corner_reprojection(self):
        uv, k, _ = self.projection()
        uv[2] += [70, 60]
        with self.assertRaises(ValueError):
            estimate_pose(
                uv,
                k,
                [],
                5.5,
                2,
                [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                [0, 0, 0],
                max_reprojection=1,
            )


if __name__ == "__main__":
    unittest.main()
