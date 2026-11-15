from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "perception"))
import ros_stubs as ros

try:
    import cv2
    from perception.net_det import NetDetector
    from perception.net_pos import NetPosition
except ImportError:
    cv2 = None


@unittest.skipIf(cv2 is None, "OpenCV required")
class PerceptionNodeTests(unittest.TestCase):
    def test_image_to_metric_net_estimate(self):
        ros.Node.overrides = {}
        ros.clock.value = 1000.0
        with patch("time.monotonic", side_effect=lambda: ros.clock.value):
            det = NetDetector()
            pos = NetPosition()
            k = np.array([[700.0, 0, 320], [0, 700.0, 240], [0, 0, 1]])
            obj = np.array([[-2.75, -1.0, 0], [2.75, -1.0, 0], [2.75, 1.0, 0], [-2.75, 1.0, 0]])
            uv, _ = cv2.projectPoints(
                obj, np.array([0.04, -0.12, 0.01]), np.array([1.0, -0.5, 25.0]), k, np.zeros(5)
            )
            image = np.zeros((480, 640, 3), np.uint8)
            cv2.fillConvexPoly(image, uv.reshape(4, 2).astype(np.int32), (255, 0, 0))
            header = ros.Header(
                stamp=det.get_clock().now().to_msg(), frame_id="camera_optical_frame"
            )
            camera = ros.CameraInfo(
                header=header,
                k=k.reshape(9).tolist(),
                d=[0.0] * 5,
                width=640,
                height=480,
                distortion_model="plumb_bob",
            )
            pos.info_cb(camera)
            det.cb(ros.Image(header=header, data=image))
            self.assertTrue(det.pub.messages[-1].valid)
            pos.cb(det.pub.messages[-1])
            result = pos.pub.messages[-1]
            self.assertTrue(result.valid, result.reason)
            self.assertAlmostEqual(result.pose.position.x, 25, delta=1.5)
            self.assertEqual(result.header.frame_id, "base_link")
            ros.clock.value += 0.3
            pos.cb(det.pub.messages[-1])
            self.assertFalse(pos.pub.messages[-1].valid)

    def test_missing_calibration_produces_invalid_estimate(self):
        ros.Node.overrides = {}
        pos = NetPosition()
        detection = ros.NetDetection(valid=True, score=1.0)
        detection.header.stamp = pos.get_clock().now().to_msg()
        pos.cb(detection)
        self.assertFalse(pos.pub.messages[-1].valid)


if __name__ == "__main__":
    unittest.main()
