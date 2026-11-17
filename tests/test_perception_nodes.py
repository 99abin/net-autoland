from pathlib import Path
import copy
import math
import sys
import unittest
from unittest.mock import patch
import numpy as np
import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/perception"))
import ros_stubs as ros
from perception.net_det import NetDetector
from perception.net_pos import NetPosition


class PerceptionNodeTests(unittest.TestCase):
    def setUp(self):
        ros.Node.overrides = {}
        ros.clock.value = 1000.
        self.timer = patch("time.monotonic", side_effect=lambda: ros.clock.value)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def header(self, node, frame="map"):
        return ros.Header(stamp=node.get_clock().now().to_msg(), frame_id=frame)

    def inputs(self, node, x=0., plan_id="abc", plane_x=25.):
        plan = ros.LandingPlan(header=self.header(node), verified=True, plan_id=plan_id,
            land_lat=0., land_lon=math.degrees(plane_x/6371000.), heading_deg=90.)
        node.plan_cb(plan)
        pose = ros.PoseStamped(header=self.header(node))
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        node.pose_cb(pose)
        gps = ros.NavSatFix(header=self.header(node, "base_link"), latitude=0.,
            longitude=math.degrees(x/6371000.), position_covariance_type=2,
            position_covariance=[.09, 0., 0., 0., .09, 0., 0., 0., .09])
        node.gps_cb(gps)

    def configured(self):
        node = NetPosition()
        node.info_cb(ros.CameraInfo(header=self.header(node, "camera_optical_frame"),
            k=[700., 0., 320., 0., 700., 240., 0., 0., 1.], d=[0.]*5,
            width=640, height=480, distortion_model="plumb_bob"))
        self.inputs(node)
        return node

    def detection(self, node, u=320., v=240.):
        return ros.NetDetection(header=self.header(node, "camera_optical_frame"),
            valid=True, score=1., center_u=u, center_v=v, image_width=640, image_height=480)

    def test_image_center_to_filtered_vertical_plane_estimate(self):
        node = self.configured()
        detector = NetDetector()
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.ellipse(image, (320, 240), (80, 30), 0, 0, 360, (255, 0, 0), -1)
        detector.cb(ros.Image(header=self.header(node, "camera_optical_frame"), data=image))
        detection = detector.pub.messages[-1]
        self.assertTrue(detection.valid, detection.reason)
        node.cb(detection)
        estimate = node.pub.messages[-1]
        self.assertTrue(estimate.valid, estimate.reason)
        self.assertEqual(estimate.plan_id, "abc")
        self.assertAlmostEqual(estimate.pose.position.x, 25., places=4)
        self.assertEqual(estimate.header.frame_id, "base_link")
        self.assertEqual(estimate.header.stamp.sec, detection.header.stamp.sec)
        self.assertGreater(estimate.ray_plane_incidence, .99)
        self.assertEqual(estimate.estimation_method, "vertical_plane_ray_filtered")
        ros.clock.value += .3
        node.cb(detection)
        self.assertFalse(node.pub.messages[-1].valid)

    def test_filter_stays_fixed_as_aircraft_moves_and_resets_with_plan(self):
        node = self.configured()
        node.cb(self.detection(node))
        ros.clock.value += .05
        self.inputs(node, x=1.)
        node.cb(self.detection(node))
        self.assertTrue(node.pub.messages[-1].valid, node.pub.messages[-1].reason)
        self.assertAlmostEqual(node.pub.messages[-1].pose.position.x, 24., places=4)
        self.assertAlmostEqual(node.tracker.position[0], 25., places=4)
        ros.clock.value += .05
        self.inputs(node, x=1., plan_id="new", plane_x=40.)
        node.cb(self.detection(node))
        self.assertTrue(node.pub.messages[-1].valid, node.pub.messages[-1].reason)
        self.assertEqual(node.pub.messages[-1].plan_id, "new")
        self.assertAlmostEqual(node.tracker.position[0], 40., places=4)

    def test_missing_inputs_wrong_resolution_and_unknown_gps_uncertainty_fail(self):
        for issue in ("plan", "pose", "gps", "calibration", "resolution", "gps_covariance"):
            with self.subTest(issue=issue):
                node = self.configured()
                detection = self.detection(node)
                if issue == "plan": node.plan = None
                elif issue == "pose": node.pose_history.clear()
                elif issue == "gps": node.gps_history.clear()
                elif issue == "calibration": node.camera = None
                elif issue == "resolution": detection.image_width = 1280
                else:
                    gps = ros.NavSatFix(header=self.header(node), position_covariance_type=0)
                    node.gps_cb(gps)
                node.cb(detection)
                self.assertFalse(node.pub.messages[-1].valid)

    def test_outlier_does_not_publish_a_fresh_valid_prediction(self):
        node = self.configured()
        node.cb(self.detection(node))
        old_stamp = node.tracker.stamp
        ros.clock.value += .05
        self.inputs(node)
        node.cb(self.detection(node, u=150.))
        self.assertFalse(node.pub.messages[-1].valid)
        self.assertIn("innovation gate", node.pub.messages[-1].reason)
        self.assertEqual(node.tracker.stamp, old_stamp)

    def test_camera_between_gps_fixes_uses_local_motion_at_image_time(self):
        node = self.configured()
        for time, x in ((1000.05, .75), (1000.10, 1.5)):
            ros.clock.value = time
            pose = ros.PoseStamped(header=self.header(node))
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
            node.pose_cb(pose)
        detection = self.detection(node)
        detection.header.stamp.sec = 1000
        detection.header.stamp.nanosec = 90000000
        node.cb(detection)
        result = node.pub.messages[-1]
        self.assertTrue(result.valid, result.reason)
        self.assertAlmostEqual(result.pose.position.x, 25.-1.35, places=4)
        self.assertAlmostEqual(node.tracker.position[0], 25., places=4)

    def test_calibration_and_clock_changes_clear_filter(self):
        node = self.configured()
        node.cb(self.detection(node))
        calibration = copy.deepcopy(node.camera)
        calibration.k[0] = 800.
        node.info_cb(calibration)
        self.assertIsNone(node.tracker.position)
        node.cb(self.detection(node))
        epoch = node.tracking_epoch
        wrong_frame = ros.PoseStamped(header=self.header(node, "different_map"))
        wrong_frame.pose.orientation.w = 1.
        node.pose_cb(wrong_frame)
        self.assertGreater(node.tracking_epoch, epoch)
        self.assertIsNone(node.tracker.position)
        self.assertFalse(node.pub.messages[-1].valid)
        ros.clock.value += .05
        self.inputs(node)
        node.cb(self.detection(node))
        ros.clock.value -= 1.
        node.expire()
        self.assertIsNone(node.tracker.position)
        self.assertIsNone(node.plan)
