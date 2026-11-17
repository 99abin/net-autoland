from pathlib import Path
import math
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/perception"))
from perception.detector import detect_center
from perception.ray_estimation import (StationaryTargetFilter, plane_measurement,
    net_offset_enu, sample_bracket, image_pose, quaternion_matrix)

MOUNT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.]])
K = np.array([[700., 0, 320], [0, 700, 240], [0, 0, 1]])


class RayEstimationTests(unittest.TestCase):
    def ray(self, pixel=(320., 240.), offset=(30., 0.), heading=90., rotation=None,
            translation=(0., 0., 0.), **extra):
        return plane_measurement(pixel, K, np.zeros(5), MOUNT, translation,
            np.eye(3) if rotation is None else rotation, offset, heading, np.eye(2)*.09, **extra)

    def test_level_camera_and_net_same_altitude_has_unique_vertical_intersection(self):
        body, covariance, incidence = self.ray()
        np.testing.assert_allclose(body, (30, 0, 0), atol=1e-7)
        self.assertAlmostEqual(incidence, 1.)
        self.assertTrue(np.isfinite(covariance).all())
        self.assertGreaterEqual(np.linalg.eigvalsh(covariance).min(), -1e-9)

    def test_distorted_center_mount_offset_and_rotated_aircraft(self):
        rotation = quaternion_matrix((0, 0, math.sin(math.pi/4), math.cos(math.pi/4)))
        target = np.array([30., 1., -2.])
        translation = np.array([.45, -.02, .01])
        optical = MOUNT.T @ (target-translation)
        distortion = np.array([.08, -.03, .001, -.002, 0.])
        pixel, _ = cv2.projectPoints(optical.reshape(1, 3), np.zeros(3), np.zeros(3), K, distortion)
        body, covariance, _ = plane_measurement(pixel.reshape(2), K, distortion, MOUNT,
            translation, rotation, (-1., 30.), 0., np.eye(2)*.09)
        np.testing.assert_allclose(body, target, atol=1e-5)
        self.assertTrue(np.isfinite(covariance).all())

    def test_normal_sign_does_not_change_plane(self):
        np.testing.assert_allclose(self.ray(heading=270.)[0], self.ray()[0], atol=1e-7)

    def test_parallel_behind_and_distant_intersections_are_rejected(self):
        for kwargs in (dict(heading=0.), dict(offset=(-30., 0.)), dict(offset=(250., 0.)),
                       dict(heading=5.)):
            with self.assertRaises(ValueError):
                self.ray(**kwargs)

    def test_uncertainty_grows_with_range_and_survey_uncertainty(self):
        _, near, _ = self.ray(offset=(30., 0.))
        _, far, _ = self.ray(offset=(90., 0.))
        self.assertGreater(far[2, 2], near[2, 2])
        _, poor_survey, _ = self.ray(plane_position_std=5.)
        self.assertGreater(poor_survey[0, 0], near[0, 0])

    def test_invalid_inputs_and_unsynchronized_history_fail(self):
        with self.assertRaises(ValueError):
            self.ray(pixel=(float("nan"), 240))
        with self.assertRaises(ValueError):
            plane_measurement((320, 240), K, [], -np.eye(3), (0, 0, 0),
                              np.eye(3), (30, 0), 90, np.eye(2))
        for history, stamp in (([], 1.), ([(1.,), (2.,)], 1.5), ([(1.,)], 1.2)):
            with self.assertRaises(ValueError):
                sample_bracket(history, stamp)
        p, rotation = image_pose([(1., (0, 0, 0), (0, 0, 0, 1)),
                                  (1.1, (2, 0, 0), (0, 0, 0, -1))], 1.05)
        np.testing.assert_allclose(p, (1, 0, 0))
        np.testing.assert_allclose(rotation, np.eye(3))

    def test_geographic_offset_is_east_north_and_wraps_dateline(self):
        offset = net_offset_enu(0., 0., 0., math.degrees(30/6371000.))
        np.testing.assert_allclose(offset, (30, 0), atol=1e-6)
        self.assertLess(abs(net_offset_enu(0, 179.9999, 0, -179.9999)[0]), 30)

    def test_center_detection_does_not_require_four_corners(self):
        image = np.zeros((480, 640, 3), np.uint8)
        cv2.ellipse(image, (320, 240), (90, 35), 15, 0, 360, (255, 0, 0), -1)
        result, _ = detect_center(image)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0], (320, 240), atol=1.)
        cv2.rectangle(image, (0, 0), (640, 480), (0, 0, 0), -1)
        cv2.rectangle(image, (0, 150), (200, 300), (255, 0, 0), -1)
        self.assertIsNone(detect_center(image)[0])


class StationaryFilterTests(unittest.TestCase):
    def test_noise_is_smoothed_with_a_confidence_floor(self):
        tracker = StationaryTargetFilter()
        measurements, estimates = [], []
        for index in range(40):
            observation = np.array([30., (-1)**index*.4, 1.])
            filtered, covariance = tracker.update(observation, np.eye(3), 10+index*.05)
            measurements.append(observation[1])
            estimates.append(filtered[1])
            self.assertGreaterEqual(np.linalg.eigvalsh(covariance).min(), .75**2-1e-9)
        self.assertLess(np.std(estimates[10:]), np.std(measurements[10:]))

    def test_outlier_and_old_frame_do_not_renew_track(self):
        tracker = StationaryTargetFilter()
        tracker.update((30, 0, 1), np.eye(3), 1.)
        for point, stamp in (((50, 0, 1), 1.1), ((30, 0, 1), .9), ((30, 0, 1), 1.)):
            with self.assertRaises(ValueError):
                tracker.update(point, np.eye(3), stamp)
            self.assertEqual(tracker.stamp, 1.)
            np.testing.assert_equal(tracker.position, (30, 0, 1))
        tracker.reset()
        tracker.update((50, 0, 1), np.eye(3), 2.)
        self.assertEqual(tracker.stamp, 2.)
