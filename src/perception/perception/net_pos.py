"""Image centre -> surveyed vertical plane -> filtered stationary target."""
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, NavSatFix
from geometry_msgs.msg import PoseStamped
from autoland_interfaces.msg import NetDetection, NetEstimate, LandingPlan
from .ray_estimation import (StationaryTargetFilter, image_pose, net_offset_enu,
                             plane_measurement, quaternion_matrix, sample_bracket)


class NetPosition(Node):
    def __init__(self):
        super().__init__("net_pos")
        defaults = dict(
            camera_info_topic="/camera/camera_info", camera_frame="camera_optical_frame",
            body_frame="base_link", local_frame="map", mavros_namespace="/mavros",
            pixel_noise_px=2., detection_timeout_s=.25, telemetry_timeout_s=1.,
            min_detection_score=.5, max_position_std_m=2.,
            camera_to_body_rotation=[0., 0., 1., -1., 0., 0., 0., -1., 0.],
            camera_translation_body_m=[0., 0., 0.],
            attitude_std_deg=1., local_position_std_m=.5,
            gps_position_std_floor_m=.5, plane_position_std_m=.5,
            plane_heading_std_deg=1., min_ray_plane_incidence=.2, maximum_range_m=200.,
            filter_process_std_m_sqrt_s=.02, filter_min_position_std_m=.75,
            filter_innovation_gate=11.345, filter_max_jump_m=3.,
        )
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key in ("pixel_noise_px", "detection_timeout_s", "telemetry_timeout_s",
                    "max_position_std_m", "maximum_range_m"):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError("Invalid positive perception parameter: " + key)
        for key in ("attitude_std_deg", "local_position_std_m", "gps_position_std_floor_m",
                    "plane_position_std_m", "plane_heading_std_deg"):
            if not math.isfinite(self.p[key]) or self.p[key] < 0:
                raise ValueError("Invalid uncertainty parameter: " + key)
        if not 0 < self.p["min_ray_plane_incidence"] < 1 or not 0 <= self.p["min_detection_score"] <= 1:
            raise ValueError("Invalid perception score/incidence limit")
        self.tracker = StationaryTargetFilter(self.p["filter_process_std_m_sqrt_s"],
            self.p["filter_min_position_std_m"], self.p["filter_innovation_gate"], self.p["filter_max_jump_m"])
        self.tracking_epoch = 0
        self.camera = self.plan = None
        self.plan_time = self.pose_time = self.gps_time = self.last_frame = None
        self.pose_history, self.gps_history = deque(maxlen=500), deque(maxlen=250)
        self.last_clock_ns = None
        self.pub = self.create_publisher(NetEstimate, "/autoland/net_estimate", 10)
        self.create_subscription(CameraInfo, self.p["camera_info_topic"], self.info_cb, qos_profile_sensor_data)
        self.create_subscription(NetDetection, "/autoland/net_detection", self.cb, 10)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(LandingPlan, "/autoland/landing_plan", self.plan_cb, retained)
        ns = self.p["mavros_namespace"].rstrip("/")
        self.create_subscription(PoseStamped, ns + "/local_position/pose", self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, ns + "/global_position/global", self.gps_cb, qos_profile_sensor_data)
        self.create_timer(.05, self.expire)

    def reset_track(self):
        self.tracker.reset()
        self.tracking_epoch = (self.tracking_epoch+1) % (2**32)

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and now < self.last_clock_ns:
            self.pose_history.clear()
            self.gps_history.clear()
            self.reset_track()
            self.pose_time = self.gps_time = self.last_frame = None
            self.plan = self.plan_time = None
        self.last_clock_ns = now

    def info_cb(self, message):
        old = self.camera
        if old is not None and (list(old.k) != list(message.k) or list(old.d) != list(message.d)
                or old.width != message.width or old.height != message.height
                or old.header.frame_id != message.header.frame_id):
            self.reset_track()
            self.invalid("Camera calibration changed; target track reset")
        self.camera = message

    def plan_cb(self, message):
        self.check_clock()
        if (self.plan is None or not message.verified or message.plan_id != self.plan.plan_id
                or (message.land_lat, message.land_lon, message.heading_deg)
                != (self.plan.land_lat, self.plan.land_lon, self.plan.heading_deg)):
            self.reset_track()
        self.plan, self.plan_time = message, time.monotonic()

    def pose_cb(self, message):
        self.check_clock()
        try:
            if message.header.frame_id != self.p["local_frame"]:
                raise ValueError("Aircraft local frame changed")
            q, p = message.pose.orientation, message.pose.position
            quaternion = (q.x, q.y, q.z, q.w)
            quaternion_matrix(quaternion)
            if not np.isfinite([p.x, p.y, p.z]).all():
                raise ValueError("Invalid aircraft position")
            stamp = self.seconds(message.header.stamp)
            if stamp <= 0 or self.pose_history and stamp <= self.pose_history[-1][0]:
                return
            self.pose_history.append((stamp, (p.x, p.y, p.z), quaternion))
            self.pose_time = time.monotonic()
        except ValueError:
            self.pose_history.clear()
            self.pose_time = None
            self.reset_track()
            self.invalid("Aircraft local frame/pose invalid; target track reset")

    def gps_cb(self, message):
        self.check_clock()
        try:
            if message.status.status < 0 or message.position_covariance_type == 0:
                raise ValueError("GPS fix or uncertainty unavailable")
            net_offset_enu(message.latitude, message.longitude, message.latitude, message.longitude)
            covariance = np.asarray(message.position_covariance).reshape(3, 3)[:2, :2]
            if (not np.isfinite(covariance).all() or not np.allclose(covariance, covariance.T)
                    or np.min(np.linalg.eigvalsh(covariance)) < 0):
                raise ValueError("Invalid GPS covariance")
            covariance = covariance + np.eye(2)*self.p["gps_position_std_floor_m"]**2
            stamp = self.seconds(message.header.stamp)
            if stamp <= 0 or self.gps_history and stamp <= self.gps_history[-1][0]:
                return
            self.gps_history.append((stamp, (message.latitude, message.longitude), covariance))
            self.gps_time = time.monotonic()
        except ValueError:
            self.gps_history.clear()
            self.gps_time = None

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def age(self, stamp):
        return (self.get_clock().now().nanoseconds - stamp.sec*10**9 - stamp.nanosec) / 1e9

    def invalid(self, reason, stamp=None):
        message = NetEstimate()
        message.header.frame_id = self.p["body_frame"]
        message.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        message.plan_id = self.plan.plan_id if self.plan else ""
        message.tracking_epoch = self.tracking_epoch
        message.estimation_method = "vertical_plane_ray_filtered"
        message.reason = reason
        self.pub.publish(message)

    def expire(self):
        self.check_clock()
        if self.last_frame is None or time.monotonic()-self.last_frame > self.p["detection_timeout_s"]:
            self.invalid("Detection stream unavailable")

    def cb(self, detection):
        self.check_clock()
        self.last_frame = time.monotonic()
        p = self.p
        try:
            if not detection.valid or not math.isfinite(detection.score) or detection.score < p["min_detection_score"]:
                raise ValueError("Detection invalid or weak")
            if self.seconds(detection.header.stamp) == 0 or not 0 <= self.age(detection.header.stamp) <= p["detection_timeout_s"]:
                raise ValueError("Image detection stale or future-dated")
            camera, plan = self.camera, self.plan
            if camera is None:
                raise ValueError("Waiting for CameraInfo calibration")
            if detection.header.frame_id != p["camera_frame"] or camera.header.frame_id != p["camera_frame"]:
                raise ValueError("Image/CameraInfo frame does not match calibrated camera_frame")
            if camera.distortion_model not in ("plumb_bob", "rational_polynomial"):
                raise ValueError("Use calibrated pinhole/rectified images")
            if (camera.width <= 0 or camera.height <= 0 or detection.image_width != camera.width
                    or detection.image_height != camera.height):
                raise ValueError("Detection/calibration resolution mismatch")
            if not 0 <= detection.center_u < camera.width or not 0 <= detection.center_v < camera.height:
                raise ValueError("Invalid centre pixel")
            if (plan is None or not plan.verified or not plan.plan_id or self.plan_time is None
                    or not 0 <= time.monotonic()-self.plan_time <= 3 or not 0 <= self.age(plan.header.stamp) <= 3):
                raise ValueError("Waiting for fresh verified landing plane")
            for received, history in ((self.pose_time, self.pose_history), (self.gps_time, self.gps_history)):
                if (received is None or not 0 <= time.monotonic()-received <= p["telemetry_timeout_s"]
                        or not history or not 0 <= self.get_clock().now().nanoseconds/1e9-history[-1][0] <= p["telemetry_timeout_s"]):
                    raise ValueError("Aircraft pose/GPS stale or unavailable")
            stamp = self.seconds(detection.header.stamp)
            position, rotation = image_pose(self.pose_history, stamp)
            a, b, weight = sample_bracket(self.gps_history, stamp, max_nearest=.2)
            # Transport each fix to image time using measured local motion.
            # Holding an old GPS-to-net vector would bias range by V*latency.
            position_a, _ = image_pose(self.pose_history, a[0])
            position_b, _ = image_pose(self.pose_history, b[0])
            offset_a = net_offset_enu(*a[1], plan.land_lat, plan.land_lon) - (position-position_a)[:2]
            offset_b = net_offset_enu(*b[1], plan.land_lat, plan.land_lon) - (position-position_b)[:2]
            offset = (1-weight)*offset_a + weight*offset_b
            gps_covariance = (1-weight)*a[2]+weight*b[2]
            body, local_covariance, incidence = plane_measurement(
                (detection.center_u, detection.center_v), camera.k, camera.d,
                p["camera_to_body_rotation"], p["camera_translation_body_m"], rotation,
                offset, plan.heading_deg, gps_covariance, pixel_noise=p["pixel_noise_px"],
                attitude_std_deg=p["attitude_std_deg"], plane_position_std=p["plane_position_std_m"],
                plane_heading_std_deg=p["plane_heading_std_deg"],
                min_incidence=p["min_ray_plane_incidence"], max_range=p["maximum_range_m"])
            local_covariance += np.eye(3)*p["local_position_std_m"]**2
            if np.sqrt(np.max(np.linalg.eigvalsh(local_covariance))) > p["max_position_std_m"]:
                raise ValueError("Ray/plane position uncertainty too high")
            previous = self.tracker.position, self.tracker.covariance, self.tracker.stamp
            try:
                filtered, covariance = self.tracker.update(position+rotation@body, local_covariance, stamp)
                body_position = rotation.T @ (filtered-position)
                if body_position[0] <= 0:
                    raise ValueError("Filtered target not ahead of aircraft")
                body_covariance = rotation.T @ covariance @ rotation
                if np.sqrt(np.max(np.linalg.eigvalsh(body_covariance))) > p["max_position_std_m"]:
                    raise ValueError("Filtered position uncertainty too high")
            except ValueError:
                self.tracker.position, self.tracker.covariance, self.tracker.stamp = previous
                raise
            message = NetEstimate()
            message.header.stamp, message.header.frame_id = detection.header.stamp, p["body_frame"]
            message.valid, message.plan_id = True, plan.plan_id
            message.tracking_epoch = self.tracking_epoch
            message.estimation_method = "vertical_plane_ray_filtered"
            message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, body_position)
            # Centre-only sensing does not measure net orientation.
            message.pose.orientation.w = 1.
            matrix = np.zeros((6, 6))
            matrix[:3, :3] = body_covariance
            matrix[3:, 3:] = np.eye(3)*math.pi**2
            message.covariance = matrix.reshape(36).tolist()
            message.ray_plane_incidence, message.score = incidence, detection.score
            message.reason = "Filtered centre ray / surveyed vertical net plane"
            self.pub.publish(message)
        except (ValueError, TypeError, np.linalg.LinAlgError) as exc:
            self.invalid(str(exc), detection.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = NetPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
