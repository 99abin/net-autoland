import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from autoland_interfaces.msg import NetDetection, NetEstimate
from .pose_estimation import estimate_pose


class NetPosition(Node):
    def __init__(self):
        super().__init__("net_pos")
        defaults = dict(
            camera_info_topic="/camera/camera_info",
            camera_frame="camera_optical_frame",
            body_frame="base_link",
            net_width_m=5.5,
            net_height_m=2.0,
            pixel_noise_px=2.0,
            max_reprojection_px=4.0,
            detection_timeout_s=0.25,
            min_detection_score=0.5,
            max_position_std_m=2.0,
            camera_to_body_rotation=[0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0],
            camera_translation_body_m=[0.0, 0.0, 0.0],
            target_offset_net_m=[0.0, 0.0, 0.0],
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.camera = None
        self.last_frame = None
        self.pub = self.create_publisher(NetEstimate, "/autoland/net_estimate", 10)
        self.create_subscription(
            CameraInfo, self.p["camera_info_topic"], self.info_cb, qos_profile_sensor_data
        )
        self.create_subscription(NetDetection, "/autoland/net_detection", self.cb, 10)
        self.create_timer(0.05, self.expire)

    def info_cb(self, m):
        self.camera = m

    def age(self, stamp):
        return (self.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9

    def invalid(self, reason, stamp=None):
        m = NetEstimate()
        m.header.frame_id = self.p["body_frame"]
        m.reason = reason
        m.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        self.pub.publish(m)

    def expire(self):
        if (
            self.last_frame is None
            or time.monotonic() - self.last_frame > self.p["detection_timeout_s"]
        ):
            self.invalid("Detection stream unavailable")

    def cb(self, d):
        self.last_frame = time.monotonic()
        p = self.p
        try:
            if not d.valid or d.score < p["min_detection_score"]:
                raise ValueError("Detection invalid or weak")
            if d.header.stamp.sec == 0 and d.header.stamp.nanosec == 0:
                raise ValueError("Image timestamp missing")
            if not 0 <= self.age(d.header.stamp) <= p["detection_timeout_s"]:
                raise ValueError("Image detection stale or future-dated")
            if self.camera is None:
                raise ValueError("Waiting for CameraInfo calibration")
            if (
                d.header.frame_id != p["camera_frame"]
                or self.camera.header.frame_id != p["camera_frame"]
            ):
                raise ValueError("Image/CameraInfo frame does not match calibrated camera_frame")
            if self.camera.distortion_model not in ("plumb_bob", "rational_polynomial"):
                raise ValueError(
                    "Use calibrated pinhole images; fisheye images must be rectified first"
                )
            k = np.asarray(self.camera.k).reshape(3, 3)
            if k[0, 0] <= 0 or self.camera.width == 0:
                raise ValueError("CameraInfo is uncalibrated")
            pos, quat, cov, error = estimate_pose(
                d.corners,
                k,
                self.camera.d,
                p["net_width_m"],
                p["net_height_m"],
                p["camera_to_body_rotation"],
                p["camera_translation_body_m"],
                p["target_offset_net_m"],
                p["pixel_noise_px"],
                p["max_reprojection_px"],
            )
            if np.sqrt(np.max(np.diag(cov))) > p["max_position_std_m"]:
                raise ValueError("Position uncertainty too high")
            m = NetEstimate()
            m.header.stamp = d.header.stamp
            m.header.frame_id = p["body_frame"]
            m.valid = True
            m.score = d.score
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
            (
                m.pose.orientation.x,
                m.pose.orientation.y,
                m.pose.orientation.z,
                m.pose.orientation.w,
            ) = map(float, quat)
            # Unknown orientation covariance is conservatively large; controller uses position only.
            matrix = np.zeros((6, 6))
            matrix[:3, :3] = cov
            matrix[3:, 3:] = np.eye(3) * math.pi**2
            m.covariance = matrix.reshape(36).tolist()
            m.reprojection_error_px = error
            m.reason = "Calibrated planar estimate"
            self.pub.publish(m)
        except Exception as exc:
            self.invalid(str(exc), d.header.stamp)


def main(args=None):
    rclpy.init(args=args)
    node = NetPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
