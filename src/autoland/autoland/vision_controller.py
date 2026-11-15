import time
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import VFR_HUD
from autoland_interfaces.msg import NetEstimate, LandingStatus, ControlSetpoint
from .geometry import quaternion_euler, inverse_rotate, interpolate_pose, compensate_target
from .vision_guidance import GuidanceConfig, VisionGuidance
from .ros_util import params, retained, recent, stamped_recent, run


class VisionController(Node):
    def __init__(self):
        super().__init__("vision_controller")
        self.p = params(
            self,
            dict(
                **GuidanceConfig().__dict__,
                vision_timeout_s=0.25,
                telemetry_timeout_s=0.5,
                local_frame="map",
                body_frame="base_link",
                min_vision_score=0.5,
                max_position_std_m=2.0,
                mavros_namespace="/mavros",
            ),
        )
        c = GuidanceConfig(**{k: self.p[k] for k in GuidanceConfig.__dataclass_fields__})
        self.guidance = VisionGuidance(c)
        self.history = deque(maxlen=250)
        self.status = self.net = self.pose = self.velocity = self.air = None
        self.times = {}
        self.last_tick = None
        self.active_plan = None
        self.last_state = None
        ns = self.p["mavros_namespace"].rstrip("/")
        self.pub = self.create_publisher(ControlSetpoint, "/autoland/control_setpoint", 10)
        self.create_subscription(
            LandingStatus, "/autoland/status", lambda m: self.save("status", m), retained()
        )
        self.create_subscription(
            NetEstimate, "/autoland/net_estimate", lambda m: self.save("net", m), 10
        )
        for typ, topic, key in [
            (PoseStamped, "/local_position/pose", "pose"),
            (TwistStamped, "/local_position/velocity_local", "velocity"),
            (VFR_HUD, "/vfr_hud", "air"),
        ]:
            self.create_subscription(
                typ, ns + topic, lambda m, k=key: self.save(k, m), qos_profile_sensor_data
            )
        self.create_timer(0.05, self.tick)

    def save(self, k, m):
        setattr(self, k, m)
        self.times[k] = time.monotonic()
        if k == "pose":
            t = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
            q = m.pose.orientation
            p = m.pose.position
            if self.history and t <= self.history[-1][0]:
                self.history.clear()
            self.history.append((t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    def tick(self):
        output = ControlSetpoint()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.p["local_frame"]
        output.plan_id = self.status.plan_id if self.status else ""
        now = time.monotonic()
        dt = 0.05 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        try:
            if not stamped_recent(self, self.status, self.times.get("status"), 0.3):
                raise ValueError("Supervisor unavailable")
            if self.status.state not in (
                "MISSION_READY",
                "AUTO_APPROACH",
                "HANDOFF_PENDING",
                "VISION_FINAL",
            ):
                raise ValueError("Controller inactive")
            if self.status.plan_id != self.active_plan or (
                self.status.state == "VISION_FINAL" and self.last_state != "VISION_FINAL"
            ):
                self.guidance.reset()
                self.active_plan = self.status.plan_id
            self.last_state = self.status.state
            if (
                not stamped_recent(
                    self, self.net, self.times.get("net"), self.p["vision_timeout_s"]
                )
                or not self.net.valid
            ):
                raise ValueError("Vision unavailable")
            if (
                self.net.header.frame_id != self.p["body_frame"]
                or self.net.score < self.p["min_vision_score"]
            ):
                raise ValueError("Invalid target frame/score")
            if any(
                not 0 <= self.net.covariance[k] <= self.p["max_position_std_m"] ** 2
                for k in (0, 7, 14)
            ):
                raise ValueError("Uncertain target position")
            for key in ("pose", "velocity", "air"):
                if not recent(self.times.get(key), self.p["telemetry_timeout_s"]):
                    raise ValueError("Aircraft telemetry stale")
            if (
                self.pose.header.frame_id != self.p["local_frame"]
                or self.velocity.header.frame_id != self.p["local_frame"]
            ):
                raise ValueError("Local frame mismatch")
            q = self.pose.pose.orientation
            quat = (q.x, q.y, q.z, q.w)
            roll, pitch_ros, yaw = quaternion_euler(*quat)
            v = self.velocity.twist.linear
            body_v = inverse_rotate(quat, (v.x, v.y, v.z))
            p = self.net.pose.position
            image_stamp = self.net.header.stamp.sec + self.net.header.stamp.nanosec / 1e9
            image_pose = interpolate_pose(list(self.history), image_stamp)
            cp = self.pose.pose.position
            target = compensate_target((p.x, p.y, p.z), image_pose, ((cp.x, cp.y, cp.z), quat))
            (output.roll_rad, output.pitch_rad, output.thrust), _, _ = self.guidance.update(
                target, body_v, self.air.airspeed, roll, -pitch_ros, dt
            )
            output.yaw_enu_rad = yaw
            output.valid = True
            output.reason = "Interception prototype target"
        except ValueError as exc:
            self.guidance.reset()
            output.reason = str(exc)
        self.pub.publish(output)


def main(args=None):
    run(VisionController, args)
