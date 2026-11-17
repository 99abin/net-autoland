import math
import time
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import VfrHud
from autoland_interfaces.msg import NetEstimate, LandingStatus, ControlSetpoint
from .geometry import quaternion_euler, inverse_rotate, interpolate_pose, rotate, finite, rotate_covariance
from .vision_guidance import GuidanceConfig, VisionGuidance
from .terminal import TerminalConfig, TERMINAL_STATES, ZERO_THRUST_STATES, crossing, commit_ready
from .ros_util import params, retained, recent, stamped_recent, run


class VisionController(Node):
    def __init__(self):
        super().__init__("vision_controller")
        self.p = params(
            self,
            dict(
                **GuidanceConfig().__dict__,
                **TerminalConfig().__dict__,
                terminal_enabled=False,
                vision_timeout_s=0.25,
                # Zero retains the stationary target for the active handoff;
                # positive values bound both wall and ROS observation age.
                target_hold_timeout_s=0.0,
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
        self.terminal_config = TerminalConfig(**{k: self.p[k] for k in TerminalConfig.__dataclass_fields__})
        self.terminal_config.validate()
        self.last_yaw = 0.
        if not math.isfinite(self.p["target_hold_timeout_s"]) or self.p["target_hold_timeout_s"] < 0:
            raise ValueError("Target hold timeout must be finite and nonnegative")
        self.clear_target()
        self.last_clock_ns = None
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
            (VfrHud, "/vfr_hud", "air"),
        ]:
            self.create_subscription(
                typ, ns + topic, lambda m, k=key: self.save(k, m), qos_profile_sensor_data
            )
        self.create_timer(0.05, self.tick)

    def clear_target(self):
        self.target_world = None
        self.target_stamp_ns = None
        self.target_received = None
        self.target_epoch = None
        self.target_cov_world = None

    def save(self, k, m):
        setattr(self, k, m)
        self.times[k] = time.monotonic()
        if k == "pose":
            t = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
            q = m.pose.orientation
            p = m.pose.position
            if self.history and t < self.history[-1][0]:
                self.history.clear()
                self.clear_target()
                self.guidance.reset()
            elif self.history and t == self.history[-1][0]:
                self.history.pop()
            if m.header.frame_id != self.p["local_frame"]:
                return
            self.history.append((t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    def target(self, current_pose):
        """Store accepted observations in ENU; never freeze an old body vector."""
        if (self.net is not None and self.net.plan_id == self.status.plan_id
                and self.target_epoch is not None and self.net.tracking_epoch != self.target_epoch):
            self.clear_target()
        observed = False
        problem = "Vision unavailable"
        try:
            m = self.net
            if not stamped_recent(self, m, self.times.get("net"), self.p["vision_timeout_s"]) or not m.valid:
                raise ValueError(problem)
            if m.header.frame_id != self.p["body_frame"] or not m.score >= self.p["min_vision_score"]:
                raise ValueError("Invalid target frame/score")
            if m.plan_id != self.status.plan_id:
                raise ValueError("Target belongs to a different landing plane")
            if any(not 0 <= m.covariance[k] <= self.p["max_position_std_m"] ** 2 for k in (0, 7, 14)):
                raise ValueError("Uncertain target position")
            point = m.pose.position
            if not finite(point.x, point.y, point.z):
                raise ValueError("Non-finite target position")
            stamp_ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
            if self.target_stamp_ns is not None and stamp_ns < self.target_stamp_ns:
                raise ValueError("Out-of-order target observation")
            if stamp_ns != self.target_stamp_ns:
                image_position, image_q = interpolate_pose(list(self.history), stamp_ns / 1e9)
                offset = rotate(image_q, (point.x, point.y, point.z))
                self.target_world = tuple(a + b for a, b in zip(image_position, offset))
                self.target_stamp_ns = stamp_ns
                self.target_received = self.times["net"]
                self.target_epoch = m.tracking_epoch
                covariance = [[m.covariance[6*i+j] for j in range(3)] for i in range(3)]
                if not finite(*(v for row in covariance for v in row)):
                    self.clear_target()
                    raise ValueError("Non-finite target covariance")
                self.target_cov_world = rotate_covariance(covariance, image_q)
            observed = True
        except ValueError as exc:
            problem = str(exc)
        active = self.status.state in ("HANDOFF_PENDING", "VISION_FINAL", *TERMINAL_STATES)
        if self.target_world is None or (not observed and not active):
            raise ValueError(problem)
        age = (self.get_clock().now().nanoseconds - self.target_stamp_ns) / 1e9
        if age < 0:
            self.clear_target()
            raise ValueError("Target observation is future-dated")
        timeout = self.p["target_hold_timeout_s"]
        if not observed and timeout > 0 and max(age, time.monotonic() - self.target_received) > timeout:
            raise ValueError("Last known target expired")
        position, quat = current_pose
        target = inverse_rotate(quat, tuple(a - b for a, b in zip(self.target_world, position)))
        return target, not observed, age

    def tick(self):
        output = ControlSetpoint()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.p["local_frame"]
        output.plan_id = self.status.plan_id if self.status else ""
        now = time.monotonic()
        dt = 0.05 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        clock_ns = self.get_clock().now().nanoseconds
        if self.last_clock_ns is not None and clock_ns < self.last_clock_ns:
            self.clear_target()
            self.history.clear()
            self.guidance.reset()
        self.last_clock_ns = clock_ns
        try:
            if not stamped_recent(self, self.status, self.times.get("status"), 0.3):
                raise ValueError("Supervisor unavailable")
            if self.status.state not in (
                "MISSION_READY",
                "AUTO_APPROACH",
                "HANDOFF_PENDING",
                "VISION_FINAL",
                *TERMINAL_STATES,
            ):
                self.clear_target()
                raise ValueError("Controller inactive")
            if self.status.plan_id != self.active_plan:
                self.clear_target()
            if self.status.plan_id != self.active_plan or (
                self.status.state == "VISION_FINAL" and self.last_state != "VISION_FINAL"
            ):
                self.guidance.reset()
                self.active_plan = self.status.plan_id
            self.last_state = self.status.state
            terminal = self.status.state in TERMINAL_STATES
            if terminal and not self.p["terminal_enabled"]:
                raise ValueError("Terminal control disabled")
            output.terminal_mode = terminal
            if terminal and self.status.state in ZERO_THRUST_STATES:
                previous = self.guidance.previous or (0., 0., 0.)
                output.roll_rad, output.pitch_rad = previous[:2]
                output.yaw_enu_rad, output.thrust = self.last_yaw, 0.
                output.valid = True
                output.reason = "Terminal zero-thrust hold; operator/capture handling required"
                self.pub.publish(output)
                return
            for key in ("pose", "velocity", "air"):
                if not recent(self.times.get(key), self.p["telemetry_timeout_s"]):
                    raise ValueError("Aircraft telemetry stale")
            if any(not stamped_recent(self, getattr(self, key), self.times.get(key),
                                      self.p["telemetry_timeout_s"]) for key in ("pose", "velocity")):
                raise ValueError("Aircraft pose/velocity timestamp stale")
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
            cp = self.pose.pose.position
            target, held, age = self.target(((cp.x, cp.y, cp.z), quat))
            output.target_held = held
            output.target_age_s = age
            time_to_target, error_y, error_z = crossing(target, body_v)
            output.forward_range_m, output.time_to_target_s = target[0], time_to_target
            output.crossing_error_y_m, output.crossing_error_z_m = error_y, error_z
            covariance = rotate_covariance(self.target_cov_world, quat, inverse=True)
            output.position_std_y_m = math.sqrt(max(covariance[1][1], 0.))
            output.position_std_z_m = math.sqrt(max(covariance[2][2], 0.))
            if terminal and target[0] <= 0:
                output.target_passed = True
                previous = self.guidance.previous or (roll, -pitch_ros, 0.)
                output.roll_rad, output.pitch_rad = previous[:2]
                output.thrust, output.yaw_enu_rad = 0., self.last_yaw
                output.valid = True
                output.reason = "Target plane passed; holding direction with zero thrust"
                self.pub.publish(output)
                return
            (output.roll_rad, output.pitch_rad, output.thrust), _, _ = self.guidance.update(
                target, body_v, self.air.airspeed, roll, -pitch_ros, dt, terminal=terminal
            )
            limit = min(self.guidance.c.roll_limit_deg, self.guidance.c.terminal_roll_limit_deg)
            output.terminal_ready = (self.p["terminal_enabled"] and not terminal
                and abs(math.degrees(roll)) <= limit and abs(math.degrees(output.roll_rad)) <= limit
                and commit_ready(self.terminal_config, time_to_target, error_y, error_z,
                                 output.position_std_y_m, output.position_std_z_m, age))
            output.yaw_enu_rad = yaw
            self.last_yaw = yaw
            output.valid = True
            output.reason = ("Committed terminal guidance" if terminal else
                             "Tracking last known net position" if held else "Interception prototype target")
        except ValueError as exc:
            self.guidance.reset()
            output.reason = str(exc)
        self.pub.publish(output)


def main(args=None):
    run(VisionController, args)
