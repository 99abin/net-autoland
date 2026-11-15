import math
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from mavros_msgs.msg import State, WaypointList
from autoland_interfaces.msg import (
    LandingPlan,
    LandingStatus,
    NetEstimate,
    ControlSetpoint,
    ParameterProfileStatus,
)
from autoland_interfaces.srv import SetControlOwnership
from .geometry import finite, leg_position, wrap
from .ros_util import params, retained, recent, stamped_recent, stamp_age, profile_values, run
from .state_conditions import HandoffConfig, Supervisor, handoff_ready, in_handoff_window


class LandingSupervisor(Node):
    def __init__(self):
        super().__init__("landing_supervisor")
        self.p = params(
            self,
            dict(
                handoff_enabled=False,
                telemetry_timeout_s=1.0,
                vision_timeout_s=0.25,
                vision_stable_s=0.5,
                min_vision_score=0.5,
                max_position_std_m=2.0,
                handoff_timeout_s=3.0,
                local_frame="map",
                body_frame="base_link",
                mavros_namespace="/mavros",
                **HandoffConfig().__dict__,
            ),
        )
        self.config = HandoffConfig(**{k: self.p[k] for k in HandoffConfig.__dataclass_fields__})
        self.config.validate()
        self.profile = None
        self.profile_time = None
        self.engine = Supervisor()
        self.plan = None
        self.plan_locked = None
        self.fcu = None
        self.owner = "NONE"
        self.gps = self.velocity = self.net = self.preview = None
        self.alt = None
        self.current = -1
        self.times = {}
        self.stable_since = None
        self.deadline = None
        self.future = None
        self.pub = self.create_publisher(LandingStatus, "/autoland/status", retained())
        self.client = self.create_client(SetControlOwnership, "/autoland/set_control_ownership")
        self.create_subscription(
            ParameterProfileStatus, "/autoland/parameter_profile", self.profile_cb, retained()
        )
        self.create_subscription(LandingPlan, "/autoland/landing_plan", self.plan_cb, retained())
        self.create_subscription(String, "/autoland/control_ownership", self.owner_cb, 10)
        self.create_subscription(
            NetEstimate, "/autoland/net_estimate", lambda m: self.save("net", m), 10
        )
        self.create_subscription(
            ControlSetpoint, "/autoland/control_setpoint", lambda m: self.save("preview", m), 10
        )
        ns = self.p["mavros_namespace"].rstrip("/")
        for typ, topic, key, qos in [
            (State, "/state", "fcu", 10),
            (NavSatFix, "/global_position/global", "gps", qos_profile_sensor_data),
            (TwistStamped, "/local_position/velocity_local", "velocity", qos_profile_sensor_data),
        ]:
            self.create_subscription(typ, ns + topic, lambda m, k=key: self.save(k, m), qos)
        self.create_subscription(
            Float64, ns + "/global_position/rel_alt", self.alt_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            WaypointList, ns + "/mission/waypoints", self.mission_cb, retained()
        )
        self.create_service(Trigger, "~/enable", self.enable_cb)
        self.create_service(Trigger, "~/reset", self.reset_cb)
        self.create_service(Trigger, "~/abort", self.abort_cb)
        self.create_service(Trigger, "~/confirm_capture", self.capture_cb)
        self.create_timer(0.05, self.tick)

    def profile_cb(self, msg):
        self.profile = msg
        self.profile_time = time.monotonic()
        values = profile_values(self, msg, self.profile_time)
        if values is not None and not self.engine.enabled:
            self.config.flare_alt_m = float(values["LAND_FLARE_ALT"])
            self.config.flare_time_s = float(values["LAND_FLARE_SEC"])

    def save(self, k, m):
        setattr(self, k, m)
        self.times[k] = time.monotonic()

    def plan_cb(self, m):
        if self.plan_locked and (not m.verified or m.plan_id != self.plan_locked):
            self.engine.abort("Mission changed during landing")
            self.request("ABORT")
        self.plan = m
        self.times["plan"] = time.monotonic()

    def owner_cb(self, m):
        self.owner = m.data
        self.times["owner"] = time.monotonic()

    def alt_cb(self, m):
        self.alt = m.data
        self.times["alt"] = time.monotonic()

    def mission_cb(self, m):
        self.current = m.current_seq
        self.times["mission"] = time.monotonic()

    def verified(self):
        return (
            profile_values(self, self.profile, self.profile_time) is not None
            and self.plan is not None
            and self.plan.parameter_profile_id == self.profile.profile_id
            and recent(self.times.get("plan"), 3)
            and self.plan.verified
            and bool(self.plan.plan_id)
        )

    def enable_cb(self, req, res):
        res.success = self.telemetry_ok() and self.engine.enable(self.verified())
        if res.success:
            self.plan_locked = self.plan.plan_id
        res.message = (
            self.engine.reason
            if res.success
            else "Need verified mission, fresh telemetry, and reset state"
        )
        return res

    def reset_cb(self, req, res):
        res.success = (
            self.fcu is not None and recent(self.times.get("fcu"), 3) and not self.fcu.armed
        )
        if res.success:
            self.request("RELEASE")
            self.engine = Supervisor()
            self.plan_locked = None
            self.deadline = None
            self.stable_since = None
        res.message = "Reset" if res.success else "Reset requires fresh disarmed FCU"
        return res

    def abort_cb(self, req, res):
        self.engine.abort("Operator requested abort")
        self.request("ABORT")
        res.success = True
        res.message = "Abort requested; verify actual FCU mode"
        return res

    def capture_cb(self, req, res):
        stopped = (
            self.velocity is not None
            and recent(self.times.get("velocity"), 1)
            and math.hypot(self.velocity.twist.linear.x, self.velocity.twist.linear.y) < 1
        )
        res.success = self.engine.state == "VISION_FINAL" and stopped
        if res.success:
            self.engine.state = "COMPLETE"
            self.engine.enabled = False
            self.engine.reason = "Operator confirmed stationary net capture"
        res.message = (
            self.engine.reason
            if res.success
            else "Need VISION_FINAL and fresh ground speed below 1 m/s"
        )
        return res

    def request(self, owner):
        if not self.client.service_is_ready():
            if owner == "VISION":
                self.engine.abort("FCU interface service unavailable")
            return
        req = SetControlOwnership.Request()
        req.owner = owner
        req.plan_id = self.plan_locked or ""
        self.future = self.client.call_async(req)
        if owner == "VISION":
            self.deadline = time.monotonic() + self.p["handoff_timeout_s"]

    def telemetry_ok(self):
        t = self.p["telemetry_timeout_s"]
        if not all(
            recent(self.times.get(k), 3 if k in ("fcu", "mission") else t)
            for k in ("fcu", "gps", "velocity", "alt")
        ):
            return False
        if not all(0 <= stamp_age(self, m.header.stamp) <= t for m in (self.gps, self.velocity)):
            return False
        if (
            self.current < 0
            or self.gps.status.status < 0
            or self.velocity.header.frame_id != self.p["local_frame"]
        ):
            return False
        v = self.velocity.twist.linear
        return self.fcu.connected and finite(
            self.gps.latitude, self.gps.longitude, self.alt, v.x, v.y, v.z
        )

    def vision_ready(self):
        m = self.net
        ok = (
            stamped_recent(self, m, self.times.get("net"), self.p["vision_timeout_s"])
            and m.valid
            and m.header.frame_id == self.p["body_frame"]
            and m.score >= self.p["min_vision_score"]
            and all(
                math.isfinite(m.covariance[k])
                and 0 <= m.covariance[k] <= self.p["max_position_std_m"] ** 2
                for k in (0, 7, 14)
            )
        )
        if not ok:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = time.monotonic()
        target_ok = (
            stamped_recent(
                self, self.preview, self.times.get("preview"), self.p["vision_timeout_s"]
            )
            and self.preview.valid
            and self.preview.plan_id == self.plan_locked
        )
        return target_ok and time.monotonic() - self.stable_since >= self.p["vision_stable_s"]

    def tick(self):
        ready = False
        missed_window = False
        telemetry = self.telemetry_ok()
        vision = self.vision_ready()
        if telemetry and self.verified():
            p = self.plan
            v = self.velocity.twist.linear
            try:
                progress, cross, bearing, _ = leg_position(
                    p.approach_lat,
                    p.approach_lon,
                    p.land_lat,
                    p.land_lon,
                    self.gps.latitude,
                    self.gps.longitude,
                )
                track = math.atan2(v.x, v.y)
                missed_window = in_handoff_window(
                    self.config, self.alt - p.land_alt, -v.z, progress
                )
                ready = handoff_ready(
                    self.config,
                    height=self.alt - p.land_alt,
                    sink=-v.z,
                    progress=progress,
                    cross=cross,
                    track_error=wrap(track - bearing),
                    net_range=self.net.pose.position.x if self.net else math.inf,
                    vision_ready=vision,
                )
            except ValueError:
                ready = False
        owned = self.owner == "VISION" and recent(self.times.get("owner"), 0.5)
        action = self.engine.step(
            verified=self.verified(),
            telemetry_ok=telemetry,
            mode=self.fcu.mode if self.fcu else "",
            armed=self.fcu.armed if self.fcu else False,
            landing_active=self.verified() and self.current == self.plan.land_index,
            ready=ready,
            allow_handoff=self.p["handoff_enabled"],
            owned=owned,
            missed_window=missed_window,
        )
        if action:
            self.request(action)
        if self.engine.state in ("HANDOFF_PENDING", "VISION_FINAL") and not vision:
            self.engine.abort("Vision or control target unavailable")
            self.request("ABORT")
        if (
            self.engine.state == "HANDOFF_PENDING"
            and self.deadline
            and time.monotonic() > self.deadline
        ):
            self.engine.abort("GUIDED confirmation timeout")
            self.request("ABORT")
        if self.future is not None and self.future.done():
            failed = None
            try:
                result = self.future.result()
                if not result.accepted and self.engine.state == "HANDOFF_PENDING":
                    failed = result.message
            except Exception as exc:
                if self.engine.state == "HANDOFF_PENDING":
                    failed = str(exc)
            self.future = None
            if failed is not None:
                self.engine.abort(failed)
                self.request("ABORT")
        msg = LandingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.engine.state
        msg.control_owner = "VISION" if owned else "ARDUPILOT"
        msg.plan_id = self.plan_locked or (self.plan.plan_id if self.plan else "")
        msg.parameter_profile_id = self.profile.profile_id if self.profile else ""
        msg.reason = self.engine.reason
        self.pub.publish(msg)


def main(args=None):
    run(LandingSupervisor, args)
