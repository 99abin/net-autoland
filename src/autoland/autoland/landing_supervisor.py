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
from .ros_util import params, retained, recent, stamped_recent, stamp_age, profile_values, reordered_profile_heartbeat, run
from .state_conditions import HandoffConfig, Supervisor, handoff_ready, in_handoff_window
from .terminal import TerminalConfig, TERMINAL_STATES, commit_ready, net_contact_pair
from .timing_diagnostics import TimingRecorder


class LandingSupervisor(Node):
    def __init__(self):
        super().__init__("landing_supervisor")
        self.timing = TimingRecorder(self)
        trace = self.timing.wrap
        self.p = params(
            self,
            dict(
                handoff_enabled=False,
                terminal_enabled=False,
                simulation_contact_enabled=False,
                **TerminalConfig().__dict__,
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
        self.terminal_config = TerminalConfig(**{k: self.p[k] for k in TerminalConfig.__dataclass_fields__})
        self.terminal_config.validate()
        self.commit_since = None
        self.net_contact = False
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
            ParameterProfileStatus, "/autoland/parameter_profile", trace("profile", self.profile_cb), retained()
        )
        self.create_subscription(LandingPlan, "/autoland/landing_plan", trace("plan", self.plan_cb), retained())
        self.create_subscription(String, "/autoland/control_ownership", trace("owner", self.owner_cb), 10)
        self.create_subscription(
            NetEstimate, "/autoland/net_estimate", trace("net", lambda m: self.save("net", m)), 10
        )
        self.create_subscription(
            ControlSetpoint, "/autoland/control_setpoint", trace("preview", lambda m: self.save("preview", m)), 10
        )
        ns = self.p["mavros_namespace"].rstrip("/")
        for typ, topic, key, qos in [
            (State, "/state", "fcu", 10),
            (NavSatFix, "/global_position/global", "gps", qos_profile_sensor_data),
            (TwistStamped, "/local_position/velocity_local", "velocity", qos_profile_sensor_data),
        ]:
            self.create_subscription(typ, ns + topic, trace(key, lambda m, k=key: self.save(k, m)), qos)
        self.create_subscription(
            Float64, ns + "/global_position/rel_alt", trace("altitude", self.alt_cb), qos_profile_sensor_data
        )
        self.create_subscription(
            WaypointList, ns + "/mission/waypoints", trace("mission", self.mission_cb), retained()
        )
        self.create_service(Trigger, "~/enable", trace("enable", self.enable_cb))
        self.create_service(Trigger, "~/reset", trace("reset", self.reset_cb))
        self.create_service(Trigger, "~/abort", trace("abort", self.abort_cb))
        self.create_service(Trigger, "~/confirm_capture", trace("capture", self.capture_cb))
        self.create_timer(0.05, trace("tick", self.tick))
        if self.p["simulation_contact_enabled"]:
            from ros_gz_interfaces.msg import Contacts
            self.create_subscription(Contacts, "/simulation/net_contacts", trace("contact", self.contact_cb), 10)

    def profile_cb(self, msg):
        if reordered_profile_heartbeat(self,msg,self.profile,self.profile_time):
            return
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
        if (self.plan_locked and self.engine.state not in ("ABORT", "TERMINAL_FAULT")
                and (not m.verified or m.plan_id != self.plan_locked)):
            reason = ("Landing plan invalidated: " + m.reason if not m.verified
                      else "Mission changed during landing")
            self.engine.abort(reason)
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
            self.commit_since = None
            self.net_contact = False
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
        res.success = self.engine.state in ("VISION_FINAL", "NET_CONTACT") and stopped
        if self.p["terminal_enabled"]:
            res.success = (res.success and self.engine.state == "NET_CONTACT" and self.net_contact
                and self.plan is not None and self.alt is not None
                and recent(self.times.get("alt"), 1)
                and abs(self.alt - self.plan.land_alt) <= self.p["terminal_net_half_height_m"])
        if res.success:
            self.engine.state = "COMPLETE"
            self.engine.enabled = False
            self.engine.reason = "Operator confirmed stationary net capture"
        res.message = (
            self.engine.reason
            if res.success
            else "Need stationary capture confirmation (terminal mode also requires net contact and target height)"
        )
        return res

    def contact_cb(self, message):
        if (not self.p["simulation_contact_enabled"] or not self.engine.terminal_committed
                or self.engine.state not in ("TERMINAL_COMMITTED", "TERMINAL_FAULT", "MISSED_NET")
                or not self.fcu or not self.fcu.armed or self.fcu.mode != "GUIDED"
                or not recent(self.times.get("fcu"), 3)
                or not 0 <= stamp_age(self, message.header.stamp) <= .25):
            return
        if any(net_contact_pair(contact.collision1.name, contact.collision2.name)
               for contact in message.contacts):
            self.net_contact = True
            self.engine.state = "NET_CONTACT"
            self.engine.enabled = False
            self.engine.reason = "Simulator net contact observed; zero thrust; capture unconfirmed"

    def request(self, owner):
        if not self.client.service_is_ready():
            if owner == "VISION":
                self.engine.abort("FCU interface service unavailable")
            return
        req = SetControlOwnership.Request()
        req.owner = owner
        req.plan_id = self.plan_locked or ""
        self.timing.record("ownership_request_sent", owner=owner, plan_id=req.plan_id)
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
        target_ok = (
            stamped_recent(self, self.preview, self.times.get("preview"), self.p["vision_timeout_s"])
            and self.preview.valid
            and self.preview.plan_id == self.plan_locked
            and math.isfinite(self.preview.target_age_s)
            and self.preview.target_age_s >= 0
        )
        if self.engine.state in ("HANDOFF_PENDING", "VISION_FINAL", *TERMINAL_STATES):
            # After authorization the controller may track a retained static
            # target. Raw detector jitter must not abort a healthy command stream.
            return target_ok
        m = self.net
        ok = (
            stamped_recent(self, m, self.times.get("net"), self.p["vision_timeout_s"])
            and m.valid
            and m.plan_id == self.plan_locked
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
        return (target_ok and not self.preview.target_held
                and time.monotonic() - self.stable_since >= self.p["vision_stable_s"])

    def tick(self):
        ready = False
        handoff_window = False
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
                handoff_window = in_handoff_window(
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
            handoff_window=handoff_window,
        )
        if action:
            self.request(action)
        if self.engine.state == "VISION_FINAL" and self.p["terminal_enabled"]:
            m = self.preview
            acceptable = (vision and m.terminal_ready and not m.terminal_mode
                and commit_ready(self.terminal_config, m.time_to_target_s,
                    m.crossing_error_y_m, m.crossing_error_z_m,
                    m.position_std_y_m, m.position_std_z_m, m.target_age_s))
            if acceptable:
                if self.commit_since is None:
                    self.commit_since = time.monotonic()
                if time.monotonic()-self.commit_since >= self.p["terminal_commit_stable_s"]:
                    self.engine.commit()
            else:
                self.commit_since = None
            if (self.engine.state == "VISION_FINAL" and m is not None and m.valid
                    and 0 < m.time_to_target_s <= self.p["terminal_decision_ttc_s"]):
                self.engine.abort("Terminal commitment rejected before decision boundary")
                self.request("ABORT")
        if self.engine.state == "TERMINAL_COMMITTED" and self.preview is not None and self.preview.target_passed:
            self.engine.state = "MISSED_NET"
            self.engine.enabled = False
            self.engine.reason = "Target plane passed without observed contact; zero-thrust direction hold"
        if self.engine.state in ("HANDOFF_PENDING", "VISION_FINAL", "TERMINAL_COMMITTED") and not vision:
            self.engine.abort("Control target unavailable (including retained net target)")
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
                self.timing.record("ownership_response", accepted=result.accepted, message=result.message)
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
        msg.monitoring_enabled = self.engine.enabled
        msg.net_contact = self.net_contact
        msg.control_owner = "VISION" if owned else "ARDUPILOT"
        msg.plan_id = self.plan_locked or (self.plan.plan_id if self.plan else "")
        msg.parameter_profile_id = self.profile.profile_id if self.profile else ""
        msg.reason = self.engine.reason
        self.pub.publish(msg)


def main(args=None):
    run(LandingSupervisor, args)
