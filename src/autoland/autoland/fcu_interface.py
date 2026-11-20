"""Single flight-command writer. Preview by default; never arms the aircraft."""

import math
import time
import copy
import json
from rclpy.node import Node
from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode
from std_msgs.msg import String
from autoland_interfaces.msg import LandingStatus, ControlSetpoint, ParameterProfileStatus
from autoland_interfaces.srv import SetControlOwnership
from .geometry import euler_quaternion, quaternion_euler, finite
from .ros_util import params, retained, recent, stamp_age, stamped_recent, profile_values, reordered_profile_heartbeat, run
from .terminal import TERMINAL_STATES, ZERO_THRUST_STATES
from .timing_diagnostics import TimingRecorder, command_key

# ArduPlane-specific partial quaternion interpretation in handle_set_attitude_target:
# inverted axis bits select roll/pitch; IGNORE_ATTITUDE suppresses the all-angle
# path; IGNORE_YAW_RATE prevents external yaw/rudder override. Thrust is included.
# This is intentionally NOT the generic full-quaternion mask 7 used for Copter.
ARDUPLANE_ROLL_PITCH_THRUST_MASK = 128 | 4


def command_in_envelope(cmd, p, terminal=False):
    return (
        cmd.valid
        and finite(cmd.roll_rad, cmd.pitch_rad, cmd.yaw_enu_rad, cmd.thrust, cmd.target_age_s)
        and cmd.target_age_s >= 0
        and abs(cmd.roll_rad) <= math.radians(min(p["roll_limit_deg"], p["terminal_roll_limit_deg"]) if terminal else p["roll_limit_deg"])
        and math.radians(p["pitch_min_deg"]) <= cmd.pitch_rad <= math.radians(p["pitch_max_deg"])
        and (0. if terminal else p["throttle_min"]) <= cmd.thrust <= p["throttle_max"]
    )


class FCUInterface(Node):
    def __init__(self):
        super().__init__("fcu_interface")
        self.timing = TimingRecorder(self)
        trace = self.timing.wrap
        self.p = params(
            self,
            dict(
                live_output=False,
                airborne_preparation_enabled=False,
                terminal_enabled=False,
                terminal_roll_limit_deg=5.0,
                partial_attitude_verified=False,
                mavros_namespace="/mavros",
                local_frame="map",
                abort_mode="RTL",
                status_timeout_s=0.3,
                command_timeout_s=0.2,
                mode_timeout_s=3.0,
                roll_limit_deg=20.0,
                pitch_min_deg=-8.0,
                pitch_max_deg=12.0,
                throttle_min=0.15,
                throttle_max=0.7,
                maximum_roll_step_deg=15.0,
                maximum_pitch_step_deg=12.0,
            ),
        )
        if not (
            finite(
                self.p["roll_limit_deg"],
                self.p["pitch_min_deg"],
                self.p["pitch_max_deg"],
                self.p["throttle_min"],
                self.p["throttle_max"],
            )
            and 0 < self.p["roll_limit_deg"] < 45
            and finite(self.p["terminal_roll_limit_deg"])
            and 0 < self.p["terminal_roll_limit_deg"] <= self.p["roll_limit_deg"]
            and -30 < self.p["pitch_min_deg"] < self.p["pitch_max_deg"] < 30
            and 0 <= self.p["throttle_min"] < self.p["throttle_max"] <= 1
        ):
            raise ValueError("Invalid FCU command envelope")
        if self.p["abort_mode"] not in ("RTL", "LOITER"):
            raise ValueError("Recovery mode must be RTL or LOITER")
        ns = self.p["mavros_namespace"].rstrip("/")
        self.mode_client = self.create_client(SetMode, ns + "/set_mode")
        self.pub = self.create_publisher(AttitudeTarget, ns + "/setpoint_raw/attitude", 10)
        self.preview_pub = self.create_publisher(AttitudeTarget, "/autoland/attitude_preview", 10)
        self.owner_pub = self.create_publisher(String, "/autoland/control_ownership", 10)
        self.create_subscription(State, ns + "/state", trace("state", self.state_cb), 10)
        from rclpy.qos import qos_profile_sensor_data

        self.create_subscription(
            PoseStamped, ns + "/local_position/pose", trace("pose", self.pose_cb), qos_profile_sensor_data
        )
        self.create_subscription(LandingStatus, "/autoland/status", trace("status", self.status_cb), retained())
        self.create_subscription(ControlSetpoint, "/autoland/control_setpoint", trace("command", self.command_cb), 10)
        self.create_service(SetControlOwnership, "/autoland/set_control_ownership", trace("ownership", self.owner_cb))
        self.profile = None
        self.profile_time = None
        self.active_profile_id = None
        self.create_subscription(
            ParameterProfileStatus, "/autoland/parameter_profile", trace("profile", self.profile_cb), retained()
        )
        self.pose = None
        self.pose_time = None
        self.state = self.status = self.command = None
        self.state_time = self.status_time = self.command_time = None
        self.owner = "NONE"
        self.plan_id = ""
        self.future = None
        self.requested_mode = None
        self.deadline = None
        self.last_sent = None
        self.last_sent_time = None
        self.capture_target = None
        self.terminal_latched = False
        self.terminal_fault = False
        self.fault_reason = None
        self.preparation = None
        self.preparation_time = None
        self.preparation_failed = False
        if self.p['airborne_preparation_enabled']:
            self.create_subscription(String,'/autoland/airborne_preparation',trace("preparation",self.preparation_cb),retained())
        self.create_timer(0.05, trace("tick", self.tick))

    def profile_cb(self, msg):
        if reordered_profile_heartbeat(self,msg,self.profile,self.profile_time):
            return
        self.profile = msg
        self.profile_time = time.monotonic()

    def preparation_cb(self, message):
        try:
            value=json.loads(message.data)
            if (not isinstance(value,dict) or not value.get('base_verified')
                    or not isinstance(value.get('base_id'),str) or not value['base_id']
                    or value.get('state') not in ('BASE_READY','RECHECK_WAIT','ACK_WAIT','APPEND_VERIFY_WAIT','VERIFIED','FAILED')):return
            self.preparation=value
            self.preparation_time=time.monotonic()
        except (ValueError,TypeError):
            return

    def profile_ok(self):
        return profile_values(self, self.profile, self.profile_time) is not None

    def pose_cb(self, m):
        self.pose = m
        self.pose_time = time.monotonic()

    def state_cb(self, m):
        self.state = m
        self.state_time = time.monotonic()

    def status_cb(self, m):
        self.status = m
        self.status_time = time.monotonic()

    def command_cb(self, m, info=None):
        self.command = m
        self.command_time = time.monotonic()
        if self.timing.enabled:
            self.timing.command("command_received", m,
                receipt_monotonic_s=self.command_time,
                dds_source_system_ns=getattr(info, "source_timestamp", None),
                dds_received_system_ns=getattr(info, "received_timestamp", None))

    def fresh_status(self):
        return stamped_recent(self, self.status, self.status_time, self.p["status_timeout_s"])

    def fresh_command(self):
        terminal = (self.p["terminal_enabled"] and self.terminal_latched and self.fresh_status()
                    and self.status.state in TERMINAL_STATES and self.command is not None
                    and self.command.terminal_mode)
        return (
            stamped_recent(self, self.command, self.command_time, self.p["command_timeout_s"])
            and self.command.header.frame_id == self.p["local_frame"]
            and command_in_envelope(self.command, self.p, terminal)
        )

    def fresh_state(self):
        return recent(self.state_time, 3) and self.state.connected

    def preview_diagnostic(self, plan_id):
        """Describe the locally received command without relaxing ownership checks."""
        command = self.command
        detail = dict(
            requested_plan_id=plan_id,
            ros_now_ns=self.get_clock().now().nanoseconds,
            timeout_s=self.p["command_timeout_s"],
            receipt_age_s=(None if self.command_time is None
                           else time.monotonic() - self.command_time),
            command_present=command is not None,
            timing=self.timing.context(),
        )
        if command is not None:
            detail.update(
                command_key=command_key(command),
                stamp_ns=command.header.stamp.sec * 10**9 + command.header.stamp.nanosec,
                stamp_age_s=stamp_age(self, command.header.stamp),
                frame_id=command.header.frame_id,
                expected_frame_id=self.p["local_frame"],
                command_plan_id=command.plan_id,
                valid=command.valid,
                reason=command.reason,
                target_held=command.target_held,
                target_age_s=command.target_age_s,
                roll_rad=command.roll_rad,
                pitch_rad=command.pitch_rad,
                thrust=command.thrust,
                envelope_ok=command_in_envelope(command, self.p),
            )
        return detail

    def owner_cb(self, req, res):
        res.accepted = False
        if req.owner == "VISION":
            self.timing.record("ownership_requested", plan_id=req.plan_id)
            if not self.profile_ok():
                res.message = "FCU parameter profile is not verified"
                return res
            if not self.p["live_output"] or not self.p["partial_attitude_verified"]:
                res.message = (
                    "Live output disabled or ArduPlane partial-attitude contract not verified"
                )
                return res
            if not (self.fresh_state() and self.state.armed and self.state.mode == "AUTO"):
                res.message = "Need fresh armed AUTO FCU"
                return res
            if not (
                self.fresh_status()
                and self.status.state in ("MISSION_READY", "AUTO_APPROACH", "HANDOFF_PENDING")
                and req.plan_id
                and self.status.plan_id == req.plan_id
                and self.status.parameter_profile_id == self.profile.profile_id
            ):
                res.message = "Supervisor has not authorized this handoff"
                return res
            if not self.fresh_command() or self.command.plan_id != req.plan_id:
                res.message = "No matching fresh bounded preview command"
                detail = self.preview_diagnostic(req.plan_id)
                self.timing.record("ownership_preview_rejected", **detail)
                self.get_logger().warning(
                    "Handoff preview rejection: "
                    + json.dumps(detail, sort_keys=True)
                )
                return res
            if self.command.target_held:
                res.message = "Fresh visual target required to initiate handoff"
                return res
            if not self.initial_target_ok():
                res.message = "Need fresh pose and continuous initial attitude target"
                return res
            if not self.mode_client.service_is_ready():
                res.message = "SetMode unavailable"
                return res
            self.plan_id = req.plan_id
            self.active_profile_id = self.profile.profile_id
            self.owner = "PENDING"
            self.terminal_latched = self.terminal_fault = False
            self.capture_target = self.last_sent = self.last_sent_time = None
            self.fault_reason = None
            self.begin_mode("GUIDED")
            self.timing.record("ownership_preview_accepted", **self.preview_diagnostic(req.plan_id))
            res.accepted = True
            res.message = "GUIDED requested; wait for ownership confirmation"
        elif req.owner == "ABORT":
            if (self.p['airborne_preparation_enabled'] and self.p['live_output']
                    and self.p['partial_attitude_verified'] and self.preparation
                    and recent(self.preparation_time,3) and self.preparation['state']=='FAILED'
                    and req.plan_id==self.preparation['base_id'] and self.fresh_state()
                    and self.state.armed and self.state.mode=='AUTO' and self.owner in ('NONE','ABORTING')):
                if not self.preparation_failed:
                    self.preparation_failed=True
                    self.plan_id=req.plan_id
                    self.owner='ABORTING'
                    self.abort('Airborne mission preparation failed: '+self.preparation.get('reason',''))
                res.accepted=True
                res.message='Preparation recovery requested; verify actual FCU mode'
                return res
            if not (
                self.fresh_status()
                and req.plan_id
                and self.status.plan_id == req.plan_id
                and self.status.state
                in ("AUTO_APPROACH", "HANDOFF_PENDING", "VISION_FINAL", "ABORT", *TERMINAL_STATES)
            ):
                res.message = "No active matching supervisor plan for recovery"
                return res
            self.plan_id = req.plan_id
            if self.owner == "NONE":
                self.owner = "ABORTING"
            self.abort("Supervisor requested abort")
            res.accepted = True
            res.message = "Terminal zero-thrust hold requested" if self.terminal_fault else "Recovery requested"
        elif req.owner == "RELEASE":
            # A release during owned GUIDED control must not leave the vehicle
            # with a silent command stream. Request recovery in that case.
            if (
                self.owner in ("VISION", "PENDING")
                and self.fresh_state()
                and self.state.mode in ("AUTO", "GUIDED")
            ):
                self.abort("Ownership released")
            else:
                self.owner = "NONE"
                self.requested_mode = None
            res.accepted = True
            res.message = "Ownership released"
        else:
            res.message = "Unknown owner request"
        return res

    def initial_target_ok(self):
        if not stamped_recent(self, self.pose, self.pose_time, self.p["status_timeout_s"]):
            return False
        if self.pose.header.frame_id != self.p["local_frame"]:
            return False
        q = self.pose.pose.orientation
        try:
            r, p, _ = quaternion_euler(q.x, q.y, q.z, q.w)
        except ValueError:
            return False
        return abs(self.command.roll_rad - r) <= math.radians(
            self.p["maximum_roll_step_deg"]
        ) and abs(self.command.pitch_rad + p) <= math.radians(self.p["maximum_pitch_step_deg"])

    def begin_mode(self, mode):
        self.requested_mode = mode
        self.deadline = time.monotonic() + self.p["mode_timeout_s"]
        self.future = None
        self.send_mode_request()

    def send_mode_request(self):
        if self.mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = self.requested_mode
            self.future = self.mode_client.call_async(req)

    def abort(self, reason):
        if self.p["terminal_enabled"] and self.terminal_latched:
            if self.fault_reason != reason:
                self.get_logger().error("Terminal fault: " + reason + "; zero thrust, no automatic RTL")
            self.fault_reason = reason
            self.terminal_fault = True
            self.requested_mode = self.future = None
            self.publish_zero_hold()
            return
        self.get_logger().error(reason)
        self.capture_target = None
        self.last_sent = None
        # Never overwrite a pilot-selected mode. AUTO/GUIDED are the modes
        # controlled by this prototype; other modes release ownership.
        if (
            self.p["live_output"]
            and self.owner in ("VISION", "PENDING", "ABORTING")
            and self.fresh_state()
            and self.state.mode in ("AUTO", "GUIDED")
        ):
            self.owner = "ABORTING"
            self.begin_mode(self.p["abort_mode"])
        else:
            self.owner = "NONE"
            self.requested_mode = None

    def target(self, command):
        m = AttitudeTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.type_mask = ARDUPLANE_ROLL_PITCH_THRUST_MASK
        q = euler_quaternion(command.roll_rad, -command.pitch_rad, command.yaw_enu_rad)
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
        m.thrust = command.thrust
        return m

    def tick(self):
        now = time.monotonic()
        if (self.p['airborne_preparation_enabled'] and self.p['live_output']
                and self.p['partial_attitude_verified'] and self.preparation
                and self.owner=='NONE' and not self.preparation_failed
                and self.fresh_state() and self.state.armed and self.state.mode=='AUTO'
                and self.preparation['state']!='VERIFIED'
                and (self.preparation['state']=='FAILED' or not recent(self.preparation_time,3))):
            self.preparation_failed=True
            self.plan_id=self.preparation['base_id']
            self.owner='ABORTING'
            self.abort('Airborne preparation failed or readiness lease expired')
        if (self.p["terminal_enabled"] and self.owner == "VISION" and self.fresh_status()
                and self.status.plan_id == self.plan_id and self.status.state in TERMINAL_STATES):
            self.terminal_latched = True
        # Preview never reaches MAVROS and is available in AUTO before handoff.
        if self.fresh_command():
            self.preview_pub.publish(self.target(self.command))
        if self.owner in ("PENDING", "VISION"):
            if self.fresh_state() and (not self.state.armed or self.state.mode not in ("AUTO", "GUIDED")):
                self.owner = "NONE"
                self.requested_mode = None
                self.terminal_latched = self.terminal_fault = False
            elif not self.fresh_state():
                self.abort("FCU heartbeat lost")
            elif not self.profile_ok() or self.profile.profile_id != self.active_profile_id:
                self.abort("FCU parameter readiness lease lost or changed")
            elif not self.state.armed:
                self.owner = "NONE"
                self.requested_mode = None
            elif self.state.mode not in ("AUTO", "GUIDED"):
                self.owner = "NONE"
                self.requested_mode = None
            elif not self.fresh_status() or self.status.plan_id != self.plan_id:
                self.abort("Supervisor lease expired or plan changed")
            elif self.status.state == "ABORT":
                self.abort("Supervisor aborted")
        if self.requested_mode:
            if self.fresh_state() and self.state.mode == self.requested_mode:
                if self.requested_mode == "GUIDED" and self.owner == "PENDING":
                    # Service and status use separate transports. Accept the explicit
                    # request with the preceding matching phase, but do not grant
                    # actuation until HANDOFF_PENDING has also arrived.
                    if self.status.state in ("HANDOFF_PENDING", "VISION_FINAL"):
                        self.owner = "VISION"
                        self.requested_mode = None
                        self.future = None
                    elif self.deadline is not None and now > self.deadline:
                        self.abort("Supervisor handoff-status confirmation timeout")
                elif self.owner == "ABORTING":
                    self.owner = "NONE"
                    self.requested_mode = None
                    self.future = None
            elif self.deadline is not None and now > self.deadline:
                if self.owner == "PENDING":
                    self.abort("GUIDED confirmation timeout")
                else:
                    # Recovery retry; telemetry, not service response, confirms mode.
                    self.deadline = now + self.p["mode_timeout_s"]
                    self.send_mode_request()
            elif self.future is None:
                self.send_mode_request()
            elif self.future.done():
                try:
                    response = self.future.result()
                    if not response.mode_sent:
                        self.get_logger().error("Mode request not sent; waiting/retrying")
                except Exception as exc:
                    self.get_logger().error(str(exc))
                # Avoid flooding: retry only when the deadline expires.
        if self.owner == "VISION":
            if self.terminal_fault or (self.terminal_latched and self.status.state in ZERO_THRUST_STATES):
                self.publish_zero_hold()
            elif self.status.state == "COMPLETE":
                # Explicit stationary capture confirmation: hold last attitude
                # with zero thrust until pilot disarms or changes mode.
                if self.capture_target is None and self.last_sent is not None:
                    self.capture_target = self.last_sent
                    self.capture_target.thrust = 0.0
                if self.capture_target is not None and self.p["live_output"]:
                    self.capture_target.header.stamp = self.get_clock().now().to_msg()
                    self.pub.publish(self.capture_target)
            elif self.status.state in ("HANDOFF_PENDING", "VISION_FINAL", "TERMINAL_COMMITTED"):
                if not self.fresh_command() or self.command.plan_id != self.plan_id:
                    self.abort("Vision command watchdog expired")
                elif self.p["live_output"]:
                    # Enforce command continuity independently of guidance node.
                    if self.last_sent is not None and self.last_sent_time is not None:
                        q = self.last_sent.orientation
                        r, p, _ = quaternion_euler(q.x, q.y, q.z, q.w)
                        if abs(self.command.roll_rad - r) > math.radians(
                            self.p["maximum_roll_step_deg"]
                        ) or abs(self.command.pitch_rad + p) > math.radians(
                            self.p["maximum_pitch_step_deg"]
                        ):
                            self.abort("Attitude command discontinuity")
                        else:
                            self.publish_command(now)
                    elif self.initial_target_ok():
                        self.publish_command(now)
                    else:
                        self.abort("Initial attitude target discontinuity")
            else:
                self.abort("Supervisor no longer grants vision control")
        self.owner_pub.publish(String(data=self.owner))

    def publish_zero_hold(self):
        if self.capture_target is None and self.last_sent is not None:
            self.capture_target = copy.deepcopy(self.last_sent)
            self.capture_target.thrust = 0.
        if (self.capture_target is None and self.command is not None
                and self.command.plan_id == self.plan_id
                and command_in_envelope(self.command, self.p, terminal=True)):
            self.capture_target = self.target(self.command)
            self.capture_target.thrust = 0.
        if self.capture_target is not None and self.p["live_output"]:
            self.capture_target.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.capture_target)

    def publish_command(self, now):
        target = self.target(self.command)
        self.pub.publish(target)
        self.last_sent = target
        self.last_sent_time = now


def main(args=None):
    run(FCUInterface, args)
