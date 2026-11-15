"""Preflight pull -> extend -> full push -> readback verification transaction."""

import time
from rclpy.node import Node
from mavros_msgs.msg import State, Waypoint, WaypointList
from mavros_msgs.srv import WaypointPull, WaypointPush
from std_srvs.srv import Trigger
from autoland_interfaces.msg import LandingPlan, ParameterProfileStatus
from .mission_builder import LandingConfig, extend_mission, fingerprint, same_mission
from .ros_util import params, retained, recent, profile_values, run


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")
        self.p = params(
            self,
            dict(
                configured=False,
                upload_enabled=False,
                prepare_on_start=True,
                land_lat=0.0,
                land_lon=0.0,
                land_alt=1.35,
                approach_heading_deg=0.0,
                approach_distance_m=300.0,
                approach_alt=40.0,
                transaction_timeout_s=15.0,
                mavros_namespace="/mavros",
            ),
        )
        ns = self.p["mavros_namespace"].rstrip("/")
        self.pull = self.create_client(WaypointPull, ns + "/mission/pull")
        self.push = self.create_client(WaypointPush, ns + "/mission/push")
        self.pub = self.create_publisher(LandingPlan, "/autoland/landing_plan", retained())
        self.create_subscription(WaypointList, ns + "/mission/waypoints", self.list_cb, retained())
        self.create_subscription(State, ns + "/state", self.state_cb, 10)
        self.create_service(Trigger, "~/prepare", self.prepare_cb)
        self.profile = None
        self.profile_time = None
        self.transaction_profile_id = None
        self.create_subscription(
            ParameterProfileStatus, "/autoland/parameter_profile", self.profile_cb, retained()
        )
        self.items = []
        self.generation = 0
        self.state = None
        self.state_time = None
        self.phase = "IDLE"
        self.future = None
        self.deadline = 0
        self.expected = None
        self.plan = None
        self.last_heartbeat = 0.0
        self.auto_pending = self.p["prepare_on_start"]
        self.create_timer(0.1, self.tick)
        self.invalidate("Waiting for configured landing and FCU")

    def profile_cb(self, msg):
        self.profile = msg
        self.profile_time = time.monotonic()
        if (
            self.plan is not None
            and self.plan.verified
            and (not self.profile_ok() or msg.profile_id != self.plan.parameter_profile_id)
        ):
            self.expected = None
            self.invalidate("FCU parameter profile changed; prepare again")

    def profile_ok(self):
        return profile_values(self, self.profile, self.profile_time) is not None

    def state_cb(self, msg):
        self.state = msg
        self.state_time = time.monotonic()

    def list_cb(self, msg):
        self.items = list(msg.waypoints)
        self.generation += 1
        if (
            self.phase == "IDLE"
            and self.expected is not None
            and not same_mission(self.items, self.expected)
        ):
            self.expected = None
            self.invalidate("Mission changed; prepare again before flight")

    def invalidate(self, reason):
        self.plan = LandingPlan()
        self.plan.header.stamp = self.get_clock().now().to_msg()
        self.plan.verified = False
        self.plan.reason = reason
        self.pub.publish(self.plan)
        self.get_logger().info(reason)

    def safe_to_prepare(self):
        return (
            self.profile_ok()
            and (self.phase == "IDLE" or self.transaction_profile_id == self.profile.profile_id)
            and recent(self.state_time, 3)
            and self.state.connected
            and not self.state.armed
            and self.state.mode != "AUTO"
        )

    def prepare_cb(self, req, res):
        if self.phase != "IDLE":
            res.success = False
            res.message = "Transaction already running"
        elif not self.p["configured"]:
            res.success = False
            res.message = (
                "Set configured and real landing coordinates in mission.yaml, then restart"
            )
        elif not self.safe_to_prepare():
            res.success = False
            res.message = (
                "Need verified FCU parameter profile and fresh connected, disarmed FCU outside AUTO"
            )
        else:
            self.auto_pending = False
            self.begin()
            res.success = True
            res.message = "Preparation started; observe /autoland/landing_plan"
        return res

    def begin(self):
        try:
            self.config = LandingConfig(
                **{k: self.p[k] for k in LandingConfig.__dataclass_fields__}
            )
            self.config.validate()
        except ValueError as exc:
            self.fail(str(exc))
            return
        self.transaction_profile_id = self.profile.profile_id
        self.expected = None
        self.invalidate("Pulling base mission")
        self.phase = "PULL_REQUEST"
        self.deadline = time.monotonic() + self.p["transaction_timeout_s"]

    def fail(self, reason):
        self.phase = "IDLE"
        self.future = None
        self.expected = None
        self.invalidate(reason)

    def request_pull(self, next_phase):
        if not self.pull.service_is_ready():
            return
        self.baseline = self.generation
        self.future = self.pull.call_async(WaypointPull.Request())
        self.phase = next_phase

    def tick(self):
        if self.plan is not None and time.monotonic() - self.last_heartbeat >= 1.0:
            self.plan.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.plan)
            self.last_heartbeat = time.monotonic()
        if self.auto_pending and self.p["configured"] and self.safe_to_prepare():
            self.auto_pending = False
            self.begin()
        if self.phase == "IDLE":
            return
        if not self.safe_to_prepare():
            self.fail("FCU state changed during preparation")
            return
        if time.monotonic() > self.deadline:
            self.fail("Mission transaction timed out; pull and inspect FCU mission")
            return
        try:
            if self.phase == "PULL_REQUEST":
                self.request_pull("PULL_WAIT")
            elif self.phase == "VERIFY_REQUEST":
                self.request_pull("VERIFY_WAIT")
            elif self.phase in ("PULL_WAIT", "VERIFY_WAIT"):
                if not self.future.done():
                    return
                result = self.future.result()
                if result is None or not result.success:
                    self.fail("Mission pull failed")
                    return
                # Topic and service replies can be delivered in either order.
                if self.generation <= self.baseline:
                    return
                if len(self.items) != result.wp_received:
                    return
                if self.phase == "VERIFY_WAIT":
                    if not same_mission(self.items, self.expected):
                        self.fail("Readback mismatch; inspect mission")
                        return
                    self.publish_plan(True, "Extended mission verified on FCU")
                    self.phase = "IDLE"
                    return
                self.expected, already = extend_mission(self.items, self.config)
                if already:
                    self.publish_plan(True, "Matching landing pair already present; no upload")
                    self.phase = "IDLE"
                elif not self.p["upload_enabled"]:
                    self.publish_plan(False, "Preview only: set upload_enabled to write mission")
                    self.phase = "IDLE"
                else:
                    self.phase = "PUSH_REQUEST"
            elif self.phase == "PUSH_REQUEST":
                if not self.push.service_is_ready():
                    return
                request = WaypointPush.Request()
                request.start_index = 0
                for item in self.expected:
                    w = Waypoint()
                    for k, v in item.__dict__.items():
                        setattr(w, k, v)
                    request.waypoints.append(w)
                self.future = self.push.call_async(request)
                self.phase = "PUSH_WAIT"
            elif self.phase == "PUSH_WAIT":
                if not self.future.done():
                    return
                result = self.future.result()
                if (
                    result is None
                    or not result.success
                    or result.wp_transfered != len(self.expected)
                ):
                    self.fail("Full mission upload failed or incomplete; inspect mission")
                    return
                self.phase = "VERIFY_REQUEST"
        except Exception as exc:
            self.fail(f"Mission transaction error: {exc}")

    def publish_plan(self, verified, reason):
        c = self.config
        msg = LandingPlan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.verified = verified
        msg.reason = reason
        msg.plan_id = fingerprint(self.expected)
        msg.parameter_profile_id = self.transaction_profile_id
        msg.item_count = len(self.expected)
        msg.approach_index = len(self.expected) - 2
        msg.land_index = len(self.expected) - 1
        msg.altitude_frame = 3
        msg.approach_lat = self.expected[-2].x_lat
        msg.approach_lon = self.expected[-2].y_long
        msg.approach_alt = c.approach_alt
        msg.land_lat = c.land_lat
        msg.land_lon = c.land_lon
        msg.land_alt = c.land_alt
        msg.heading_deg = c.approach_heading_deg
        msg.distance_m = c.approach_distance_m
        self.plan = msg
        self.pub.publish(msg)
        self.get_logger().info(reason)


def main(args=None):
    run(MissionManager, args)
