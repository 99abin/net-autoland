"""Preflight FCU parameter synchronization, verification and readiness lease.

Writes are sequential, never atomic. Failed/partial writes remain blocked and are
not silently rolled back. A baseline YAML backup is saved before the first write.
"""

import json
import math
from pathlib import Path
import time
import yaml
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterEvent
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from mavros_msgs.msg import State
from mavros_msgs.srv import ParamPull
from std_srvs.srv import Trigger
from autoland_interfaces.msg import ParameterProfileStatus
from .parameter_profile import (
    load_profile,
    selected_names,
    decode_value,
    typed_target,
    equal_value,
    validate_effective,
    profile_id,
)
from .ros_util import params, retained, recent, run


class ParameterManager(Node):
    def __init__(self):
        super().__init__("parameter_manager")
        self.p = params(
            self,
            dict(
                profile_path="",
                parameter_write_enabled=False,
                apply_on_start=True,
                discover_landing_parameters=True,
                mavros_namespace="/mavros",
                service_timeout_s=12.0,
                pull_timeout_s=65.0,
                monitor_interval_s=2.0,
                backup_directory="~/.ros/autoland_parameter_backups",
            ),
        )
        if not all(
            math.isfinite(self.p[key]) and self.p[key] > 0
            for key in ("service_timeout_s", "pull_timeout_s", "monitor_interval_s")
        ):
            raise ValueError("Parameter manager timeouts/interval must be finite and positive")
        ns = self.p["mavros_namespace"].rstrip("/")
        self.param_node = ns + "/param"
        self.pull = self.create_client(ParamPull, self.param_node + "/pull")
        self.list = self.create_client(ListParameters, self.param_node + "/list_parameters")
        self.get = self.create_client(GetParameters, self.param_node + "/get_parameters")
        self.set = self.create_client(SetParameters, self.param_node + "/set_parameters")
        self.pub = self.create_publisher(
            ParameterProfileStatus, "/autoland/parameter_profile", retained()
        )
        self.create_subscription(State, ns + "/state", self.state_cb, 10)
        self.create_subscription(ParameterEvent, "/parameter_events", self.event_cb, 10)
        self.create_service(Trigger, "~/sync", self.sync_cb)
        self.create_service(Trigger, "~/export_profile", self.export_cb)
        self.fcu = None
        self.state_time = None
        self.phase = "IDLE"
        self.future = None
        self.deadline = None
        self.names = []
        self.baseline = {}
        self.kinds = {}
        self.expected = {}
        self.pending = []
        self.backup_path = ""
        self.attempted = []
        self.current_name = None
        self.auto_pending = self.p["apply_on_start"]
        self.last_pub = 0.0
        self.last_monitor = 0.0
        self.status = ParameterProfileStatus()
        self.status.writes_enabled = self.p["parameter_write_enabled"]
        self.status.state = "WAITING"
        self.status.reason = "Waiting for reviewed profile and disarmed FCU"
        self.create_timer(0.1, self.tick)

    def publish(self):
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.status)
        self.last_pub = time.monotonic()

    def state_cb(self, msg):
        if (
            self.fcu is not None
            and self.fcu.connected
            and not msg.connected
            and self.status.verified
        ):
            self.fail("FCU disconnected; preflight verification must be repeated")
        self.fcu = msg
        self.state_time = time.monotonic()

    def event_cb(self, event):
        if event.node != self.param_node:
            return
        if not self.status.verified and self.phase not in (
            "CACHE_RECHECK_REQUEST", "CACHE_RECHECK_WAIT"
        ):
            return
        for p in list(event.new_parameters) + list(event.changed_parameters):
            if p.name in self.expected:
                try:
                    value, kind = decode_value(p.value)
                    if kind != self.kinds[p.name] or not equal_value(value, self.expected[p.name]):
                        self.fail(f"Verified FCU parameter changed: {p.name}")
                        return
                except ValueError as exc:
                    self.fail(str(exc))
                    return
        removed = sorted(p.name for p in event.deleted_parameters if p.name in self.expected)
        if removed:
            # MAVROS clears its ROS parameter cache on a pull/reconnect. Revoke
            # readiness immediately; while safely on the ground, read the whole
            # verified set again instead of treating a cache event as FCU deletion.
            if self.phase == "IDLE" and self.safe_preflight():
                self.status.verified = False
                self.status.state = "READING"
                self.status.reason = "MAVROS parameter cache refreshed; rechecking verified values"
                self.phase = "CACHE_RECHECK_REQUEST"
                self.deadline = time.monotonic() + self.p["service_timeout_s"]
                self.publish()
            else:
                self.fail("MAVROS cache removed verified parameters: " + ", ".join(removed)
                          + "; preflight sync required")

    def safe_preflight(self):
        return (
            recent(self.state_time, 3)
            and self.fcu.connected
            and not self.fcu.armed
            and self.fcu.mode != "AUTO"
        )

    def sync_cb(self, request, response):
        if self.phase != "IDLE" or (self.future is not None and not self.future.done()):
            response.success = False
            response.message = "A parameter operation is still in flight"
        elif not self.safe_preflight():
            response.success = False
            response.message = "Sync requires connected, disarmed FCU outside AUTO"
        else:
            response.success = self.begin()
            response.message = self.status.reason
        return response

    def export_cb(self, request, response):
        if not self.status.verified or not self.expected:
            response.success = False
            response.message = "Verify the profile before exporting it"
            return response
        try:
            directory = Path(self.p["backup_directory"]).expanduser()
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / f"fc-verified-{self.status.profile_id}.yaml"
            payload = dict(
                schema_version=1,
                configured=False,
                description="Last verified native landing values; review before using as a write profile",
                parameters=self.expected,
            )
            path.write_text(yaml.safe_dump(payload, sort_keys=True))
            response.success = True
            response.message = str(path)
        except OSError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def begin(self):
        try:
            profile = load_profile(self.p["profile_path"])
            if not profile.configured:
                raise ValueError("Review fc_parameters.yaml and set configured: true")
        except (ValueError, TypeError, OSError, yaml.YAMLError) as exc:
            self.fail(str(exc))
            return False
        self.profile = profile
        self.auto_pending = False
        self.status.verified = False
        self.status.state = "READING"
        self.status.profile_id = ""
        self.status.parameters_json = ""
        self.status.reason = "Refreshing FCU parameter list"
        self.attempted = []
        self.backup_path = ""
        self.status.backup_path = ""
        self.phase = "PULL_REQUEST"
        self.deadline = time.monotonic() + self.p["pull_timeout_s"]
        self.publish()
        return True

    def fail(self, reason):
        self.status.verified = False
        self.status.state = "FAILED"
        self.phase = "IDLE"
        suffix = (
            f'; attempted writes: {", ".join(self.attempted)}; inspect FCU/backup before retry'
            if self.attempted
            else ""
        )
        self.status.reason = reason + suffix
        self.get_logger().error(self.status.reason)
        self.publish()
        # Retain an unfinished RPC so a second transaction cannot race its late write.
        if self.future is not None and self.future.done():
            self.future = None

    def start_rpc(self, client, request, phase, timeout=None):
        if not client.service_is_ready():
            return False
        self.future = client.call_async(request)
        self.phase = phase
        self.deadline = time.monotonic() + (timeout or self.p["service_timeout_s"])
        return True

    def decode_response(self, result):
        if result is None or len(result.values) != len(self.names):
            raise ValueError("Incomplete FCU parameter response")
        values = {}
        kinds = {}
        for name, raw in zip(self.names, result.values):
            values[name], kinds[name] = decode_value(raw)
        return values, kinds

    def snapshot(self):
        directory = Path(self.p["backup_directory"]).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        filename = directory / f"fc-baseline-{time.time_ns()}.yaml"
        # Capture values before any write. Restoring through this manager must
        # still satisfy its standard-glide-slope profile validation.
        payload = dict(
            schema_version=1,
            configured=False,
            description="Baseline before parameter sync; review and set configured true to restore",
            parameters=self.baseline,
        )
        with filename.open("x") as stream:
            yaml.safe_dump(payload, stream, sort_keys=True)
        self.backup_path = str(filename)
        self.status.backup_path = self.backup_path

    def tick(self):
        now = time.monotonic()
        if now - self.last_pub >= 0.5:
            self.publish()
        if self.status.verified and not (recent(self.state_time, 3) and self.fcu.connected):
            self.fail("FCU heartbeat unavailable")
            return
        if self.phase == "IDLE":
            if self.future is not None:
                if not self.future.done():
                    return
                self.future = None
            if self.auto_pending and self.safe_preflight():
                self.auto_pending = False
                self.begin()
            elif self.status.verified and now - self.last_monitor >= self.p["monitor_interval_s"]:
                request = GetParameters.Request()
                request.names = self.names
                if self.start_rpc(self.get, request, "MONITOR_WAIT"):
                    self.last_monitor = now
                else:
                    self.fail("MAVROS parameter read service unavailable")
            return
        if self.phase != "MONITOR_WAIT" and not self.safe_preflight():
            self.fail("FCU left preflight state during parameter sync; no further writes")
            return
        if self.deadline is not None and now > self.deadline:
            self.fail("FCU parameter operation timed out (an in-flight write may still complete)")
            return
        try:
            if self.phase == "PULL_REQUEST":
                request = ParamPull.Request()
                request.force_pull = True
                self.start_rpc(self.pull, request, "PULL_WAIT", self.p["pull_timeout_s"])
                return
            if self.phase == "LIST_REQUEST":
                self.start_rpc(self.list, ListParameters.Request(), "LIST_WAIT")
                return
            if self.phase in ("READ_REQUEST", "VERIFY_REQUEST"):
                request = GetParameters.Request()
                request.names = self.names
                self.start_rpc(
                    self.get,
                    request,
                    "READ_WAIT" if self.phase == "READ_REQUEST" else "VERIFY_WAIT",
                )
                return
            if self.phase == "CACHE_RECHECK_REQUEST":
                request = GetParameters.Request()
                request.names = self.names
                self.start_rpc(self.get, request, "CACHE_RECHECK_WAIT")
                return
            if self.phase == "WRITE_REQUEST":
                if not self.pending:
                    # A forced re-pull verifies FCU values, not just cached desired values.
                    self.phase = "VERIFY_PULL_REQUEST"
                    self.deadline = now + self.p["pull_timeout_s"]
                    return
                name = self.pending[0]
                value = ParameterValue()
                value.type = self.kinds[name]
                if value.type == 2:
                    value.integer_value = self.expected[name]
                else:
                    value.double_value = self.expected[name]
                parameter = Parameter()
                parameter.name = name
                parameter.value = value
                request = SetParameters.Request()
                request.parameters = [parameter]
                if self.start_rpc(self.set, request, "WRITE_WAIT"):
                    self.current_name = name
                    self.attempted.append(name)
                return
            if self.phase == "VERIFY_PULL_REQUEST":
                request = ParamPull.Request()
                request.force_pull = True
                self.start_rpc(self.pull, request, "VERIFY_PULL_WAIT", self.p["pull_timeout_s"])
                return
            if self.future is None or not self.future.done():
                return
            result = self.future.result()
            self.future = None
            if self.phase in ("PULL_WAIT", "VERIFY_PULL_WAIT"):
                if result is None or not result.success:
                    raise ValueError("FCU parameter pull failed")
                self.phase = "LIST_REQUEST" if self.phase == "PULL_WAIT" else "VERIFY_REQUEST"
                self.deadline = now + self.p["service_timeout_s"]
            elif self.phase == "LIST_WAIT":
                self.names = selected_names(
                    self.profile, result.result.names, self.p["discover_landing_parameters"]
                )
                self.phase = "READ_REQUEST"
                self.deadline = now + self.p["service_timeout_s"]
            elif self.phase == "READ_WAIT":
                self.baseline, self.kinds = self.decode_response(result)
                self.expected = {
                    name: typed_target(
                        self.profile.parameters.get(name), self.baseline[name], self.kinds[name]
                    )
                    for name in self.names
                }
                validate_effective(self.expected)
                self.status.profile_id = profile_id(self.expected)
                self.status.parameters_json = json.dumps(
                    self.expected, sort_keys=True, allow_nan=False
                )
                self.pending = [
                    name
                    for name in self.names
                    if not equal_value(self.baseline[name], self.expected[name])
                ]
                if not self.pending:
                    self.ready("FCU landing profile already matches; no writes")
                elif not self.p["parameter_write_enabled"]:
                    self.status.state = "PREVIEW"
                    self.status.reason = "Would change: " + ", ".join(self.pending)
                    self.phase = "IDLE"
                    self.publish()
                else:
                    self.snapshot()
                    self.status.state = "APPLYING"
                    self.status.reason = "Applying profile sequentially"
                    self.phase = "WRITE_REQUEST"
                    self.deadline = now + self.p["service_timeout_s"]
                    self.publish()
            elif self.phase == "WRITE_WAIT":
                if result is None or len(result.results) != 1 or not result.results[0].successful:
                    raise ValueError("FCU rejected parameter write: " + str(self.current_name))
                self.pending.pop(0)
                self.phase = "WRITE_REQUEST"
                self.deadline = now + self.p["service_timeout_s"]
            elif self.phase in ("VERIFY_WAIT", "MONITOR_WAIT", "CACHE_RECHECK_WAIT"):
                actual, kinds = self.decode_response(result)
                changed = [
                    name
                    for name in self.names
                    if kinds[name] != self.kinds[name]
                    or not equal_value(actual[name], self.expected[name])
                ]
                if changed:
                    raise ValueError("FCU profile mismatch: " + ", ".join(changed))
                self.ready("FCU landing profile verified")
        except Exception as exc:
            self.fail(str(exc))

    def ready(self, reason):
        self.phase = "IDLE"
        self.status.state = "READY"
        self.status.verified = True
        self.status.reason = reason
        self.last_monitor = time.monotonic()
        self.publish()


def main(args=None):
    run(ParameterManager, args)
