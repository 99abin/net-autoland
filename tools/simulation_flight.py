"""Operator-driven native AUTO flight experiment. No arming or MAVLink writes."""

import json
import hashlib
import math
import os
from pathlib import Path
import re
import subprocess
import time

import yaml

from flight_checks import finite_json, launch_blockers, native_takeoff_blockers

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / ".simulation"
PREFIX = "/world/merlin_landing/launch_assist"


def managed_session(config):
    path = STATE / "active.json"
    if not path.exists():
        raise RuntimeError("Start a managed session with --launch-assist first")
    session = json.loads(path.read_text())
    if not session.get("launch_assist"):
        raise RuntimeError("This session has no launch assist; restart with --launch-assist")
    if session.get("ros_domain_id") != config["ros_domain_id"]:
        raise RuntimeError("Managed ROS domain does not match settings")
    pid = session.get("children", {}).get("sitl")
    command = Path(f"/proc/{pid}/cmdline")
    if not command.exists() or command.read_bytes().split(b"\0")[0] != os.fsencode(config["ardupilot_binary"]):
        raise RuntimeError("The recorded local ArduPlane SITL process is not alive")
    if "mavros" not in session.get("children", {}):
        raise RuntimeError("Managed MAVROS is required")
    mavros = Path(f"/proc/{session['children']['mavros']}/cmdline")
    if not mavros.exists() or os.fsencode(str(ROOT / "simulation/mavros.launch.py")) not in mavros.read_bytes().split(b"\0"):
        raise RuntimeError("Recorded managed localhost MAVROS launch is not alive")
    return session


def gazebo_service(service, request_type, response_type, request=""):
    try:
        result = subprocess.run([
            "/usr/bin/gz", "service", "-s", PREFIX + service,
            "--reqtype", request_type, "--reptype", response_type,
            "--timeout", "1500", "--req", request,
        ], text=True, capture_output=True, timeout=4)
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError("Gazebo launch service timed out; inspect plugin status before retrying") from exc
    if result.returncode != 0:
        raise RuntimeError("Gazebo launch service failed: " + result.stderr.strip())
    return result.stdout.strip()


def plugin_status():
    output = gazebo_service("/status", "gz.msgs.Empty", "gz.msgs.StringMsg")
    match = re.search(r'^data:\s*(".*")\s*$', output, re.MULTILINE)
    if not match:
        raise RuntimeError("No launch-plugin status received; restart with --launch-assist")
    return json.loads(json.loads(match.group(1)))


class FlightObserver:
    def __init__(self, full_autoland=False):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from mavros_msgs.msg import State as FCUState, WaypointList, VfrHud
        from geometry_msgs.msg import PoseStamped, TwistStamped, TwistWithCovarianceStamped
        from sensor_msgs.msg import NavSatFix, CameraInfo
        from std_msgs.msg import Float64, String
        from autoland_interfaces.msg import (
            ParameterProfileStatus, LandingPlan, LandingStatus, NetDetection,
            NetEstimate, ControlSetpoint,
        )
        rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true", "-r", "/clock:=/simulation/clock"])
        self.ros = rclpy
        self.node = rclpy.create_node("native_auto_flight_observer")
        self.messages = {}
        self.full_autoland = full_autoland
        self.guided_observed = False
        self.vision_owner_observed = False
        self.supervisor_states = set()
        self.live_command_count = 0
        self.held_target_command_count = 0
        self.zero_thrust_count = 0
        self.close_terminal_command_count = 0
        self.net_contact_observed = False
        self.received = {}
        self.file = None
        self.started = time.monotonic()
        self.heights = []
        self.airspeeds = []
        self.sequences = set()
        self.auto_sequences = set()
        self.armed_auto = False
        self.takeoff_index = None
        self.takeoff_alt = None
        self.land_index = None
        self.faults = set()
        self.last_state = None
        self.state_gaps = []
        self.subscriptions = []
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topics = [
            ("camera_info", CameraInfo, "/camera/camera_info", qos_profile_sensor_data),
            ("state", FCUState, "/mavros/state", 10),
            ("mission", WaypointList, "/mavros/mission/waypoints", retained),
            ("pose", PoseStamped, "/mavros/local_position/pose", qos_profile_sensor_data),
            ("velocity", TwistStamped, "/simulation/velocity_local_enu", qos_profile_sensor_data),
            ("altitude", Float64, "/mavros/global_position/rel_alt", qos_profile_sensor_data),
            ("gps", NavSatFix, "/mavros/global_position/global", qos_profile_sensor_data),
            ("wind", TwistWithCovarianceStamped, "/mavros/wind_estimation", qos_profile_sensor_data),
            ("airspeed", VfrHud, "/mavros/vfr_hud", qos_profile_sensor_data),
            ("profile", ParameterProfileStatus, "/autoland/parameter_profile", retained),
            ("plan", LandingPlan, "/autoland/landing_plan", retained),
            ("supervisor", LandingStatus, "/autoland/status", retained),
            ("ownership", String, "/autoland/control_ownership", 10),
            ("preparation", String, "/autoland/airborne_preparation", retained),
            ("detection", NetDetection, "/autoland/net_detection", 10),
            ("estimate", NetEstimate, "/autoland/net_estimate", 10),
            ("preview", ControlSetpoint, "/autoland/control_setpoint", 10),
        ]
        from mavros_msgs.msg import AttitudeTarget
        topics.append(("live_attitude", AttitudeTarget, "/mavros/setpoint_raw/attitude", 10))
        for name, typ, topic, qos in topics:
            self.subscriptions.append(self.node.create_subscription(
                typ, topic, lambda msg, key=name: self.receive(key, msg), qos))
        from mavros_msgs.msg import Mavlink
        for key,topic in (('mission_tx','/uas1/mavlink_sink'),('mission_rx','/uas1/mavlink_source')):
            self.subscriptions.append(self.node.create_subscription(Mavlink,topic,
                lambda msg,key=key:self.receive(key,msg) if msg.msgid in (30,36,38,39,40,41,44,45,47,51,62,65,73,74,82,83,194) else None,
                qos_profile_sensor_data))

    def receive(self, key, message):
        now = time.monotonic()
        self.messages[key] = message
        self.received[key] = now
        if key == "state":
            if self.last_state is not None:
                self.state_gaps.append(now - self.last_state)
            self.last_state = now
            if message.armed and message.mode == "AUTO":
                self.armed_auto = True
            if message.armed and message.mode == "GUIDED":
                self.guided_observed = True
                if not self.full_autoland:
                    self.faults.add("GUIDED observed during native AUTO-only test")
        elif key == "supervisor":
            self.supervisor_states.add(message.state)
            self.net_contact_observed |= message.net_contact
            if message.state in ("ABORT", "TERMINAL_FAULT", "MISSED_NET"):
                self.faults.add("Supervisor " + message.state + ": " + message.reason)
        elif key == "ownership" and message.data == "VISION":
            self.vision_owner_observed = True
        elif key == "preview" and message.valid:
            if message.target_held:
                self.held_target_command_count += 1
            if (message.terminal_mode and message.forward_range_m > 0
                    and (message.forward_range_m < 2.5 or message.time_to_target_s < .2)):
                self.close_terminal_command_count += 1
        elif key == "live_attitude":
            self.live_command_count += 1
            if message.thrust == 0:
                self.zero_thrust_count += 1
        elif key == "mission":
            self.sequences.add(message.current_seq)
            state = self.messages.get("state")
            if (state and state.connected and state.armed and state.mode == "AUTO"
                    and now - self.received.get("state", -1e30) <= 3):
                self.auto_sequences.add(message.current_seq)
            for i, item in enumerate(message.waypoints):
                if item.command == 22 and self.takeoff_index is None:
                    self.takeoff_index, self.takeoff_alt = i, item.z_alt
                if item.command == 21:
                    self.land_index = i
        elif key == "altitude" and math.isfinite(message.data):
            self.heights.append(message.data)
        elif key == "airspeed" and math.isfinite(message.airspeed):
            self.airspeeds.append(message.airspeed)
        if self.file:
            from rosidl_runtime_py.convert import message_to_ordereddict
            self.file.write(json.dumps(finite_json({
                "wall_elapsed_s": now - self.started,
                "sim_time_s": self.node.get_clock().now().nanoseconds / 1e9,
                "topic": key, "message": message_to_ordereddict(message),
            }), allow_nan=False) + "\n")

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ros.spin_once(self.node, timeout_sec=0.1)

    def flags(self):
        from rcl_interfaces.srv import GetParameters
        values = {}
        for name, keys in (("fcu_interface", ["live_output", "partial_attitude_verified", "airborne_preparation_enabled"]),
                           ("mission_manager", ["airborne_preparation_enabled"]),
                           ("landing_supervisor", ["handoff_enabled"])):
            client = self.node.create_client(GetParameters, f"/{name}/get_parameters")
            try:
                if not client.wait_for_service(timeout_sec=1):
                    continue
                req = GetParameters.Request()
                req.names = keys
                future = client.call_async(req)
                self.ros.spin_until_future_complete(self.node, future, timeout_sec=2)
                if future.done() and future.result():
                    for key, value in zip(keys, future.result().values):
                        if value.type == 1:
                            values['fcu_airborne_preparation_enabled' if name=='fcu_interface' and key=='airborne_preparation_enabled' else key] = value.bool_value
            finally:
                self.node.destroy_client(client)
        return values

    def takeoff_parameters(self):
        from rcl_interfaces.srv import GetParameters
        from autoland.parameter_profile import decode_value
        client = self.node.create_client(GetParameters, "/mavros/param/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=2):
                return {"error": "MAVROS parameter cache unavailable"}
            req = GetParameters.Request()
            req.names = ["TKOFF_THR_MINACC", "TKOFF_THR_MINSPD", "TKOFF_THR_DELAY",
                         "TKOFF_ACCEL_CNT", "TKOFF_THR_MAX", "AIRSPEED_MIN",
                         "AIRSPEED_CRUISE", "SCALING_SPEED", "AHRS_EKF_TYPE", "ARSPD_TYPE", "TECS_LAND_SPDWGT",
                         "TECS_LAND_ARSPD", "TECS_APPR_SMAX", "LAND_OPTIONS",
                         "PTCH_RATE_FF", "PTCH_RATE_I", "PTCH_RATE_P", "PTCH_RATE_D"]
            future = client.call_async(req)
            self.ros.spin_until_future_complete(self.node, future, timeout_sec=8)
            if not future.done() or not future.result():
                return {"error": "MAVROS parameter cache read timed out"}
            return {name: decode_value(value)[0] if value.type in (2, 3) else None
                    for name, value in zip(req.names, future.result().values)}
        finally:
            self.node.destroy_client(client)

    def snapshot(self, flags, status):
        from autoland.geometry import quaternion_euler
        from autoland.mission_builder import LandingConfig, landing_pair, same_mission
        m = self.messages
        result = {key + "_received": value for key, value in self.received.items()}
        if 'preparation' in m:
            try:
                from autoland.mission_builder import fingerprint
                p=json.loads(m['preparation'].data)
                result.update(preparation_state=p['state'],base_profile_id=p['profile_id'],
                    base_matches=(p['base_verified'] and 'mission' in m
                        and fingerprint(m['mission'].waypoints)==p['base_id']),
                    preparation_stamp_age_s=self.node.get_clock().now().nanoseconds/1e9-p['stamp_s'])
            except (ValueError,KeyError,TypeError):
                result['base_matches']=False
        result.update(flags)
        result.update(plugin_ready=status.get("ready", False))
        if "state" in m:
            result.update(connected=m["state"].connected, armed=m["state"].armed, mode=m["state"].mode)
        if "pose" in m:
            q = m["pose"].pose.orientation
            angles = quaternion_euler(q.x, q.y, q.z, q.w)
            result.update(zip(("roll_deg", "pitch_deg", "yaw_deg"), map(math.degrees, angles)))
        if "velocity" in m:
            v = m["velocity"].twist.linear
            result["speed_mps"] = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        if "altitude" in m:
            result["relative_alt_m"] = m["altitude"].data
        if "profile" in m:
            result.update(profile_verified=m["profile"].verified, profile_id=m["profile"].profile_id)
        if "plan" in m:
            result.update(plan_verified=m["plan"].verified, plan_profile_id=m["plan"].parameter_profile_id)
        if "supervisor" in m:
            status = m["supervisor"]
            stamp = status.header.stamp
            age = (self.node.get_clock().now().nanoseconds
                   - stamp.sec * 10**9 - stamp.nanosec) / 1e9
            result.update(supervisor_state=status.state,
                          supervisor_enabled=status.monitoring_enabled,
                          supervisor_stamp_age_s=age,
                          supervisor_receipt_age_s=time.monotonic() - self.received["supervisor"],
                          supervisor_fresh=(0 <= age <= .3
                              and time.monotonic() - self.received["supervisor"] <= .3
                              and "plan" in m and status.plan_id == m["plan"].plan_id
                              and status.parameter_profile_id == result.get("profile_id")))
        for key in ("profile", "plan"):
            if key in m:
                stamp = m[key].header.stamp
                result[key + "_stamp_age_s"] = (
                    (self.node.get_clock().now().nanoseconds - stamp.sec * 10**9 - stamp.nanosec) / 1e9
                    if stamp.sec or stamp.nanosec else math.inf
                )
        if "mission" in m:
            mission = m["mission"]
            if 0 <= mission.current_seq < len(mission.waypoints):
                result["current_command"] = mission.waypoints[mission.current_seq].command
            params = yaml.safe_load((ROOT / "src/start/config/mission.yaml").read_text())["mission_manager"]["ros__parameters"]
            expected = landing_pair(LandingConfig(**{k: params[k] for k in LandingConfig.__dataclass_fields__}))
            result["mission_matches"] = ("plan" in m and len(mission.waypoints) == m["plan"].item_count
                                          and same_mission(mission.waypoints[-2:], expected))
        return result

    def begin_recording(self):
        directory = STATE / "flight_runs" / (time.strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}")
        directory.mkdir(parents=True)
        self.file = (directory / "telemetry.jsonl").open("w", buffering=1)
        from capture_diagnostics import log_inventory
        (directory / "native_log_inventory_before.json").write_text(json.dumps(log_inventory(), indent=2))
        session = json.loads((STATE / "active.json").read_text())
        files = [ROOT / "simulation/launch_assist.yaml", ROOT / "src/start/config/mission.yaml",
                 ROOT / "src/start/config/control.yaml", STATE / "geometry.json",
                 ROOT / "src/start/config/perception.yaml", STATE / "perception.yaml",
                 ROOT / "src/perception/perception/net_det.py",
                 ROOT / "src/perception/perception/net_pos.py",
                 ROOT / "src/perception/perception/detector.py",
                 ROOT / "src/perception/perception/ray_estimation.py",
                 ROOT / "src/autoland_interfaces/msg/NetDetection.msg",
                 ROOT / "src/autoland_interfaces/msg/NetEstimate.msg"]
        files.extend(ROOT/'src/autoland/autoland'/name for name in
            ('adaptive_approach.py','alignment_planner.py','airborne_preparation.py','mission_partial.py',
             'fcu_interface.py','vision_controller.py','landing_supervisor.py','timing_diagnostics.py','ros_util.py'))
        files.extend(ROOT/'tools'/name for name in
                     ('simulation_auto.py','analyse_command_timing.py','ros_env.sh'))
        files.append(ROOT/'simulation/settings.yaml')
        files.append(STATE/'merlin_landing.sdf')
        files.append(ROOT/'tools/simulation_wind.py')
        if any(__import__('yaml').safe_load((ROOT/'simulation/settings.yaml').read_text()).get('wind_enu_mps',[])):
            files.append(STATE/'models/merlinv2/model.sdf')
        cfg=yaml.safe_load((ROOT/'simulation/settings.yaml').read_text())
        if cfg.get('mission_config'):
            mission_override=Path(cfg['mission_config']).expanduser()
            files.append(mission_override if mission_override.is_absolute() else ROOT/mission_override)
        if cfg.get('control_config'):
            override=Path(cfg['control_config']).expanduser()
            files.append(override if override.is_absolute() else ROOT/override)
        context = {"session": session, "files": {}}
        native_trial = STATE / "active_native_parameter_trial.json"
        if native_trial.exists():
            context["native_parameter_trial"] = json.loads(native_trial.read_text())
        context["full_autoland"] = self.full_autoland
        contract = STATE / "attitude_contract.json"
        if self.full_autoland and contract.exists():
            context["attitude_contract"] = json.loads(contract.read_text())
        for path in files:
            context["files"][str(path.relative_to(ROOT))] = {
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "text": path.read_text(),
            }
        (directory / "context.json").write_text(json.dumps(context, indent=2) + "\n")
        return directory

    def summary(self):
        max_alt = max(self.heights, default=None)
        takeoff = (self.armed_auto and self.takeoff_index is not None
                   and self.takeoff_alt is not None and max_alt is not None
                   and max_alt >= self.takeoff_alt - 2
                   and any(i > self.takeoff_index for i in self.auto_sequences))
        gaps = list(self.state_gaps)
        if self.last_state is not None:
            gaps.append(time.monotonic() - self.last_state)
        return dict(
            scope=("full autoland experiment; observations do not establish capture"
                   if self.full_autoland else "native AUTO observations only; GUIDED/net capture not validated"),
            guided_observed=self.guided_observed,
            vision_ownership_observed=self.vision_owner_observed,
            supervisor_states_observed=sorted(self.supervisor_states),
            live_attitude_message_count=self.live_command_count,
            held_target_preview_count=self.held_target_command_count,
            terminal_committed_observed="TERMINAL_COMMITTED" in self.supervisor_states,
            net_contact_observed=self.net_contact_observed,
            zero_thrust_message_count=self.zero_thrust_count,
            close_terminal_preview_count=self.close_terminal_command_count,
            armed_auto_observed=self.armed_auto, takeoff_completion_observed=takeoff,
            native_land_item_observed=self.land_index is not None and self.land_index in self.auto_sequences,
            maximum_relative_altitude_m=max_alt,
            maximum_airspeed_mps=max(self.airspeeds, default=None),
            mission_indices_observed=sorted(self.sequences),
            armed_auto_mission_indices_observed=sorted(self.auto_sequences),
            maximum_state_gap_s=max(gaps, default=None), faults=sorted(self.faults),
        )

    def close(self):
        if self.file:
            self.file.close()
        self.node.destroy_node()
        if self.ros.ok():
            self.ros.shutdown()


def run_flight(config, args):
    from rclpy.executors import ExternalShutdownException
    managed_session(config)
    if args.command == "launch" and (not 0 <= args.wait <= 60 or not 0 <= args.record_duration <= 600):
        raise ValueError("Wait must be 0..60 s; recording duration 0..600 s")
    if args.command == "record-flight" and not 0 < args.duration <= 600:
        raise ValueError("Recording duration must be 0..600 s")
    full_autoland = getattr(args, "full_autoland", False)
    if full_autoland:
        from simulation_contract import require_contract
        require_contract(config)
    observer = FlightObserver(full_autoland=full_autoland)
    directory = observer.begin_recording()
    try:
        observer.spin(3)
        native = observer.takeoff_parameters()
        (directory / "native_takeoff_parameters.json").write_text(json.dumps(native, indent=2) + "\n")
        print("Native takeoff parameters (read-only):", json.dumps(native), flush=True)
        if args.command == "launch":
            deadline = time.monotonic() + args.wait
            previous = None
            while True:
                managed_session(config)
                flags = observer.flags()
                native = observer.takeoff_parameters()
                status = plugin_status()
                observer.spin(.15)
                snapshot = observer.snapshot(flags, status)
                native_reasons = native_takeoff_blockers(native)
                snapshot["native_takeoff_ok"] = not native_reasons
                if full_autoland:
                    require_contract(config)
                reasons = launch_blockers(snapshot, time.monotonic(), full_autoland=full_autoland)
                reasons.extend(native_reasons)
                if reasons != previous:
                    print(json.dumps({"execute_requested": args.execute, "launch_blockers": reasons,
                                      "state": snapshot.get("mode"), "armed": snapshot.get("armed"),
                                      "plugin": status}, indent=2), flush=True)
                    previous = reasons
                if not args.execute:
                    print("Dry run: no force applied. Operator must arm/select AUTO in QGC.")
                    return
                if not reasons:
                    output = gazebo_service("/trigger", "gz.msgs.Boolean", "gz.msgs.Boolean", "data: true")
                    if not re.search(r"data:\s*true\b", output):
                        raise RuntimeError("Gazebo did not confirm launch; inspect status before retrying")
                    print("One launch request accepted. No arming or mode command was sent.", flush=True)
                    observer.spin(2)
                    (directory / "launch_result.json").write_text(json.dumps(plugin_status(), indent=2) + "\n")
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Launch blocked: " + "; ".join(reasons))
                observer.spin(0.2)
            duration = args.record_duration
        else:
            duration = args.duration
        print(f"Recording flight telemetry for {duration:g} s to {directory}", flush=True)
        observer.spin(duration)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Observer stopped; any accepted launch force expires in simulation time.")
    finally:
        summary = observer.summary()
        (directory / "summary.json").write_text(json.dumps(finite_json(summary), indent=2) + "\n")
        print("Flight observations:", json.dumps(summary, indent=2), flush=True)
        print("Logs:", directory, flush=True)
        observer.close()
