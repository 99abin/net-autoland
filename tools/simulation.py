"""Local Gazebo/ArduPlane simulation; autonomous flight requires auto-test --execute."""

import argparse
import errno
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / ".simulation"


def mesh_with_normals(source):
    """Copy scene OBJ meshes with normals required by Gazebo's DART collision loader."""
    target = STATE / "meshes" / source.name
    if target.exists() and target.stat().st_mtime >= source.stat().st_mtime:
        return target
    vertices, faces = [], []
    for line in source.read_text().splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append([float(v) for v in fields[1:4]])
        elif fields and fields[0] == "f":
            faces.append([int(v.split("/")[0]) - 1 for v in fields[1:]])
    vertices = np.asarray(vertices)
    normals = np.zeros_like(vertices)
    for face in faces:
        a, b, c = vertices[face[:3]]
        n = np.cross(b - a, c - a)
        for i in face:
            normals[i] += n
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths < 1e-12):
        raise ValueError(f"Degenerate normals in {source}")
    normals /= lengths[:, None]
    target.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# Generated copy with vertex normals; original mesh is unchanged"]
    lines.extend("v " + " ".join(map(str, v)) for v in vertices)
    lines.extend("vn " + " ".join(map(str, v)) for v in normals)
    lines.append("s 1")
    lines.extend("f " + " ".join(f"{i+1}//{i+1}" for i in face) for face in faces)
    target.write_text("\n".join(lines) + "\n")
    return target


def settings():
    data = yaml.safe_load((ROOT / "simulation/settings.yaml").read_text())
    for key in ("aircraft_directory", "source_world", "ardupilot_binary",
                "ardupilot_plugin_directory", "qgroundcontrol"):
        data[key] = str(Path(data[key]).expanduser().resolve())
    return data


def prepare(cfg, launch_assist=False):
    """Derive a combined world, retaining Merlin's original complete spawn pose."""
    aircraft = Path(cfg["aircraft_directory"])
    source = Path(cfg["source_world"])
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    world.set("name", "merlin_landing")
    includes = [e for e in world.findall("include")
                if e.findtext("uri") == "model://zephyr_with_ardupilot"]
    if len(includes) != 1:
        raise ValueError("Expected exactly one Zephyr include in the landing world")
    original = ET.parse(aircraft / "merlinv2_world.sdf").find("world")
    spawn = next(e for e in original.findall("include")
                 if e.findtext("uri") == "model://merlinv2")
    includes[0].find("uri").text = "model://merlinv2"
    includes[0].find("pose").attrib.clear()
    includes[0].find("pose").attrib.update(spawn.find("pose").attrib)
    includes[0].find("pose").text = spawn.findtext("pose")
    world.find("physics/real_time_factor").text = "1"
    if not any(p.get("name") == "gz::sim::systems::Sensors"
               for p in world.findall("plugin")):
        p = ET.SubElement(world, "plugin", filename="gz-sim-sensors-system",
                          name="gz::sim::systems::Sensors")
        ET.SubElement(p, "render_engine").text = "ogre2"
    if launch_assist:
        from flight_checks import validate_assist_config
        assist = yaml.safe_load((ROOT / "simulation/launch_assist.yaml").read_text())
        validate_assist_config(assist)
        p = ET.SubElement(world, "plugin", filename=str(
            STATE / "plugins/build/libAutolandLaunchAssist.so"), name="autoland::LaunchAssist")
        for name, value in assist.items():
            ET.SubElement(p, name).text = str(value)
    # The generated world lives elsewhere; preserve the original relative meshes.
    for uri in world.iter("uri"):
        if uri.text and "://" not in uri.text:
            asset = (source.parent / uri.text).resolve()
            if not asset.is_file():
                raise FileNotFoundError(asset)
            if asset.suffix == ".obj":
                asset = mesh_with_normals(asset)
            uri.text = asset.as_uri()
    STATE.mkdir(exist_ok=True)
    ET.indent(tree)
    tree.write(STATE / "merlin_landing.sdf", encoding="utf-8", xml_declaration=True)

    # The installed bridge preserves Gazebo's scoped camera frame and timestamps.
    perception = yaml.safe_load((ROOT / "src/start/config/perception.yaml").read_text())
    perception["net_pos"]["ros__parameters"]["camera_translation_body_m"] = [0.451, 0., 0.008]
    perception["net_pos"]["ros__parameters"]["camera_frame"] = "merlinv2/body/nose_camera"
    (STATE / "perception.yaml").write_text(yaml.safe_dump(perception, sort_keys=False))
    panel = world.find("model[@name='among_us_landing_panel']")
    if panel is None or panel.findtext("pose") != "0 -50 0 0 0 0":
        raise ValueError("Landing panel moved: recalculate mission coordinates before use")
    geo = world.find("spherical_coordinates")
    if float(geo.findtext("heading_deg")) != 0:
        raise ValueError("World heading changed: recalculate the ENU-to-NED mapping")
    origin_lat = float(geo.findtext("latitude_deg"))
    origin_lon = float(geo.findtext("longitude_deg"))
    # Installed ArduPilot SIM_JSON receives NED position, not Gazebo GPS latitude.
    # Match AP_Common/Location::offset and AP_Math/definitions.h (1e-7 degrees).
    net_lat = (int(origin_lat * 1e7) + int(-50 * 89.83204953368922)) / 1e7
    metadata = dict(world=str(STATE / "merlin_landing.sdf"),
                    spawn_pose=spawn.findtext("pose"),
                    home=f"{origin_lat},{origin_lon},{geo.findtext('elevation')},90",
                    net_world_enu=[0, -50, 1.35],
                    net_lat=net_lat, net_lon=origin_lon,
                    approach_heading_deg=180., approach_distance_m=500.,
                    launch_assist=launch_assist)
    (STATE / "geometry.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return metadata


def environment(cfg):
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(cfg["ros_domain_id"]), ROS_LOCALHOST_ONLY="1",
               GZ_PARTITION=cfg["gazebo_partition"], GZ_IP="127.0.0.1",
               ROS_LOG_DIR=str(STATE / "ros_logs"))
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([
        str(Path(cfg["aircraft_directory"]) / "models"),
        str(Path(cfg["source_world"]).parent / "models"),
        str(Path(cfg["source_world"]).parent),
    ])
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = cfg["ardupilot_plugin_directory"]
    env["MAVROS_UAS_EXECUTOR_THREADS"] = str(cfg.get("mavros_executor_threads", 8))
    return env


def doctor(cfg, allow_missing_mavros=False, qgc=True):
    required = [cfg["ardupilot_binary"],
                str(Path(cfg["ardupilot_plugin_directory"]) / "libArduPilotPlugin.so"),
                "/usr/bin/gz", "/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge",
                str(ROOT / "install_handoff/setup.bash")]
    if qgc:
        required.append(cfg["qgroundcontrol"])
    if not allow_missing_mavros:
        required.extend(["/opt/ros/humble/share/mavros/package.xml",
                         "/opt/ros/humble/share/mavros_extras/package.xml",
                         "/usr/share/GeographicLib/geoids/egm96-5.pgm"])
    missing = [p for p in required if not Path(p).exists()]
    for p in missing:
        print(f"MISSING: {p}", file=sys.stderr)
    if missing:
        print("MAVROS install: sudo apt-get install --no-install-recommends ros-humble-mavros ros-humble-mavros-extras",
              file=sys.stderr)
        raise RuntimeError("Simulator prerequisites are incomplete")


def stop(_cfg):
    active = STATE / "active.json"
    if not active.exists():
        print("No managed simulation session is recorded")
        return
    record = json.loads(active.read_text())
    pid = record["pid"]
    cmdline = Path(f"/proc/{pid}/cmdline")
    if not cmdline.exists() or b"tools/simulation.py" not in cmdline.read_bytes():
        raise RuntimeError("Recorded launcher is no longer running; no processes were signalled")
    os.kill(pid, signal.SIGINT)
    print(f"Requested shutdown of managed simulator launcher {pid}")
    deadline = time.monotonic() + 40
    while active.exists() and time.monotonic() < deadline:
        time.sleep(0.1)
    if active.exists():
        raise RuntimeError("Simulator shutdown has not completed; inspect session logs before restarting")
    print("Managed simulator shutdown completed")


def ensure_no_existing_mavros(cfg):
    """Refuse duplicate ROS services even when an old launcher's lock is gone."""
    import rclpy
    rclpy.init(args=[], domain_id=int(cfg["ros_domain_id"]))
    node = rclpy.create_node("simulation_startup_guard")
    try:
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        if (node.count_publishers("/mavros/state") > 0
                or names & {"mavros", "mavros_router", "mavros_uas_container"}):
            raise RuntimeError(
                f"MAVROS already exists in ROS domain {cfg['ros_domain_id']}. "
                "Use the existing session or stop it completely before starting another. "
                "An old MAVROS process may still be alive after its launcher exited."
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def preview(cfg, upload=False, full_autoland=False):
    active = STATE / "active.json"
    record = json.loads(active.read_text()) if active.exists() else {}
    if "preview" in record.get("children", {}):
        raise RuntimeError("Preview is already running; start the simulator with --no-preview first")
    if upload and "mavros" not in record.get("children", {}):
        raise RuntimeError("Mission upload requires a managed localhost SITL/MAVROS session")
    if full_autoland:
        from simulation_contract import require_contract, require_preflight
        from simulation_flight import managed_session
        managed_session(cfg)
        require_contract(cfg)
        os.environ.update(environment(cfg))
        require_preflight()
    # Reuse the running world's generated perception settings. Regenerating its
    # SDF here would overwrite a launch-enabled world with the preview variant.
    if not (STATE / "perception.yaml").exists():
        prepare(cfg)
    command = ["bash", "tools/run.sh", "use_sim_time:=true",
               "clock_topic:=/simulation/clock",
               "velocity_topic:=/simulation/velocity_local_enu",
               f"perception_config:={STATE / 'perception.yaml'}",
               f"fc_profile:={ROOT / 'simulation/fc_parameters.yaml'}"]
    if upload:
        command.append("upload_enabled:=true")
    if full_autoland:
        command.extend(["handoff_enabled:=true", "live_output:=true",
                        "partial_attitude_verified:=true"])
        print("LOCAL SITL AUTOLAND: handoff and live simulated commands enabled. "
              "Enable the supervisor after mission verification; arm/select AUTO in QGC.", flush=True)
    os.execvpe(command[0], command, environment(cfg))


def check(cfg, timeout, require_mavros):
    """Read ROS topics only; do not call mission/parameter/mode services."""
    os.environ.update(environment(cfg))
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import TwistStamped
    from rosgraph_msgs.msg import Clock
    from mavros_msgs.msg import State as FCUState, HomePosition
    from autoland_interfaces.msg import NetDetection, NetEstimate, ParameterProfileStatus
    rclpy.init()
    node = rclpy.create_node("simulation_observer")
    received = {}
    received_at = {}
    fcu_times = []

    def observe(key, message):
        received[key] = message
        received_at[key] = time.monotonic()
        if key == "fcu":
            fcu_times.append(received_at[key])

    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = []
    for name, typ, topic, qos in [
        ("clock", Clock, "/simulation/clock", 10),
        ("image", Image, "/camera/image_raw", qos_profile_sensor_data),
        ("calibration", CameraInfo, "/camera/camera_info", qos_profile_sensor_data),
        ("detection", NetDetection, "/autoland/net_detection", 10),
        ("estimate", NetEstimate, "/autoland/net_estimate", 10),
        ("fcu", FCUState, "/mavros/state", 10),
        ("velocity", TwistStamped, "/simulation/velocity_local_enu", qos_profile_sensor_data),
        ("home", HomePosition, "/mavros/home_position/home", transient),
        ("profile", ParameterProfileStatus, "/autoland/parameter_profile", transient),
    ]:
        subscriptions.append(node.create_subscription(
            typ, topic, lambda message, key=name: observe(key, message), qos))
    began = time.monotonic()
    try:
        while time.monotonic() - began < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
        for key in ("clock", "image", "calibration", "detection", "estimate", "fcu", "velocity", "home", "profile"):
            msg = received.get(key)
            if msg is None:
                optional = " (not required before starting preview)" if key in (
                    "detection", "estimate", "profile") else ""
                print(f"{key}: no message{optional}")
            elif key == "image":
                print(f"image: {msg.width}x{msg.height} {msg.encoding}, frame={msg.header.frame_id}")
            elif key == "calibration":
                print(f"calibration: fx={msg.k[0]:.3f}, fy={msg.k[4]:.3f}, "
                      f"model={msg.distortion_model}, frame={msg.header.frame_id}")
            elif key == "fcu":
                print(f"fcu: connected={msg.connected}, armed={msg.armed}, mode={msg.mode}")
                age = time.monotonic() - received_at[key]
                gaps = [b - a for a, b in zip(fcu_times, fcu_times[1:])]
                print(f"fcu stream: {len(fcu_times)} samples, latest age={age:.2f}s, "
                      f"largest observed gap={max(gaps, default=0):.2f}s")
            elif key == "home":
                print(f"home: {msg.geo.latitude}, {msg.geo.longitude}, altitude={msg.geo.altitude}")
            elif key == "velocity":
                print(f"velocity: frame={msg.header.frame_id}, latest age={time.monotonic() - received_at[key]:.2f}s")
            elif key in ("estimate", "profile"):
                print(f"{key}: {msg.reason}")
            elif key == "clock":
                print(f"clock: {msg.clock.sec}.{msg.clock.nanosec:09d}")
            else:
                print(f"detection: valid={msg.valid}, score={msg.score}")
        required = ["clock", "image", "calibration"]
        if require_mavros:
            required.extend(["fcu", "velocity"])
        missing = [key for key in required if key not in received]
        if missing:
            raise RuntimeError("Required simulator streams are missing: " + ", ".join(missing))
        if require_mavros and (received["velocity"].header.frame_id != "map"
                              or time.monotonic() - received_at["velocity"] > .5):
            raise RuntimeError("Corrected ENU velocity must be fresh (<0.5 s) in map")
        if received["clock"].clock.sec == 0:
            raise RuntimeError("Simulation clock is not advancing")
        expected_frame = yaml.safe_load((STATE / "perception.yaml").read_text())["net_pos"]["ros__parameters"]["camera_frame"]
        if (received["image"].header.frame_id != expected_frame
                or received["calibration"].header.frame_id != expected_frame):
            raise RuntimeError("Camera image frame does not match perception configuration")
        if received["calibration"].k[0] <= 0:
            raise RuntimeError("Camera calibration is missing intrinsics")
        if require_mavros and not received["fcu"].connected:
            raise RuntimeError("MAVROS is not connected to SITL")
        if require_mavros and time.monotonic() - received_at["fcu"] > 3:
            raise RuntimeError("FCU state is stale (older than the application's 3-second limit)")
        if require_mavros and any(b - a > 3 for a, b in zip(fcu_times, fcu_times[1:])):
            raise RuntimeError("FCU state had a gap above the application's 3-second limit during this check")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def start(cfg, args):
    doctor(cfg, args.without_mavros, not args.no_qgc)
    lock = (STATE / "session.lock").open("w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("Another managed session is running; use bash tools/sim.sh stop")
    ensure_no_existing_mavros(cfg)
    if args.launch_assist:
        if args.without_mavros:
            raise ValueError("Launch assistance requires managed MAVROS")
        build_launch_plugin()
    geometry = prepare(cfg, launch_assist=args.launch_assist)
    # Refuse to reuse another simulator's local ports. Do not probe TCP by
    # connecting: a connection could consume the simulator's one MAVLink client.
    for kind, port in ((socket.SOCK_DGRAM, 9002), (socket.SOCK_STREAM, 5762),
                       (socket.SOCK_STREAM, 5763)):
        with socket.socket(socket.AF_INET, kind) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", port))
    run_dir = STATE / "logs" / time.strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True)
    sitl_dir = STATE / "sitl"
    sitl_dir.mkdir(exist_ok=True)
    env = environment(cfg)
    children, handles = [], []
    shutdown = False

    def interrupted(_signum, _frame):
        nonlocal shutdown
        shutdown = True

    old_handlers = {s: signal.signal(s, interrupted)
                    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}

    def launch(name, command, cwd=ROOT):
        output = (run_dir / f"{name}.log").open("w")
        handles.append(output)
        process = subprocess.Popen(command, cwd=cwd, env=env, stdout=output,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        children.append((name, process))
        print(f"{name}: PID {process.pid}, log {output.name}", flush=True)

    try:
        gz = ["/usr/bin/gz", "sim", "-s", "-r", "-v", "3", geometry["world"]]
        if args.headless:
            gz.append("--headless-rendering")
        launch("gazebo", gz)
        launch("sitl", [cfg["ardupilot_binary"], "--model", "JSON", "--home", geometry["home"],
                        "--speedup", "1", "--defaults",
                        str(Path(cfg["aircraft_directory"]) / "merlinv2.parm"),
                        "--serial0", "udpclient:127.0.0.1:14550",
                        "--serial1", "tcp:5762", "--serial2", "tcp:5763"], sitl_dir)
        launch("camera_bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                "--ros-args", "-p", f"config_file:={ROOT / 'simulation/bridge.yaml'}"])
        if not args.without_mavros:
            launch("mavros_clock", ["/usr/bin/python3", "tools/simulation_clock.py"])
            # SITL creates SERIAL1 after receiving Gazebo JSON sensor data.
            # Wait without opening a connection, which would consume its client.
            deadline = time.monotonic() + 45
            print("Waiting for SITL SERIAL1 on TCP 5762...", flush=True)
            while not shutdown:
                for name, child in children:
                    if child.poll() is not None:
                        raise RuntimeError(f"{name} exited ({child.returncode}); see {run_dir / (name + '.log')}")
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
                    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    try:
                        probe.bind(("127.0.0.1", 5762))
                    except OSError as exc:
                        if exc.errno != errno.EADDRINUSE:
                            raise
                        break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"SITL SERIAL1 did not open; inspect {run_dir / 'sitl.log'} and gazebo.log")
                time.sleep(0.1)
            if shutdown:
                return
            launch("mavros", ["ros2", "launch", str(ROOT / "simulation/mavros.launch.py")])
            launch("velocity_frame", ["/usr/bin/python3", "tools/simulation_velocity.py"])
        if not args.no_preview:
            launch("preview", ["bash", "tools/run.sh", "use_sim_time:=true",
                               "clock_topic:=/simulation/clock",
                               "velocity_topic:=/simulation/velocity_local_enu",
                               f"perception_config:={STATE / 'perception.yaml'}",
                               f"fc_profile:={ROOT / 'simulation/fc_parameters.yaml'}"])
        if not args.headless:
            launch("gazebo_gui", ["/usr/bin/gz", "sim", "-g", "-v", "2"])
        if not args.no_qgc:
            # QGC permits one instance. Reuse the user's open application and
            # leave it outside this launcher's shutdown lifecycle.
            running = subprocess.run(["pgrep", "-u", str(os.getuid()), "-x", "QGroundControl"],
                                     stdout=subprocess.DEVNULL, check=False)
            if running.returncode == 0:
                print("qgc: reusing the running QGroundControl application", flush=True)
            elif running.returncode == 1:
                launch("qgc", [cfg["qgroundcontrol"]])
            else:
                raise RuntimeError("Could not check for an existing QGroundControl process")
        active = dict(pid=os.getpid(), log_directory=str(run_dir),
                      ros_domain_id=cfg["ros_domain_id"],
                      launch_assist=args.launch_assist,
                      children={name: p.pid for name, p in children})
        (STATE / "active.json").write_text(json.dumps(active, indent=2) + "\n")
        print("Preview only. No arming, mission upload, parameter writes or handoff enabled.", flush=True)
        began = time.monotonic()
        while not shutdown and (not args.duration or time.monotonic() - began < args.duration):
            for name, child in children:
                if child.poll() is not None:
                    raise RuntimeError(f"{name} exited ({child.returncode}); see {run_dir / (name + '.log')}")
            time.sleep(0.25)
    finally:
        # Stop only process groups created by this launcher; never broad pkill.
        for _, child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and any(p.poll() is None for _, p in children):
            time.sleep(0.1)
        for _, child in children:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
        for _, child in children:
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for handle in handles:
            handle.close()
        (STATE / "active.json").unlink(missing_ok=True)
        for s, handler in old_handlers.items():
            signal.signal(s, handler)


def build_launch_plugin():
    build = STATE / "plugins/build"
    build.mkdir(parents=True, exist_ok=True)
    log = build / "build.log"
    print(f"Building Gazebo launch plugin (log: {log})...", flush=True)
    with log.open("w") as output:
        for command in (["cmake", "-S", str(ROOT / "simulation/plugins"), "-B", str(build),
                         "-DCMAKE_BUILD_TYPE=RelWithDebInfo"],
                        ["cmake", "--build", str(build), "-j2"]):
            result = subprocess.run(command, stdout=output, stderr=subprocess.STDOUT, check=False)
            if result.returncode:
                raise RuntimeError(f"Launch plugin build failed; inspect {log}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    preparation = sub.add_parser("prepare")
    preparation.add_argument("--launch-assist", action="store_true")
    sub.add_parser("build-launch-plugin")
    sub.add_parser("doctor")
    sub.add_parser("stop")
    sub.add_parser("status")
    sub.add_parser("shell")
    mission = sub.add_parser("mission-altitude", help="Back up and edit local SITL waypoint heights while disarmed")
    mission.add_argument("--altitude", required=True, type=float, help="Metres above HOME; preserves HOME and LAND")
    mission.add_argument("--apply", action="store_true", help="Upload and verify; otherwise read-only")
    app = sub.add_parser("preview")
    app.add_argument("--upload-mission", action="store_true",
                     help="Explicitly allow preflight append/upload/readback to localhost SITL")
    full = sub.add_parser("autoland", help="Enable full handoff/control in verified managed SITL")
    full.add_argument("--upload-mission", action="store_true")
    sub.add_parser("verify-attitude", help="Disarmed local SITL GUIDED attitude interface test")
    sub.add_parser("recover", help="Confirm RTL in the managed local SITL; never overrides pilot modes")
    parameters = sub.add_parser("native-parameters", aliases=["native-pitch"],
                                help="Inspect/apply a backed-up local-SITL native parameter trial")
    parameters.add_argument("--file", required=True, help="Explicit simulator-only YAML; baseline.yaml can restore values")
    parameters.add_argument("--apply", action="store_true", help="Write and verify the reviewed pitch values; otherwise read-only")
    auto = sub.add_parser("auto-test", help="Explicit managed-SITL-only autonomous flight experiment")
    auto.add_argument("--execute", action="store_true",
                      help="Allow reviewed takeoff parameter writes, arming, AUTO and one simulated launch")
    auto.add_argument("--duration", type=float, default=900,
                      help="Maximum wall-clock recording time after launch")
    observer = sub.add_parser("check")
    observer.add_argument("--timeout", type=float, default=8)
    observer.add_argument("--require-mavros", action="store_true")
    launch = sub.add_parser("start")
    launch.add_argument("--headless", action="store_true")
    launch.add_argument("--no-qgc", action="store_true")
    launch.add_argument("--no-preview", action="store_true")
    launch.add_argument("--without-mavros", action="store_true")
    launch.add_argument("--duration", type=float, default=0, help="Stop after this many wall-clock seconds")
    launch.add_argument("--launch-assist", action="store_true",
                        help="Load the optional finite Gazebo catapult (does not trigger it)")
    catapult = sub.add_parser("launch", help="Check native-AUTO launch readiness; dry run by default")
    catapult.add_argument("--execute", action="store_true",
                          help="Explicitly request one Gazebo catapult after all guards pass")
    catapult.add_argument("--wait", type=float, default=0,
                          help="Wait up to 60 wall seconds for operator arming/AUTO in QGC")
    catapult.add_argument("--record-duration", type=float, default=180)
    catapult.add_argument("--full-autoland", action="store_true",
                          help="Allow verified live handoff flags instead of native-only flags")
    recording = sub.add_parser("record-flight", help="Read-only native flight telemetry recorder")
    recording.add_argument("--duration", type=float, default=180)
    recording.add_argument("--full-autoland", action="store_true")
    args = parser.parse_args()
    cfg = settings()
    if args.command == "prepare":
        print(json.dumps(prepare(cfg, launch_assist=args.launch_assist), indent=2))
    elif args.command == "build-launch-plugin":
        build_launch_plugin()
    elif args.command == "doctor":
        doctor(cfg)
        print("Simulator prerequisites found")
    elif args.command == "stop":
        stop(cfg)
    elif args.command == "status":
        p = STATE / "active.json"
        print(p.read_text() if p.exists() else "No managed simulation session is recorded")
    elif args.command == "mission-altitude":
        from simulation_mission import run
        os.environ.update(environment(cfg))
        run(cfg, args)
    elif args.command in ("preview", "autoland"):
        preview(cfg, args.upload_mission, full_autoland=args.command == "autoland")
    elif args.command == "verify-attitude":
        from simulation_contract import verify
        os.environ.update(environment(cfg))
        verify(cfg)
    elif args.command == "auto-test":
        from simulation_auto import run
        os.environ.update(environment(cfg))
        run(cfg, args)
    elif args.command == "recover":
        from simulation_recover import recover
        os.environ.update(environment(cfg))
        recover(cfg)
    elif args.command in ("native-pitch", "native-parameters"):
        from simulation_parameters import run
        os.environ.update(environment(cfg))
        run(cfg, args)
    elif args.command == "shell":
        os.execvpe("/bin/bash", ["bash", "--noprofile", "--norc", "-i"], environment(cfg))
    elif args.command == "check":
        check(cfg, args.timeout, args.require_mavros)
    elif args.command in ("launch", "record-flight"):
        from simulation_flight import run_flight
        os.environ.update(environment(cfg))
        run_flight(cfg, args)
    else:
        start(cfg, args)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, StopIteration, ET.ParseError,
            subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
        print(f"Simulation setup: {exc}", file=sys.stderr)
        sys.exit(1)
