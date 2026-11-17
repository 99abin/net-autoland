"""Disarmed, managed-SITL-only check of the ArduPlane partial attitude contract.

Runtime checks traverse the real FCUInterface target conversion and MAVROS.
Yaw-ignore and thrust decoding are checked against the local C++ handler.
This does not test airborne response, positive thrust, or flight readiness.
"""

import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / ".simulation"
REPORT = STATE / "attitude_contract.json"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identity(config):
    from simulation_flight import managed_session
    session = managed_session(config)
    pid = session["children"]["sitl"]
    source = Path(config["ardupilot_binary"]).parents[3] / "ArduPlane/GCS_MAVLink_Plane.cpp"
    files = [Path(config["ardupilot_binary"]), source,
             ROOT / "src/autoland/autoland/fcu_interface.py",
             ROOT / "tools/simulation_contract.py",
             Path("/opt/ros/humble/lib/libmavros_plugins.so")]
    return dict(session_pid=session["pid"], sitl_pid=pid,
                process_start=Path(f"/proc/{pid}/stat").read_text().split()[21],
                ros_domain_id=config["ros_domain_id"],
                files={str(p): digest(p) for p in files})


def require_contract(config):
    if not REPORT.exists():
        raise RuntimeError("Run bash tools/sim.sh verify-attitude while disarmed in MANUAL first")
    report = json.loads(REPORT.read_text())
    if report.get("passed") is not True or report.get("identity") != identity(config):
        raise RuntimeError("Attitude check is stale/failed; rerun verify-attitude in this SITL session")
    return report


def require_preflight():
    """A passed test is evidence, not permission to start a new stack in flight."""
    import rclpy
    from mavros_msgs.msg import State as FCUState
    rclpy.init()
    node = rclpy.create_node("autoland_startup_check")
    state, received = None, 0

    def save(message):
        nonlocal state, received
        state, received = message, time.monotonic()

    node.create_subscription(FCUState, "/mavros/state", save, 10)
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if (state is None or time.monotonic() - received > 3 or not state.connected
                or state.armed or state.mode != "MANUAL"):
            raise RuntimeError("Start autoland only with fresh connected DISARMED MANUAL SITL")
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        if names & {"fcu_interface", "landing_supervisor", "mission_manager", "parameter_manager"}:
            raise RuntimeError("Stop the existing application/preview before starting autoland")
        if node.count_publishers("/mavros/setpoint_raw/attitude"):
            raise RuntimeError("Another attitude writer exists; stop it before starting autoland")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def source_contract(config):
    """Execute the actual local handler body with minimal MAVLink/Plane doubles."""
    path = Path(config["ardupilot_binary"]).parents[3] / "ArduPlane/GCS_MAVLink_Plane.cpp"
    source = path.read_text()
    start = source.index("void GCS_MAVLINK_Plane::handle_set_attitude_target(")
    end = source.index("void GCS_MAVLINK_Plane::", start + 5)
    handler = source[start:end]
    harness = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
struct mavlink_set_attitude_target_t {float q[4]; uint8_t type_mask; float thrust;};
using mavlink_message_t = mavlink_set_attitude_target_t;
void mavlink_msg_set_attitude_target_decode(const mavlink_message_t* m,
    mavlink_set_attitude_target_t* a) {*a=*m;}
float degrees(float r) {return r*180.0f/3.14159265358979323846f;}
struct Quaternion {
 float w,x,y,z;
 Quaternion(float a,float b,float c,float d):w(a),x(b),y(c),z(d){}
 float get_euler_roll(){return atan2(2*(w*x+y*z),1-2*(x*x+y*y));}
 float get_euler_pitch(){return asin(2*(w*y-z*x));}
 float get_euler_yaw(){return atan2(2*(w*z+x*y),1-2*(y*y+z*z));}
};
namespace AP_HAL {uint32_t millis(){return 1234;}}
struct Vec {float x=0,y=0,z=0;};
struct Plane {
 int mode_guided, other; int* control_mode=&mode_guided;
 struct {Vec forced_rpy_cd,last_forced_rpy_ms;
 float forced_throttle=0; uint32_t last_forced_throttle_ms=0;} guided_state;
} plane;
struct GCS_MAVLINK_Plane {void handle_set_attitude_target(const mavlink_message_t&);};
'''
    harness += handler
    harness += r'''
int main(){
 GCS_MAVLINK_Plane g;
 for(float sign: {-1.f,1.f}) {
  float r=sign*.1f,p=-sign*.07f,y=.3f;
  float cr=cos(r/2),sr=sin(r/2),cp=cos(p/2),sp=sin(p/2),cy=cos(y/2),sy=sin(y/2);
  mavlink_message_t m={{cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
    cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy},132,.37f};
  plane.guided_state={}; plane.guided_state.forced_rpy_cd.z=765;
  g.handle_set_attitude_target(m);
  auto& s=plane.guided_state;
  assert(fabs(s.forced_rpy_cd.x-degrees(r)*100)<.01);
  assert(fabs(s.forced_rpy_cd.y-degrees(p)*100)<.01);
  assert(s.forced_rpy_cd.z==765 && s.last_forced_rpy_ms.z==0);
  assert(fabs(s.forced_throttle-37)<.001 && s.last_forced_throttle_ms==1234);
  plane.control_mode=&plane.other; s={}; g.handle_set_attitude_target(m);
  assert(s.last_forced_rpy_ms.x==0 && s.last_forced_throttle_ms==0);
  plane.control_mode=&plane.mode_guided;
 }
}
'''
    harness = "#include <initializer_list>\n" + harness
    with tempfile.TemporaryDirectory(prefix="autoland-contract-") as folder:
        cpp, binary = Path(folder) / "handler.cpp", Path(folder) / "handler"
        cpp.write_text(harness)
        subprocess.run(["/usr/bin/c++", "-std=c++17", str(cpp), "-o", str(binary)],
                       check=True, capture_output=True, text=True, timeout=20)
        subprocess.run([str(binary)], check=True, timeout=5)
    return str(path)


def verify(config):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from mavros_msgs.msg import State as FCUState, NavControllerOutput, AttitudeTarget
    from mavros_msgs.srv import SetMode, CommandLong
    from autoland.fcu_interface import FCUInterface

    stamp = identity(config)
    REPORT.unlink(missing_ok=True)  # A failed recheck must revoke the old pass.
    source = source_contract(config)
    rclpy.init()
    node = rclpy.create_node("sitl_attitude_contract_check")
    latest, received = {}, {}
    samples = []

    def save(key, msg):
        latest[key], received[key] = msg, time.monotonic()

    node.create_subscription(FCUState, "/mavros/state", lambda m: save("state", m), 10)
    node.create_subscription(NavControllerOutput, "/mavros/nav_controller_output/output",
                             lambda m: save("nav", m), qos_profile_sensor_data)
    pub = node.create_publisher(AttitudeTarget, "/mavros/setpoint_raw/attitude", 10)
    mode = node.create_client(SetMode, "/mavros/set_mode")
    rate = node.create_client(CommandLong, "/mavros/cmd/command")
    requested = False
    restored = False

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def guard(expected=None):
        s = latest.get("state")
        if (s is None or time.monotonic() - received["state"] > 3
                or not s.connected or s.armed or (expected and s.mode != expected)):
            raise RuntimeError("Attitude test requires fresh connected DISARMED FCU in " + str(expected))
        if identity(config) != stamp:
            raise RuntimeError("Managed SITL identity changed")

    def set_mode(name):
        if not mode.wait_for_service(timeout_sec=2):
            raise RuntimeError("MAVROS SetMode unavailable")
        req = SetMode.Request()
        req.custom_mode = name
        future = mode.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        end = time.monotonic() + 6
        while time.monotonic() < end:
            spin(.1)
            guard()
            if latest["state"].mode == name:
                return
        raise RuntimeError("FCU did not confirm " + name)

    try:
        spin(3)
        guard("MANUAL")
        names = [name for name, _ in node.get_node_names_and_namespaces()]
        if "fcu_interface" in names or node.count_publishers("/mavros/setpoint_raw/attitude") != 1:
            raise RuntimeError("Stop the landing program/other attitude writers before verifying")
        # Request telemetry only: MAV_CMD_SET_MESSAGE_INTERVAL, NAV_CONTROLLER_OUTPUT=62.
        if not rate.wait_for_service(timeout_sec=2):
            raise RuntimeError("MAVROS telemetry-rate command unavailable")
        req = CommandLong.Request()
        req.command, req.param1, req.param2 = 511, 62., 100000.
        future = rate.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        if not future.done() or not future.result() or not future.result().success:
            raise RuntimeError("FCU rejected navigation telemetry rate request")
        requested = True
        set_mode("GUIDED")
        for roll, pitch in ((6., 4.), (-6., -4.), (0., 0.)):
            start = time.monotonic()
            matching = 0
            while time.monotonic() - start < 3:
                guard("GUIDED")
                command = SimpleNamespace(roll_rad=math.radians(roll),
                                          pitch_rad=math.radians(pitch), yaw_enu_rad=.4, thrust=0.)
                target = FCUInterface.target(SimpleNamespace(get_clock=node.get_clock), command)
                pub.publish(target)
                spin(.05)
                nav = latest.get("nav")
                if nav is not None and received["nav"] > start + .3:
                    # MAVROS reports NED/FRD demand angles in degrees.
                    if abs(nav.nav_roll - roll) < 1 and abs(nav.nav_pitch - pitch) < 1:
                        matching += 1
                    else:
                        matching = 0
                    if matching >= 4:
                        samples.append(dict(roll=roll, pitch=pitch,
                                            nav_roll=nav.nav_roll, nav_pitch=nav.nav_pitch))
                        break
            else:
                raise RuntimeError(f"Attitude response mismatch at roll={roll}, pitch={pitch}; "
                                   f"last nav={latest.get('nav')}")
        set_mode("MANUAL")
        restored = True
        guard("MANUAL")
        REPORT.write_text(json.dumps(dict(
            passed=True, identity=stamp, samples=samples, source_handler=source,
            scope="Disarmed runtime roll/pitch via FCUInterface and MAVROS; local C++ handler "
                  "yaw-ignore/thrust/mode decoding. No positive runtime thrust or airborne validation."
        ), indent=2) + "\n")
        print("PASS: disarmed SITL attitude contract; MANUAL restored. Report:", REPORT)
    finally:
        if requested and not restored:
            # Never override a newly selected pilot mode or an armed FCU.
            try:
                guard("GUIDED")
                set_mode("MANUAL")
            except RuntimeError as exc:
                print("MANUAL restore not confirmed; inspect QGC:", exc)
        node.destroy_node()
        rclpy.shutdown()
