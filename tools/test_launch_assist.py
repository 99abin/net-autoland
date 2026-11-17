"""Exercise the finite launch plugin in an isolated five-kilogram box world.

This never connects to ArduPilot or ROS and never loads the aircraft model.
"""

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT / ".simulation/plugins/build/libAutolandLaunchAssist.so"
PREFIX = "/world/launch_assist_test/launch_assist"


def main():
    if not PLUGIN.is_file():
        raise RuntimeError("Build first: bash tools/sim.sh build-launch-plugin")
    env = os.environ.copy()
    env.update(GZ_PARTITION=f"autoland_launch_unit_{os.getpid()}", GZ_IP="127.0.0.1")
    with tempfile.TemporaryDirectory(prefix="autoland-launch-test-") as folder:
        directory = Path(folder)
        world = directory / "test.sdf"
        world.write_text(f'''<sdf version="1.9"><world name="launch_assist_test">
          <physics name="physics" type="ignore"><max_step_size>0.001</max_step_size>
          <real_time_factor>1</real_time_factor></physics>
          <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
          <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
          <plugin filename="{PLUGIN}" name="autoland::LaunchAssist">
            <service_prefix>{PREFIX}</service_prefix></plugin>
          <model name="ground"><static>true</static><link name="ground">
          <collision name="ground"><geometry><plane><normal>0 0 1</normal></plane>
          </geometry></collision></link></model>
          <model name="merlinv2"><pose>0 0 0.101 0 0 0</pose><link name="body">
          <inertial><mass>5</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz>
          </inertia></inertial><collision name="box"><geometry><box><size>.2 .2 .2</size>
          </box></geometry></collision></link></model></world></sdf>''')
        with (directory / "gazebo.log").open("w") as output:
            process = subprocess.Popen(["/usr/bin/gz", "sim", "-s", "-v", "2", str(world)],
                                       env=env, cwd=directory, stdout=output,
                                       stderr=subprocess.STDOUT, start_new_session=True)

            def service(name, reqtype, reptype, request=""):
                command = ["/usr/bin/gz", "service", "-s", PREFIX + name,
                           "--reqtype", reqtype, "--reptype", reptype,
                           "--timeout", "500", "--req", request]
                return subprocess.run(command, env=env, capture_output=True,
                                      text=True, timeout=3).stdout

            def status():
                text = service("/status", "gz.msgs.Empty", "gz.msgs.StringMsg")
                match = re.search(r'^data:\s*(".*")\s*$', text, re.MULTILINE)
                return json.loads(json.loads(match.group(1))) if match else None

            def control(paused):
                result = subprocess.run([
                    "/usr/bin/gz", "service", "-s", "/world/launch_assist_test/control",
                    "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1000", "--req", f"pause: {str(paused).lower()}",
                ], env=env, capture_output=True, text=True, timeout=3)
                assert "data: true" in result.stdout, result.stdout + result.stderr

            try:
                deadline = time.monotonic() + 15
                current = None
                while current is None and time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError((directory / "gazebo.log").read_text())
                    current = status()
                assert current is not None, "Plugin service did not appear"
                # Proto3 omits default false fields in text output. Check both
                # the absence of true and the plugin's authoritative used flag.
                assert "data: true" not in service("/trigger", "gz.msgs.Boolean", "gz.msgs.Boolean", "data: true"), "Paused launch accepted"
                assert not status()["used"], "Paused request consumed the launch"
                control(False)
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    current = status()
                    if current and current["ready"]:
                        break
                assert current and current["ready"], current
                assert "data: true" not in service("/trigger", "gz.msgs.Boolean", "gz.msgs.Boolean", "data: false")
                assert not status()["used"], "A dry-run request consumed the launch"
                assert "data: true" in service("/trigger", "gz.msgs.Boolean", "gz.msgs.Boolean", "data: true")
                assert "data: true" not in service("/trigger", "gz.msgs.Boolean", "gz.msgs.Boolean", "data: true"), "Duplicate launch accepted"
                deadline = time.monotonic() + 10
                while time.monotonic() < deadline:
                    current = status()
                    if current and current["state"] == "COMPLETE":
                        break
                assert current and current["state"] == "COMPLETE", current
                assert not current["active"] and current["used"], current
                assert 0 < current["applied_steps"] <= 750, current
                assert current["speed_mps"] > 10, "No physical launch acceleration observed"
                steps = current["applied_steps"]
                time.sleep(.3)
                after = status()
                assert after["applied_steps"] == steps and not after["active"], after
                print("PASS: paused/dry-run rejection, one-shot launch, physical acceleration, "
                      "finite force expiry and no residual force")
                print(json.dumps(current, indent=2))
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()


if __name__ == "__main__":
    main()
