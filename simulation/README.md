# Merlin V2 + landing net: local SITL setup

For the command checklist, use [simulation_command.md](../simulation_command.md).
That guide includes the explicit `verify-attitude`, `autoland --upload-mission`
and `launch --full-autoland` workflow for exercising early GUIDED handoff and
live simulated control. A passing check is required for each simulator session;
normal preview defaults remain disabled. Net capture and close-range terminal
control remain experimental; see the guide's limits and recovery instructions.
For the optional operator-triggered Gazebo catapult and native AUTO recording
stage, use [NATIVE_AUTO_FLIGHT_TEST.md](../NATIVE_AUTO_FLIGHT_TEST.md).

This setup uses the existing Gazebo Harmonic 8 installation, ArduPlane SITL
binary, QGroundControl AppImage and ROS 2 Humble. It reads the original model
folders without modifying them. Local paths are in `simulation/settings.yaml`.

## Current landing configuration

`src/start/config/mission.yaml` now describes this simulator scene:

| Setting | Value |
| --- | --- |
| Net target latitude | -35.3637111 |
| Net target longitude | 149.165237 |
| Target altitude relative to HOME | 1.26 m |
| Aircraft travel direction toward net | 180 degrees, south |
| Approach waypoint distance before LAND | 500 m |
| Approach waypoint altitude relative to HOME | 50 m |
| Approach waypoint latitude calculated by the program | -35.35921449197041 |
| Approach waypoint longitude | 149.165237 |

The scene's panel centre is Gazebo ENU `(0, -50, 1.35)`, facing north. The
aircraft must fly south into its front, so the approach waypoint is north of
the panel. The world origin is `-35.363262, 149.165237`, elevation 584 m.
The GPS latitude follows the installed ArduPilot JSON simulator's NED-position
conversion (`AP_Common/Location::offset`), including its integer precision.
The mission builder uses its existing geodesic calculation for the 500 m leg.

Live, disarmed SITL telemetry reported HOME at 584.09 m AMSL. The target centre
is 584 + 1.35 = 585.35 m AMSL; therefore `land_alt` is 585.35 - 584.09 = 1.26 m.
Recheck this after changing the world, HOME, or aircraft spawn. These GPS values
and this altitude are simulator configuration, not a real aircraft setup.

Merlin's original spawn is preserved: `(0, 0, 0.095, 0, 0, 0)` in Gazebo.
SITL telemetry confirmed its initial heading is east (90 degrees). It is not
rotated or teleported toward the net. Consequently the forward camera does not
see the panel at spawn; an invalid detection there is expected. The original
Merlin model rests on its belly and has no validated takeoff/hand-launch model.

## Dependencies

MAVROS core and extras are installed. `mavros-extras` supplies the VFR HUD airspeed
topic consumed by the vision controller. To install/check both packages from your
own terminal (sudo needs your local password):

```bash
sudo apt-get install -y --no-install-recommends ros-humble-mavros ros-humble-mavros-extras
```

The Humble/Harmonic bridge packages are already installed under the apt name
`ros-humble-ros-gzharmonic-bridge`, exposing ROS package `ros_gz_bridge`.
GeographicLib's `egm96-5` data is already present. No Conda/pyenv changes are needed.

## Start, plan in QGC, then run the program

Terminal 1:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh doctor
bash tools/sim.sh start --no-preview
```

This opens Gazebo and QGC, and starts SITL, the camera/clock bridge, and MAVROS.
If QGC is already running, the launcher reuses that application and leaves it
open on shutdown. `simulation/mavros.launch.py` runs the MAVROS router and UAS
components in separate processes, with the localhost TCP endpoint and no extra
GCS endpoint. Plugin settings come from `simulation/mavros.yaml`. MAVROS waits
for SITL's TCP port before starting.
Keep this terminal open. Ctrl+C stops the session; from another terminal,
`bash tools/sim.sh stop` signals only the managed session. Session state and
logs are stored under `.simulation/`, which git ignores. SITL's EEPROM and
mission state persist in `.simulation/sitl/`; no existing SITL state is wiped.
The launcher does not rebuild the ArduPilot binary or change the model's tune.

In QGC, connect to the local simulated vehicle, plan your base mission and
upload it **while disarmed and outside AUTO**. Its final item should be a normal
waypoint; an existing LAND, terminal RTL, unlimited loiter or DO_JUMP may prevent
the appended landing leg from being reached and is rejected by this program.
Arrange the last part of your base mission so the aircraft can join the
southbound approach. Do not add the approach/LAND pair manually as well.

Terminal 2, after QGC has uploaded the base mission:

```bash
cd /home/abin/td/autoland1
# Read-only program preview, with no FCU parameter or mission writes:
bash tools/sim.sh preview
```

For the separate mission-preparation test, stop that preview with Ctrl+C and
explicitly allow the mission upload:

```bash
bash tools/sim.sh preview --upload-mission
```

That command enables **mission upload only**, in the managed local simulator.
Parameter writes, handoff, live actuation and partial-attitude verification
remain disabled. `simulation/fc_parameters.yaml` verifies and preserves the
simulator's native landing values instead of uploading the original illustrative
aircraft tune. Its named LAND parameters were confirmed present through a
read-only SITL MAVLink query. The MAVROS parameter-service path still needs a
live verification after MAVROS is installed.

Once the parameter profile is verified, the program automatically pulls the base
mission, appends command 16 (approach) and command 21 (LAND), uploads the full
mission, then pulls and compares the result. A list of 16 items becomes 18; QGC's
visible numbering may differ. Restarting with an identical final pair reuses it.
No mission clear or active-index change is issued. Preparation refuses an armed
FCU or AUTO mode. If preparation fails, inspect the reason, correct it, and call
`/mission_manager/prepare` again while disarmed, or restart the preview.

Generated LAND items use ArduPlane's canonical `param4=1` direction flag, so
strict readback matches the firmware's normalization of nonnegative inputs.
For this project's required LAND_TYPE=0, that deep-stall-only flag does not
change the 180-degree approach direction. Other mission fields remain checked.

Terminal 3:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh check --require-mavros
bash tools/sim.sh shell
# This shell already has the installation and isolated simulator ROS domain:
ros2 topic echo /autoland/parameter_profile --once
ros2 topic echo /autoland/landing_plan --once
ros2 service call /mission_manager/prepare std_srvs/srv/Trigger '{}'
```

Wait for a verified landing plan, then download the mission from the vehicle
in QGC and inspect the last two items. Do not overwrite the extended mission
with the old QGC plan afterward: a mission change invalidates readiness.

The operator still handles arming, launching/takeoff, and selecting AUTO. ArduPlane
executes the existing mission followed by the appended approach and LAND.
This setup does not yet enable the custom-control handoff. Supervisor enable,
handoff permission, valid live inputs, and independently verified attitude-mask
support are still required for that later test. No flag bypasses those checks.

The agreed handoff is **early AUTO to GUIDED near the configured flare
thresholds**. It does not wait for a measured internal flare phase, flare
completion, or settled sink rate. Actual GUIDED telemetry must confirm the
transition before the vision controller gets ownership.

## Connections

| Link | Endpoint |
| --- | --- |
| Gazebo ArduPilot plugin | UDP 127.0.0.1:9002 |
| SITL SERIAL0 to QGC | UDP 127.0.0.1:14550 |
| MAVROS to SITL SERIAL1 | TCP 127.0.0.1:5762 |
| Read-only diagnostic client to SITL SERIAL2 | TCP 127.0.0.1:5763 |
| ROS isolation | Domain 37, localhost only |
| Gazebo isolation | Partition `autoland_merlinv2` |

QGC normally auto-connects to UDP port 14550. If needed, enable its UDP
auto-connect or create a UDP communication link listening on port 14550.
MAVROS uses its own TCP connection, so it does not compete with QGC for a socket.
Do not run the older model launchers at the same time: their Gazebo plugin also
uses port 9002. To inspect this simulator's ROS topics, use `tools/sim.sh shell`;
an ordinary terminal may be in a different ROS domain.

## Camera and generated world

`tools/sim.sh prepare` produces `.simulation/merlin_landing.sdf` from the landing
world. It replaces the Zephyr include with Merlin's original spawn, adds the
Ogre2 Sensors system, resolves relative assets, and supplies missing vertex
normals in generated copies of the scene meshes. Without those normals, the
installed DART collision engine crashed. Original meshes remain unchanged.

The bridge publishes `/simulation/clock`, `/camera/image_raw` and `/camera/camera_info`
from Gazebo to ROS. Images are RGB8 at 1280x720 with real simulated pinhole
intrinsics (`fx=fy=762.722` pixels). The generated perception configuration
matches the observed `merlinv2/body/nose_camera` frame and the camera's body
translation `(0.451, 0, 0.008)` m, retaining the original optical-to-FLU rotation.
All application nodes use the original simulation clock via the `clock_topic`
launch argument. `tools/simulation_clock.py` copies its latest timestamp at up
to 50 Hz to `/clock`, used by all MAVROS plugins. It does not invent or scale
timestamps. This limits clock-callback load without delaying the camera/application
clock. No detector thresholds, control algorithms or freshness limits have changed.

The packaged combined `mavros_node` showed delayed state-message bursts and
heartbeat timeouts on this machine. The supported setup now uses separate
router/UAS component processes, 11 required plugins, and eight UAS worker threads.
After these changes, a 45-second connection check received 33 state messages,
with maximum observed gap 1.49 s and final age 0.53 s. Required parameter and
mission services and telemetry publishers were also found in the ROS graph.

`check --require-mavros` reports received sample count, last-message age and
largest observed gap. It fails for a stale state or any observed gap above the
application's existing 3-second limit. Before `preview` starts, missing detection,
estimate and profile messages are expected and do not make this check fail.

For a camera/preview test without MAVROS or QGC:

```bash
bash tools/sim.sh start --headless --no-qgc --without-mavros --duration 90
# In another terminal during the test:
bash tools/sim.sh check
```

## Verified and remaining work

Verified: generated SDF, Gazebo server and GUI startup, JSON physics exchange,
disarmed SITL heartbeat and HOME/GPS/attitude telemetry, QGC startup and local
UDP target, ROS clock/image/CameraInfo streams, perception output publication,
and 86 unit tests. The spawned camera faces east, so detection was correctly
invalid rather than claiming a net observation.

Pending: complete parameter-profile
verification through MAVROS; a real QGC base-mission append/readback transaction;
flight/takeoff dynamics, approach tracking, and later GUIDED handoff testing.
No mission uploads, parameter writes, arming, mode changes or live setpoints were
sent during setup. QGC emitted OpenSSL compatibility warnings and a missing
`ARMING_CHECK` parameter warning with this firmware; local telemetry was available,
but map downloads and that UI control were not validated. Successful startup
and tests do not establish flight readiness.

Official references: [ArduPilot Gazebo setup](https://ardupilot.org/dev/docs/sitl-with-gazebo.html),
[Humble/Harmonic packages](https://gazebosim.org/docs/harmonic/ros_installation/),
[Gazebo ROS bridge](https://gazebosim.org/docs/harmonic/ros2_integration/).

## Full-test telemetry adapter

`tools/simulation_velocity.py` preserves MAVROS velocity values and image/telemetry
timestamps. It republishes `velocity_local` on `/simulation/velocity_local_enu`
with the `map` label: MAVROS 2.16 converts this vector to ENU but labels it with
`tf_child_frame_id` (`base_link`), as shown in its
[local-position source](https://github.com/mavlink/mavros/blob/2.16.0/mavros/src/plugins/local_position.cpp).
Only the simulator application selects this corrected topic through the new
`velocity_topic` launch argument. Raw MAVROS/body-velocity topics are unchanged.
Unknown input frames are rejected rather than relabelled.

After FCU connection the adapter requests MAVLink message intervals for attitude
and local position at 20 Hz, global position and VFR_HUD at 10 Hz. This changes
stream rates on the MAVROS connection without writing native parameters. The
previous approximately 1 Hz stream did not satisfy the controller watchdog.
`sim.sh check --require-mavros` now also requires fresh corrected map velocity.
The existing freshness, frame, ownership and handoff limits are retained.


## Autonomous SITL experiment

`sim.sh auto-test --execute` is a separate opt-in tool which can arm only the
managed local SITL after contract, profile, mission, supervisor and launch
checks. The ordinary application never arms. This tool backs up native values,
sets the four reviewed takeoff-trigger settings, verifies
readback, selects the existing TAKEOFF, arms without force, selects AUTO and
requests one finite Gazebo launch. Its normal cleanup confirms RTL before
stopping the application; `sim.sh recover` provides an independent local-SITL
recovery client if the observer's ROS context has closed.

The recorder logs the first failure without replacing it with later plan
invalidations. A passed LAND point without vision ownership ends the test in
recovery. It does not fake target detections, relax the early-handoff conditions
or mark a capture complete. See [simulation_command.md](../simulation_command.md).

The first real flight exposed a native approach-tracking problem: at the LAND
point the aircraft was about 50 m too high. The onboard TECS log shows filtered
height demand lagging its input by tens of metres. Inspection of local
AP_TECS::_update_height_demand shows zero TECS_APPR_SMAX inherits the adaptive
sink scaler, while an explicit approach maximum bypasses that scaler. A repeat
experiment setting 5 m/s, matching the general TECS_SINK_MAX, did not improve
the approach: both runs passed LAND about 53 m above the net centre. That
experimental setting was restored to 0 and removed from the autonomous recipe.
The log lag is confirmed; its complete aerodynamic/control cause remains
unresolved. Do not treat the approach-limit hypothesis as a proven fix. See
[SIMULATION_TEST_RESULTS.md](../SIMULATION_TEST_RESULTS.md).

The simulator telemetry adapter also requests genuine HEARTBEAT messages at
10 Hz in FCU time. The previous 1 Hz default exceeded the unchanged 3 s wall
watchdog when Gazebo ran at a low real-time factor. No state messages are
synthesized and no stale timestamps are renewed.
