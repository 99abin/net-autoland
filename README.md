# Net autoland: AUTO approach → vision-guided final

A ROS 2 Python implementation for the `handoff` branch. ArduPlane flies its native
AUTO landing approach. A supervisor can transfer final-phase control to GUIDED
when the approach geometry, vision estimate, and controller are ready.

**Default behavior is preview-only:** no FCU parameter writes, mission upload, mode change, arming, or
attitude/throttle actuation. The system never arms the aircraft automatically.
The final controller is experimental interception guidance, not a port of TECS
and not a validated aircraft/net capture controller.

## What is included

| Package | Executables |
| --- | --- |
| `perception` | `net_det`, `net_pos` |
| `autoland` | `parameter_manager`, `mission_manager`, `landing_supervisor`, `vision_controller`, `fcu_interface` |
| `autoland_interfaces` | Generated messages and ownership service |
| `start` | `autoland.launch.py` and configuration |

MAVROS and a calibrated camera driver must run separately. The launch file does
not guess your serial device, MAVLink port, camera device, or simulator.

The blue/purple **5.5 m × 2.0 m rectangular panel** from your reference is the
visual target. The color detector uses HSV segmentation and quadrilateral shape;
no ArUco marker or downloaded model is required. The supplied reference image is
included only as a detection regression fixture under `tests/data`.

## Put these files on the branch

The source bundle contains a ready-made `src` tree and supporting files. From the
root of your existing repository:

```bash
git switch handoff
unzip /path/to/net-autoland-handoff.zip -d .
```

If you already edited any of the same files, inspect/merge them before extracting.
The bundle does not include `.git`, `build`, `install`, or `log`; your repository
and history are preserved. Existing tracked generated files from the branch are
left untouched. Builds below use separate output directories.

## Build: Ubuntu 22.04 / ROS 2 Humble example

Use the Python environment supplied by your ROS installation for runtime.

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash tools/build.sh
source install_handoff/setup.bash
```

`tools/build.sh` uses `build_handoff`, `install_handoff`, and `log_handoff` to avoid
reusing the branch's old generated files. Change the ROS distribution source path
if appropriate; Humble/Python 3.10 is the intended build target.

A Dockerfile is also supplied as a build recipe (`docker build -t net-autoland .`).
It was not built in the review environment. MAVROS itself also requires its usual
GeographicLib dataset installation and your normal FCU connection configuration.

Do not pip-upgrade system NumPy underneath an installed binary `cv_bridge`.
For tests outside ROS, use a separate virtual environment and
`requirements-test.txt`.

## Configure and verify the FCU landing parameters

The early handoff design is unchanged. There is no new post-flare settling check.
A new `parameter_manager` runs BEFORE mission preparation. Its readiness lease is
required independently by the mission manager, supervisor, and FCU writer.

Two files have different purposes:

| File | Purpose |
| --- | --- |
| `src/start/config/fc_parameters.yaml` | Desired native FCU parameter values |
| `src/start/config/parameter_manager.yaml` | ROS node timeouts, namespace and backup directory |

Review `fc_parameters.yaml`, enter your aircraft/SITL-specific values, then set
`configured: true`. Explicit numbers are desired FCU values. **`null` means preserve
the value read from the FCU**, not zero and not an unset parameter. The included
preflare airspeed is null: change `LAND_PF_ARSPD` to a validated positive speed to
enable native preflare, or preserve 0 to keep it disabled.

The manager discovers available `LAND_*` and `TECS_LAND_*` names plus relevant
flare/approach and airspeed-limit parameters on the actual FCU. Names not explicitly
listed in your YAML are read, preserved, and included in verification. You can
add another numeric native FCU parameter explicitly. Unknown names, duplicate YAML
keys, fractional values for integer FCU parameters, and invalid basic landing/
airspeed relationships are rejected before writes. It requires `LAND_TYPE=0`.
There is no universal complete aircraft tune: this preserves your existing general
flight tune and only changes explicitly requested profile values.

Startup, while connected/disarmed/outside AUTO:

1. Force a full MAVROS parameter pull from the FCU.
2. List available native parameter names and read numeric values/types.
3. Resolve null/preserved values and validate the effective profile.
4. If already matching, report READY without writes, even in preview mode.
5. If changes are needed but writes are disabled, report PREVIEW, not READY.
6. If writes are enabled, save baseline YAML first and send one changed parameter
   at a time through MAVROS's standard `SetParameters` service.
7. Force another full FCU pull and compare readback before reporting READY.

This requires MAVROS exposing `/mavros/param/pull`, `/list_parameters`,
`/get_parameters` and `/set_parameters` under its `/mavros/param` node. It does not
use the unsupported device-side atomic setter or force writes to unknown settings.
Older MAVROS builds lacking those ROS 2 services need a compatible MAVROS version;
there is no silent fallback to unchecked writes.

Observe the profile result:

```bash
ros2 topic echo /autoland/parameter_profile --qos-durability transient_local
```

A READY heartbeat contains the verified values as JSON and a profile identifier.
Mission plans are bound to that identifier. The supervisor obtains
`LAND_FLARE_ALT` and `LAND_FLARE_SEC` from these verified values rather than the
old duplicate ROS YAML settings; its existing early-handoff margins still apply.

The FCU parameter plugin's observed change events and periodic cache reads are
monitored after verification, including in flight. A mismatch, missing manager
heartbeat, or FCU disconnect blocks readiness; active custom control requests
recovery. Monitoring never writes in flight. Cache/event monitoring only sees
changes observed by MAVROS; it is not a guaranteed continuous fresh read of every
parameter over every FCU link. A new preflight sync performs a fresh full pull.

### Run with source configuration files

After building, this helper uses the editable source YAML files directly:

```bash
bash tools/run.sh
```

To apply the reviewed parameter profile and prepare the mission automatically on
startup, explicitly enable both kinds of writes:

```bash
bash tools/run.sh parameter_write_enabled:=true upload_enabled:=true
```

These are separate from `handoff_enabled` and `live_output`. Parameter writes can
persist on the FCU. Startup verification happens before mission preparation; a
failure does not proceed to mission upload or grant vision ownership. The program
does NOT physically inhibit FCU arming, so keep the FCU disarmed until preparation
and ground-station checks are complete.

Retry after correcting/reviewing the same profile file:

```bash
ros2 service call /parameter_manager/sync std_srvs/srv/Trigger '{}'
```

### Export a full editable landing profile

Once READY, export all selected native values, including discovered parameters:

```bash
ros2 service call /parameter_manager/export_profile std_srvs/srv/Trigger '{}'
```

The response gives a full YAML path. It is created with `configured: false` for
review. You can copy it over `src/start/config/fc_parameters.yaml`, edit the desired
values, set configured true, and sync again. This allows your full parameter YAML
to match the actual firmware instead of guessing optional or renamed parameters.

Baseline backups are saved under `~/.ros/autoland_parameter_backups` before the
first write. A failed or partial write leaves readiness FAILED and reports the
attempted names and backup path. Writes are not transactional and there is **no
silent automatic rollback**. Inspect the FCU and restore reviewed values explicitly
using a ground station or the baseline profile. Restoring through this manager is
still subject to its LAND_TYPE=0 and other validations. A timed-out RPC can finish
late; the manager will not start a second transaction while its future is pending.

## Configure the mission

Edit `src/start/config/mission.yaml` BEFORE starting:

- Set `configured: true` only after entering your actual simulator/site values.
- `land_lat` / `land_lon`: target coordinates, decimal degrees.
- `land_alt`: metres above the FCU HOME elevation.
- `approach_heading_deg`: direction of travel INTO the net, clockwise from north.
- `approach_distance_m`: distance behind LAND at which to place approach.
- `approach_alt`: approach waypoint altitude, also above HOME.

Heading 0° puts the approach point south of LAND. Heading 90° puts it west.
The program uses a spherical geodesic offset, not a fixed degrees-to-metres rule.

The depicted target centre is 0.35 + 2.0/2 = **1.35 m above the ground beneath the
net**. This is NOT automatically 1.35 m above FCU HOME: add the net site's ground
elevation difference when configuring `land_alt`. Coordinates must represent the
chosen net interception point.

The mission manager pulls the FCU mission, creates NAV_WAYPOINT (16) followed by
NAV_LAND (21), and performs a full upload with `start_index=0` when enabled. It
then pulls again and verifies all items. A 16-item list becomes 18 items. Ground
station numbering may differ because home/other commands count as mission items.

A matching existing final pair is reused; restarting does not append duplicates.
A different existing NAV_LAND is rejected. A terminal unlimited loiter, RTL, or
DO_JUMP is rejected because the added leg may be unreachable. Other mission
routing constraints must be checked in your ground station.

Uploads are allowed only with a fresh connected, disarmed FCU outside AUTO. Do
not use this implementation to edit an airborne mission. An interrupted upload
may leave the FCU with a partial/different mission: inspect and pull it again.
This code does not clear the mission first and does not change the current index.

## Configure camera and perception

Edit `src/start/config/perception.yaml`:

1. Set image and CameraInfo topics and their exact optical frame name.
2. Supply real pinhole camera calibration at the actual image resolution.
3. Set physical target width/height and the measured camera-to-body transform.
4. Adjust HSV thresholds to the real camera's blue panel under expected lighting.
5. Keep `target_offset_net_m: [0, 0, 0]` to aim at the rectangle centre.

The default optical→body rotation assumes a rigid, forward-facing camera:
optical x right, y down, z forward → body x forward, y left, z up (FLU). Camera
translation defaults to zero and MUST be replaced if materially offset.

Object points are ordered top-left, top-right, bottom-right, bottom-left; the net
object frame has x right, y down, z normal. Large camera roll/occlusion can make
this corner association ambiguous. The detector requires the complete target
quadrilateral, and selects the largest matching region. Another similarly colored
rectangle can be misidentified; color score is NOT a semantic probability.

`net_pos` uses planar PnP with known dimensions, checks positive depth and
reprojection error, and estimates local pixel-noise position covariance. That
covariance does not include calibration error, incorrect dimensions, or false
association. Fisheye sources need rectification/calibration in a supported pinhole
model. Do not invent a camera matrix from the reference screenshot.

The estimate retains the image timestamp. Guidance interpolates aircraft pose
history to compensate the target from image time to the current body frame. A
single rigid camera transform is supported; moving/gimballed cameras need an
appropriate timestamped transform integration.

## Run preview

Start MAVROS and your camera driver, then:

```bash
source /opt/ros/humble/setup.bash
source install_handoff/setup.bash
ros2 launch start autoland.launch.py
```

For simulator time, add `use_sim_time:=true` only when `/clock` is provided.
Publish real timestamps in the same clock domain on camera and FCU telemetry.

The default mission config is deliberately unconfigured. After editing, restart
with the edited source files (installed YAML is copied during build):

```bash
ros2 launch start autoland.launch.py \
  mission_config:=$PWD/src/start/config/mission.yaml \
  perception_config:=$PWD/src/start/config/perception.yaml \
  control_config:=$PWD/src/start/config/control.yaml \
  parameter_config:=$PWD/src/start/config/parameter_manager.yaml \
  fc_profile:=$PWD/src/start/config/fc_parameters.yaml
```

Observe outputs:

```bash
ros2 topic echo /autoland/landing_plan --qos-durability transient_local
ros2 topic echo /autoland/status --qos-durability transient_local
ros2 topic echo /autoland/net_estimate
ros2 topic echo /autoland/control_setpoint
ros2 topic echo /autoland/attitude_preview
```

`/net_det/mask` is available for an image viewer. It shows threshold segmentation.
When upload is disabled, a new landing pair is published as an unverified preview.
The supervisor will not enable from that preview. A matching pair already read
back from the FCU can be verified without another upload.

## Enable mission preparation (preflight)

To actually extend the mission, explicitly enable uploads at launch:

```bash
bash tools/run.sh parameter_write_enabled:=true upload_enabled:=true
```

Preparation starts once connected/disarmed/outside AUTO if configured. Retry:

```bash
ros2 service call /mission_manager/prepare std_srvs/srv/Trigger '{}'
```

Verify the resulting mission in the ground station, then enable monitoring while
fresh aircraft telemetry is available:

```bash
ros2 service call /landing_supervisor/enable std_srvs/srv/Trigger '{}'
```

The aircraft stays under its native controller until an explicitly enabled and
confirmed handoff. Arming, takeoff, AUTO selection and route execution remain
operator/FCU responsibilities.

## SITL handoff configuration

The switches are separate:

| Launch flag | Effect |
| --- | --- |
| `parameter_write_enabled:=true` | Allows reviewed preflight FCU parameter writes |
| `upload_enabled:=true` | Allows preflight full mission upload |
| `handoff_enabled:=true` | Allows supervisor to request vision ownership |
| `live_output:=true` | Allows FCU interface mode changes and setpoints |

There is an additional `partial_attitude_verified:=true` acknowledgement for the
ArduPlane-specific message contract below. Do not set it merely to silence an
error; first verify the actual firmware's handler in SITL.

The supervisor checks that NAV_LAND is the active mission item, along-track
progress is between halfway and LAND, track/cross-track errors are bounded,
height/sink rate satisfy the configured handoff threshold, and fresh stable
vision plus bounded preview commands exist. It requests ownership, and the FCU
interface requests GUIDED. Only FCU telemetry confirming GUIDED grants ownership;
`SetMode.mode_sent` alone is not confirmation.

When live handoff is enabled, entering the handoff window without stable vision,
alignment and bounded control targets requests recovery instead of continuing
blindly toward the net. Vision must be acquired and stable BEFORE that boundary.
The trigger uses a margin before the nominal flare threshold. It is a ROS
supervisor condition, NOT telemetry of ArduPilot's internal flare stage. Its
height/sink estimates need not match native rangefinder/terrain-corrected flare.
Configure the margins against SITL logs, camera range, and mode-change latency.
The native approach controller is not reimplemented in Python.

An explicit simulator-only launch after verification is:

```bash
bash tools/run.sh \
  parameter_write_enabled:=true upload_enabled:=true handoff_enabled:=true live_output:=true \
  partial_attitude_verified:=true
```

### ArduPlane attitude target contract

The interface publishes to `/mavros/setpoint_raw/attitude`, after confirming
GUIDED and receiving fresh authorized targets. Configure MAVROS's `setpoint_raw`
plugin with `thrust_scaling: 1.0` (its usual fully qualified node name is
`/mavros/setpoint_raw`; inspect your installation). NaN scaling can reject nonzero
thrust targets in MAVROS. Generic position targets do not implement this approach.

Current reviewed ArduPlane source interprets partial quaternion selection using
axis bits. This implementation uses mask **132** (`IGNORE_ATTITUDE |
IGNORE_YAW_RATE`) deliberately, with quaternion roll/pitch and included thrust.
It selects roll/pitch without forcing yaw/rudder. This is an ArduPlane-specific
interpretation, NOT the standard generic MAVLink meaning of IGNORE_ATTITUDE, and
must be checked in your exact firmware version. Mask 7 would activate the
all-angle path, including an unwanted external yaw/rudder override in the reviewed
handler. Do not apply this node unchanged to Copter or PX4.

Relevant upstream code:

- [ArduPlane attitude-target handler](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/GCS_MAVLink_Plane.cpp): `handle_set_attitude_target()`.
- [ArduPlane GUIDED mode](https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_guided.cpp) for timeout/inner-controller behavior.
- [MAVROS raw setpoints](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/setpoint_raw.cpp) for quaternion transforms and thrust scaling.

Roll in ControlSetpoint is right-bank-positive; pitch is nose-up-positive; both
are radians. Yaw is ROS ENU and retained only to construct the correctly transformed
quaternion. Thrust is normalized [0,1]. State/plan IDs, timestamp/receipt watchdogs,
command limits, first-target pose continuity, and subsequent step limits are
checked by the single FCU writer independently of the guidance node.

## Abort, override and completion

- Freshness uses both measurement timestamps where available and monotonic receipt
  time. Float64 relative altitude and some status inputs have receipt checks only.
- Loss of vision/commands/supervisor ownership requests configured recovery while
  owned in AUTO/GUIDED. The default recovery is RTL. Success is confirmed through
  actual mode telemetry. Link loss cannot guarantee that an RTL request arrives;
  configure/test onboard failsafes independently.
- A pilot-selected mode outside AUTO/GUIDED releases custom ownership; the node
  does not force GUIDED again.
- There is NO validated close-range commit/impact policy. At low range or loss of
  operating airspeed, guidance becomes invalid and recovery is requested. Whether
  any go-around is physically possible there is aircraft/site dependent. This is
  one reason this is not ready for an unattended real net landing.

Operator abort:

```bash
ros2 service call /landing_supervisor/abort std_srvs/srv/Trigger '{}'
```

Capture is not inferred from the flare stage or a missing image. After physically
confirming capture and fresh ground speed below 1 m/s, an operator/test harness can:

```bash
ros2 service call /landing_supervisor/confirm_capture std_srvs/srv/Trigger '{}'
```

COMPLETE keeps the last attitude target with zero thrust while still owned in
GUIDED. The operator must disarm/change mode; the software never arms or disarms.
This explicit event is not an automatic contact/capture detector.

Reset requires a fresh disarmed FCU:

```bash
ros2 service call /landing_supervisor/reset std_srvs/srv/Trigger '{}'
```

## Guidance and validation limits

`vision_guidance.py` predicts a constant-velocity crossing using target range and
body velocity, corrects lateral crossing error with a bank demand, corrects
vertical crossing error with pitch, and uses airspeed PI for throttle. It includes
attitude/thrust limits, slew limits, and integral anti-windup. These calculations
are experimental, not the original TECS controller. Aerodynamics, net deformation,
wind, stall margin under bank, changing angle of attack, and camera/detector error
need aircraft-specific validation. A positive VFR_HUD airspeed alone does not prove
sensor health. The throttle policy does not implement a validated motor-cutoff
sequence for net impact.

Native AUTO landing remains in charge until the handoff. Once GUIDED is confirmed,
native AUTO flare is no longer running the final controller; this is a replacement
terminal phase, not an in-firmware vision correction.

## Tests

```bash
# Separate non-ROS test environment:
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-test.txt
bash tools/test.sh
```

Tests cover mission geometry/duplicates/readback, handshake sequencing, command
watchdogs, pilot override, capture event, synthetic calibrated pose recovery,
detection of your actual reference image, image-to-pose processing, timestamped
motion compensation, and a complete AUTO→GUIDED→vision-loss recovery scenario.
ROS transports in node tests are doubles. A passing suite does NOT establish DDS,
colcon/IDL generation, MAVROS interoperability, or flight stability.

The creation environment has no ROS installation, Docker daemon, FCU, camera
calibration or simulator. Those integration checks and aircraft tuning remain
outstanding. Start with preview, then SITL, and verify the contracts before any
hardware actuation.

See `VALIDATION.md` for the exact test coverage and outstanding checks.
