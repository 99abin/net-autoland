# Stage 1: Merlin launch and native AUTO flight in Gazebo

This is the first flight-test stage. It adds a finite simulated catapult and
records the native ArduPlane flight. **You arm and select AUTO in QGC.** The
helper never arms, changes an FCU mode, uploads parameters, or changes a mission.
GUIDED handoff and live vision control remain disabled in this stage.

## What the launch helper does

With `--launch-assist`, the generated world loads a Gazebo-only plugin. It waits
without moving Merlin. An explicit, guarded launch request applies an experimental
120 N forward force and 80 N upward force for 0.75 simulation seconds. It applies
the force at the original model origin, in its current horizontal heading.
There is no teleport, spawn rotation, persistent force, or automatic second launch.
Restart the simulation for another launch attempt.

The settings are in `simulation/launch_assist.yaml`. Python and C++ enforce hard
force/time limits. These are initial simulator values, not a validated catapult
specification or an aircraft tune. Original model/world source folders are unchanged.

## 1. Start the launch-enabled simulator — Terminal 1

If an old preview is running, stop it with Ctrl+C. Stop an old managed simulator
and wait for complete shutdown:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh stop
```

Then start the new session:

```bash
bash tools/sim.sh doctor
bash tools/sim.sh start --no-preview --launch-assist
```

The plugin builds automatically, using installed Gazebo development libraries.
Keep this terminal open. Gazebo and QGC open; Merlin retains its original
`0 0 0.095 0 0 0` spawn (east-facing, heading about 90 degrees).
Without `--launch-assist`, the normal world has no launch plugin.

Optional build/test commands, before starting a simulator:

```bash
bash tools/sim.sh build-launch-plugin
bash tools/ros_env.sh python3 tools/test_launch_assist.py
bash tools/ros_env.sh bash tools/test.sh
```

The physics test launches a separate five-kilogram box in a unique Gazebo
partition. It never loads or arms an aircraft.

## 2. Check connections and prepare your mission

Terminal 2:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh check --require-mavros
```

Wait for connected/disarmed/MANUAL and healthy camera/clock streams.

In QGC, upload the base mission while disarmed. Include a **NAV_TAKEOFF** item
before the normal flight waypoints. For the initial experiment, a 50 m takeoff
altitude and 15 degree minimum takeoff pitch are starting values for review.
Use normal waypoints after it; finish the base mission with a normal waypoint.
The landing program supplies the final approach/LAND pair. If the current
mission already has a matching pair, it can be reused.

Ensure the takeoff item is selected/current before starting AUTO. The helper
requires command 22 to be active and never changes the active mission index.
The takeoff direction follows the aircraft's original east-facing heading;
the later southbound landing approach does not change launch direction.

## 3. Verify the parameter profile and appended mission — Terminal 2

```bash
bash tools/sim.sh preview --upload-mission
```

Leave this running. In Terminal 3:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh shell
```

At the new `bash-5.1$` prompt, run each command separately:

```bash
ros2 topic echo /autoland/parameter_profile
```

Wait for `verified: true`; Ctrl+C stops the topic display. Then:

```bash
ros2 topic echo /autoland/landing_plan
```

Again wait for `verified: true`, then Ctrl+C. Download the mission from the
vehicle in QGC to confirm the 500 m southbound approach at **50 m above HOME**
and LAND at **1.26 m above HOME**. Do not overwrite it with the old base plan.
Resolve any failed readiness status while disarmed.

## 4. Dry-run the launch checks — Terminal 3

```bash
bash tools/sim.sh launch
```

This applies **no force**. It prints the native takeoff parameters, plugin status,
and every launch blocker. While disarmed, the arming/AUTO/active-TAKEOFF blockers
are expected. Resolve all other blockers before proceeding.

The helper requires fresh FCU state, pose, velocity, relative altitude, verified
profile/plan leases, matching profile identifiers, the configured landing pair,
and disabled `live_output`/`handoff_enabled` flags. It also checks the recorded
local SITL process and managed ROS domain. Ground speed must be at most 2 m/s,
height near the ground, attitude near level, and heading near the original spawn.
The plugin independently rejects paused, moving, airborne, excessive-attitude or
already-used launches.

Before arming, set/review these **simulator-only** values in QGC's Parameters
view. They make native throttle wait for the simulated launch acceleration:

| Parameter | SITL starting value | Purpose |
| --- | --- | --- |
| TKOFF_THR_MINACC | 15 | Require the launch acceleration before motor start |
| TKOFF_THR_MINSPD | 0 | No additional GPS-speed trigger for this initial test |
| TKOFF_THR_DELAY | 2 | Native motor-start delay, in tenths of a second |
| TKOFF_ACCEL_CNT | 1 | One launch acceleration event |

The observed baseline already had the last three values; MINACC was 0 and
needs review/change for this launch experiment. Rerun the dry run to confirm
the cached values match. The helper reads these parameters but never writes
them, and refuses to launch if this trigger contract is not met. No arming
check is disabled. Airframe/TECS tuning is still experimental.

## 5. Request one launch, then use QGC to arm and start AUTO

In Terminal 3:

```bash
bash tools/sim.sh launch --execute --wait 60 --record-duration 180
```

This explicitly requests one launch and waits up to 60 seconds for the guards.
**While it waits, use QGC to arm the simulated aircraft and select AUTO.**
When fresh telemetry confirms armed AUTO with NAV_TAKEOFF active and every
other check passes, the helper sends one Gazebo launch request. ArduPlane
handles throttle, climb and the original mission. The helper applies its finite
force, then records flight observations for 180 seconds. It never sends an
arming command, mode command, attitude demand, or thrust demand to MAVROS.

If the wait expires or a guard fails, no launch request is sent. Resolve the
reported cause while disarmed. An unconfirmed service reply is not permission
to keep retrying; inspect the plugin/session before another attempt.

## 6. Observe the first flight and inspect results

For the first test, look for acceleration, positive climb, controlled attitude,
and mission progress beyond NAV_TAKEOFF after reaching its configured altitude.
Stop this stage after evaluating native flight; it does not authorize a vision
net-interception attempt.

Each run writes to `.simulation/flight_runs/<timestamp>-<pid>/`:

- `context.json`: session, world metadata and configuration snapshots/hashes.
- `native_takeoff_parameters.json`: read-only native takeoff/airspeed values.
- `telemetry.jsonl`: simulation timestamps, FCU state, mission progress, pose,
  velocity, airspeed, vision estimates, ownership and preview commands.
- `launch_result.json`: plugin snapshot after an accepted request.
- `summary.json`: observed takeoff completion, native LAND activation, maximum
  height/airspeed, state gaps and unexpected GUIDED observations.

`takeoff_completion_observed` requires armed AUTO, height near the takeoff target,
and mission progress past TAKEOFF while armed AUTO. It is an observation, not
flight-dynamics certification or proof of net capture. No valid takeoff is
claimed when the required observations are missing.

To record another interval without applying force:

```bash
bash tools/sim.sh record-flight --duration 180
```

**Ctrl+C stops the observer, not the aircraft's native mission.** To end this
simulator experiment, stop the preview in Terminal 2, then:

```bash
bash tools/sim.sh stop
```

## What remains before the full net-interception test

This stage does not enable `partial_attitude_verified`, `handoff_enabled` or
`live_output`. Next stages must validate the installed firmware's roll/pitch/
thrust command contract at altitude, then the existing early AUTO-to-GUIDED
handoff, and finally terminal commitment, capture detection and motor cutoff.
The current controller's close-range/time-to-contact limits are still present.

The configured 50 m to 1.26 m descent over 500 m is approximately 5.6 degrees.
The original mission must establish that approach height before LAND. Earlier
100 m trials used approximately 11.2 degrees; they do not validate this setup.
The net is a rigid panel collision, not a validated flexible capture model.

References: [ArduPlane automatic takeoff](https://ardupilot.org/plane/docs/automatic-takeoff.html),
[Gazebo link-force API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Link.html).
