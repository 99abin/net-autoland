# Validation report

## Centre-ray / vertical-plane / filtered-target implementation

The runtime detector publishes a colour-region centre without requiring four
corners. The estimator intersects its calibrated ray with a surveyed vertical
plane derived from the verified landing-plan GPS/heading. It aligns GPS fixes
to image time using local motion, propagates configurable uncertainty, and
filters a stationary target in local ENU with innovation/jump rejection and a
confidence floor. Invalid/missing/outlier frames do not renew accepted image
timestamps. Plan/calibration/clock/frame resets propagate a tracking_epoch to
invalidate incompatible controller-held coordinates. Fresh estimates are
explicitly bound to the current landing-plan ID.

All **142 Python tests passed**: geometry at equal target/camera altitude,
distortion/mount/attitude transforms, grazing/backward/distant intersections,
uncertainty growth, noisy-target filtering/outlier rejection, timestamped motion
alignment, node input/freshness/plan/calibration checks, and controller track
reset/plan binding. All four ROS packages built successfully with system Python.
The new generated message fields were inspected and a real isolated ROS
publish/subscribe test produced the expected 30 m target. Fresh-session
disarmed attitude-interface verification passed.

The first flight invocation `20261117-105333-25359` refused before arming or
catapult use because the MAVROS GetParameters read timed out. Added an 8 s read
deadline and bounded preflight retries while requiring fresh disarmed MANUAL
telemetry. Thirteen relevant simulator/launch checks passed afterward. No
flight watchdog or readiness threshold was widened. The same unused simulator
session was retested successfully through takeoff and handoff. Flight
`20261117-105653-25990` recorded 20 valid filtered vertical-plane estimates,
actual GUIDED and vision ownership, 29 live attitude messages and 15 valid
held-target previews. No plane-intersection, filter or timestamp-alignment
rejection was observed in this flight; invalid detections and stream gaps still
occurred. The flight ended with the unchanged insufficient-interception-time
guard, followed by telemetry-confirmed RTL. Capture and a safe physical
go-around were not established. The maximum heartbeat gap was 0.755 s, below
the unchanged 3 s watchdog. App processes terminated cleanly; Gazebo/camera were
left visible. Evidence: `.simulation/analysis/center_ray_result.json` and
`.simulation/analysis/center_ray_handoff_timeline.json`. Geometry/filter
assumptions and limitations are documented in CENTER_RAY_ESTIMATION.md.

## Native approach target reduced to 15 m/s (2026-11-17)

Updated both native FCU profile YAMLs and added a reproducible simulator-only
trial for TECS_LAND_ARSPD=15, retaining LAND_PF_ARSPD=13.3 and ROS target=13.3.
Seven existing parameter/configuration checks passed. Managed disarmed MANUAL
checks, baseline backup, full FCU pull and readback confirmed 15.0; only the
approach parameter was written. Baseline is under
`.simulation/parameter_trials/20261117-092005-9758/`. The fresh-session disarmed
attitude contract passed. Gazebo and the camera viewer were verified visible.

Real flight `20261117-092232-10239` completed takeoff and reached native LAND.
No GUIDED, vision ownership, live attitude commands or held-target previews
occurred. The handoff trigger was true during valid previews, but raw estimate
bursts each lasted less than 0.5 s (approximately 0.46/0.28/0.31/0.35 s), ending
with "Detection stream unavailable". Preview failures also included two invalid
timesteps, one stale-aircraft-telemetry sample and one insufficient-interception-
time sample. All existing guards remain unchanged. No fault was injected.

The aircraft abruptly stopped about 0.5 m before the rigid panel and fell to
ground level, consistent with a simulated collision rather than a demonstrated
controlled capture. The last reported airspeed before stopping was about
19.7 m/s, despite the 15 m/s target. Native disarming ended the recorder without
an RTL or ROS disarm request. The maximum FCU state gap was about 1.066 s, below
the 3 s watchdog. Evidence and geometry are saved under
`.simulation/analysis/approach15_handoff_checks.json` and
`.simulation/analysis/approach15_initial_metrics.json`. The utility returned a
nonzero result because net capture was not confirmed. Gazebo and camera were
left open for inspection.

## Stall-estimate-plus-margin speed request (2026-11-17)

Configured both native preflare and custom ROS target to 13.3 m/s, using the
Merlin README's analytical stall estimate of 11.3 m/s at 5 kg plus the requested
2 m/s. AIRSPEED_MIN was not treated as stall speed or changed. Source and
installed control YAML both resolve to 13.3. The native preflare write was
guarded by managed local SITL identity and disarmed MANUAL checks, with backup
and forced FCU readback: previous 0.0, verified 13.300000190734863. Baseline:
`.simulation/parameter_trials/20261117-000645-65121/baseline.yaml`.

The native parameter utility now permits a reviewed LAND_PF_ARSPD trial, checks
the existing airspeed envelope, and permits zero for restoration. Added checks
reject values below/above that envelope, including a stricter 13.5 m/s minimum;
no minimum is silently lowered. All **127 Python tests passed**. The current
13.3 m/s configuration was then flown in real Gazebo/SITL in run
`20261117-001017-65851`. Native AUTO takeoff/landing completed and native FCU
disarming was observed; no ROS disarm command was sent. There was no GUIDED,
vision ownership, live custom output, or held-target preview. The aircraft
passed the net position at approximately 7.04 m above HOME / 5.78 m above net
centre, then stopped on the ground about 200 m beyond the net. This is a native
fallback observation, not successful net capture.

The handoff trigger was false during every valid controller preview. For
example, at about 43 m range, height was 8.08 m above net centre and sink was
2.02 m/s: neither the 4.5 m height threshold nor the approximately 5.05 m time
threshold was met. Valid estimate runs also lasted less than 0.5 wall seconds,
often ending with "Detection stream unavailable". The controlled detector-gap
observer never paused the detector because GUIDED vision ownership did not
occur, and it was stopped after flight. A native disarm after LAND ended the
autonomous recorder without requesting RTL. The front camera viewer was opened
as requested, adding a processing-load variable; no causal performance claim
is established. Evidence:
`.simulation/analysis/preflare13_handoff_checks.json`.

## Retained stationary target through detector gaps

The controller now transforms each accepted image-time net estimate into the
local map frame and retains that stationary point. During HANDOFF_PENDING or
VISION_FINAL it recomputes the body-frame target using current aircraft pose;
fresh valid observations update it. Invalid, weak, nonfinite, out-of-order or
wrong-frame observations do not replace the point or renew observation age.
Held targets cannot initiate handoff. The supervisor monitors valid fresh
controller output during active handoff/control instead of aborting solely on
raw detector gaps. The independent FCU command watchdog remains in place.

ControlSetpoint now explicitly reports target_held and target_age_s. The default
target_hold_timeout_s=0 retains the point without an age expiry for that active
plan; a positive value bounds both monotonic receipt age and image age. This
assumes a stationary net and continuous local positioning. Plan changes,
inactive states and clock rewind clear the cache. Stale aircraft telemetry,
invalid guidance and existing close-range/time-to-contact limits still fail.

All **126 Python tests passed**, including a complete handoff/hold/reacquisition
scenario followed by bad-command recovery, changing body geometry, history
eviction, cache lifetime/reset, invalid-observation rejection and fresh-only
handoff initiation. The system-Python build completed all four packages, the
generated ROSIDL message includes the new fields, and the fresh-session
disarmed SITL attitude-contract verification passed. Flight
`20261116-235627-62883` subsequently confirmed GUIDED and vision ownership and
published 31 live attitude messages. It aborted at the existing minimum
interception time guard, not because the detector was lost. No valid held-target
previews occurred, so this flight did not exercise actual detector gaps. RTL
mode was confirmed; subsequent ground-level/inverted pose telemetry does not
establish a safe physical go-around or net capture. A controlled detector-gap
repeat had not launched before the operator requested the new airspeed targets.
Timeline: `.simulation/analysis/target_hold_handoff_timeline.json`.

## Conditional handoff with native AUTO fallback

Changed the supervisor to remain in AUTO_APPROACH when the existing height/time
window opens without ready vision/control. It can request handoff later only if
all existing checks pass, including stable fresh vision, matching bounded
preview, alignment, range and progress before LAND. Missing/lost vision before
the request causes no mode request; it resets vision stability. Vision/control
loss during HANDOFF_PENDING or VISION_FINAL originally aborted to configured
recovery; the newer target-retention policy above supersedes raw detector-loss
recovery while keeping invalid-command recovery.
No actual ArduPilot internal flare-state telemetry or post-flare wait was added.

Updated the autonomous observer so passing LAND without takeover does not force
RTL. Normal observation expiry preserves uninterrupted native AUTO; faults,
interruptions and attempted handoffs retain recovery. Native disarming after
LAND is recorded separately from net capture. All **121 Python tests passed**,
including later acquisition, pre-handoff loss/stability reset, post-request loss,
post-ownership loss, and simulator fallback/recovery distinctions. These are
regression checks with transport doubles; this policy has not yet had a new
complete Gazebo flight. The recorded flight below used the earlier abort policy.
The system-Python colcon build completed all four packages, and package discovery
returned the five autoland and two perception executables.

## Waypoint height update (2026-11-16)

At the operator's request, the YAML approach is now 50 m above HOME and the
saved local SITL mission has been uploaded and read back with TAKEOFF and every
WAYPOINT at 50 m. LAND remains 1.26 m, and HOME, coordinates, other item fields
and current index are preserved. The complete original and verified missions
are saved under `.simulation/mission_backups/20261116-231053-55291/`.

Added an explicit, default-read-only `mission-altitude` simulator utility, with
managed-process identity and disarmed MANUAL guards, backup before upload and
fresh full readback. Unit checks cover HOME/LAND preservation, coordinates,
input preservation, invalid heights and unsupported frames/commands. All
**117 Python tests passed**. The fresh-session disarmed attitude contract also
passed. Historical 100 m flight results below are unchanged; the new 50 m flight
is a separate experiment.

The new flight `20261116-231145-55537` completed takeoff, flew the 50 m mission
and descended in native AUTO to approximately 10.81 m above HOME. It aborted
about 130 m before the net because vision control was not ready at the early
handoff boundary. No valid LAND-leg estimates, GUIDED, vision ownership or live
commands were observed. RTL recovery was confirmed. This was not a successful
landing or a measured net-plane crossing. The trigger occurred outside the
configured 80 m handoff range. See the current parameter-tuning report for
measurements and evidence paths; the existing early-handoff checks are intact.

## Autonomous flight attempts and remaining blocker (2026-11-16)

Implemented explicit simulator-only autonomy and executed three real Gazebo /
ArduPlane SITL flight attempts. See [SIMULATION_TEST_RESULTS.md](SIMULATION_TEST_RESULTS.md)
for measurements and the approach plot, and [simulation_command.md](simulation_command.md)
for the updated autonomous and QGC workflows.

Fixed the confirmed original failures without weakening watchdogs: request genuine
10 Hz FCU heartbeats, set/verify the reviewed native takeoff trigger values in the
explicit autonomous utility, preserve the first abort cause, avoid repeated abort
requests from invalid plans, calculate status ages in integer nanoseconds and
wait boundedly for actual fresh readiness. Added a local-SITL recovery client and
independent recovery fallback when a ROS context closes. No normal application
launch arms or enables writes; `auto-test --execute` is required for autonomy.

All three attempts armed without force, triggered the finite catapult, completed
native takeoff around 50 m, flew the original uploaded mission and reached the
appended NAV_LAND. The 100 m / 500 m geometry and original spawn were unchanged.
The largest recorded heartbeat gaps were 0.660, 0.740 and 0.734 wall seconds,
respectively, below the unchanged three-second watchdog. The original heartbeat
and profile-invalidation failure did not recur.

Full net landing remains **unresolved**. The baseline and explicit-approach-limit
trial passed LAND 52.97 m and 53.47 m above net centre, without valid net estimates
on the LAND leg. The third native airspeed/weight trial improved that to 25.70 m
and produced 18 valid estimates, but still missed the early-handoff envelope;
maximum observed airspeed was 28.23 m/s. No GUIDED or VISION ownership, live custom
commands or capture were observed in any flight. Readiness checks were retained.

The approach-limit hypothesis did not fix tracking. Experimental landing values
were backed up and restored to TECS_APPR_SMAX=0, TECS_LAND_ARSPD=-1 and
TECS_LAND_SPDWGT=-1; the takeoff fix TKOFF_THR_MINACC=15 is retained. Original
native parameter-profile/configuration files and external model/firmware sources
were not altered. Native descent/model tuning or a reviewed geometry change is
needed before actual handoff can be demonstrated. The early-handoff law was not
changed and no flare-settling wait was added.

The first recorder's ROS context closed after approximately 300 wall seconds;
RTL was confirmed through an independent client. Subsequent runs used dedicated
background processes, ended on a missed LAND point, confirmed RTL and cleaned up
the application. Raw flight/parameter/action records and native DataFlash logs
remain under `.simulation/`.

All **112 Python tests passed**, including new explicit-execution/local-session
interlocks, exact-zero timestamp age and first-abort-cause regressions. Existing
mission, vision, parameter, ownership and watchdog pipeline tests remain passing.
Shell/Python syntax and whitespace checks passed. Test infrastructure and the
native AUTO flight are exercised; successful vision handoff/net capture remain
unvalidated. This software is not flight-ready.

## Full-autolanding simulation workflow (2026-11-16)

Added explicit `sim.sh autoland`, `verify-attitude`, and
`launch --full-autoland` workflows. Ordinary preview/run defaults remain off.
The full command requires a managed local SITL, fresh disarmed/MANUAL state,
no competing application/attitude writer, and attitude evidence matching the
current simulator process and code. The launcher still never arms or selects
AUTO; the operator does that in QGC. The existing early handoff, vision law,
mission geometry and recovery limits are unchanged.

Implemented `LandingStatus.monitoring_enabled` so a disabled MISSION_READY
supervisor cannot satisfy the full launch guard. Recording now includes live
attitude commands, GUIDED/ownership observations, supervisor states and faults.
`simulation_command.md` contains the complete operator guide;
`SIMULATION_COMMANDS.md` links to it for compatibility.

Confirmed two additional integration blockers and fixed them only in simulation:
MAVROS 2.16 publishes ENU velocity_local with a base_link label, and the default
approximately 1 Hz streams do not satisfy the existing controller watchdog.
The adapter corrects only the known label, preserves vectors/timestamps, rejects
unknown frames, and requests attitude/local telemetry at 20 Hz plus global/VFR
at 10 Hz. No native parameter writes are involved. See the
[MAVROS local-position source](https://github.com/mavlink/mavros/blob/2.16.0/mavros/src/plugins/local_position.cpp)
for the ENU vector and incorrect header assignment. The main application keeps
its existing strict map-frame check.

Validation completed:

- All four ROS packages built successfully with system Python. All seven
  expected autoland/perception executables are discoverable.
- All **107 Python tests passed**, including full-mode launch interlocks,
  disabled/enabled monitoring status, stale/changed/session-specific contract
  evidence, velocity vector/timestamp preservation, and the existing early
  handoff/confirmed-ownership/watchdog recovery pipeline with ROS doubles.
- The real disarmed SITL attitude check passed multiple times: requested
  roll/pitch +6/+4 and -6/-4 degrees produced navigation demands approximately
  +5.99/+4 and -5.99/-4, then zero/zero. The test used the project's actual
  attitude conversion and MAVROS. Zero thrust only; MANUAL telemetry confirmed
  after the test. Source-handler assertions checked yaw-ignore, thrust decoding
  and non-GUIDED rejection. This is limited interface evidence, not an airborne
  response or positive-thrust test. Reports are tied to process/code hashes;
  a report from the previous session was actually rejected.
- All seven full-mode nodes started in real local SITL. Parameter profile and
  existing landing pair verified; no mission upload was needed. All three
  handoff/output/contract flags read true. Supervisor enable succeeded and
  MISSION_READY reported monitoring_enabled=true while FCU stayed disarmed
  MANUAL. Corrected velocity frame was map, observed age about 0.04 s. No live
  attitude commands were observed while disarmed.
- The full launch dry run retains disarmed/AUTO/active-TAKEOFF/native-parameter
  interlocks. Starting a competing full application was rejected. Shell/Python
  syntax, CLI option checks and whitespace checks passed.
- Fixed duplicate rclpy shutdown in perception nodes encountered during test
  cleanup. No detection or position-estimation algorithm changed.

No arming, simulated aircraft launch, mission upload or FCU parameter writes
were performed in this implementation turn. Tests requested GUIDED only while
disarmed, with zero thrust, and restored MANUAL. Test processes were stopped.

Still unvalidated: armed Merlin takeoff/dynamics, airborne positive-thrust
response, native tracking of the 100 m/500 m descent, actual camera-based
handoff and net interception. The existing controller invalidates commands below
2.5 m forward range or 0.2 s time to target and can request RTL before contact.
No close-range commitment, automatic capture detection, net-restraint physics
or impact motor-cutoff policy was added. The guide states these limits rather
than treating a compiled program or observed GUIDED state as capture success.

## Native AUTO launch-test stage (2026-11-15)

Implemented the first flight-test stage: an optional Gazebo launch plugin,
operator-driven launch guards, native AUTO telemetry recording, and the full
`NATIVE_AUTO_FLIGHT_TEST.md` command guide. Original Merlin spawn, source model
folders, early handoff logic and preview defaults are preserved. No code arms
the FCU or selects AUTO; those remain operator QGC actions.

- The Harmonic C++ plugin builds successfully. The generated launch-enabled
  world passes SDF validation. The default world has no launch plugin.
- An isolated five-kilogram box-world test passed paused-state refusal, false
  request refusal, one-shot acceptance, repeat refusal, physical acceleration,
  exact finite force expiry and no residual force. The 0.75 s impulse applied
  for 750 steps at a 1 ms physics timestep; observed speed was 18.31 m/s.
  No aircraft was loaded or armed in this mechanical test.
- A live disarmed Merlin dry run showed original settled height 0.091 m,
  MANUAL, no launch used, and valid plugin status. An explicit execute request
  was refused by the arming/AUTO/profile/mission/parameter guards. A subsequent
  plugin status read confirmed `used=false`, `active=false`, `applied_steps=0`.
- Baseline native takeoff values were read without changes: MINACC=0,
  MINSPD=0, DELAY=2, ACCEL_CNT=1. The helper requires operator review/change
  of MINACC to the SITL starting value 15 m/s² before launch. It never writes
  those settings or disables ArduPlane arming checks.
- All 104 Python tests passed, including missing/stale/future telemetry,
  verified-plan/profile leases, original heading/ground envelope, active
  TAKEOFF, disabled custom-control flags, force/time limits and native trigger
  parameter requirements. Existing mission, parameter and early-handoff tests
  still pass. Syntax and whitespace checks passed.
- Recording generates configuration snapshots/hashes, read-only native takeoff
  values, ROS JSONL telemetry and a summary of observed native flight progress.
  No automatic flight-readiness or net-capture claim is produced.

Not verified: an armed Merlin launch, native climb/turn/landing dynamics or
aircraft-specific tuning. These require the operator to arm/select AUTO in QGC.
GUIDED attitude-contract validation, live handoff, terminal commitment, capture
detection and motor cutoff remain later stages; this implementation does not
enable them. No FCU parameter writes, mission uploads, arming or mode requests
were performed during these implementation checks.

## Cache-event and GPS-boundary compatibility (2026-11-15)

- The stored LAND latitude differed from YAML by exactly the existing 2e-7 degree
  tolerance, but binary floating-point subtraction put it slightly outside the
  boundary. The comparator now allows arithmetic roundoff of 1e-12 degrees only
  at that GPS boundary; outside-tolerance changes remain rejected.
- A verified profile failed on a MAVROS parameter-deletion event. MAVROS clears
  its ROS cache during refresh and publishes deleted-parameter events. A live
  read of all 32 profile entries found no missing values or value/type mismatches.
- Cache deletions still revoke readiness immediately. Only while safely in
  preflight does the manager reread the whole verified set without writes and
  restore readiness on an exact value/type match. Missing values, changed values,
  changes during recheck and in-flight deletion remain failures.
- All 96 tests passed, including GPS boundary/restart matching and cache-event
  regressions. Live verification read the cache only; no mission or parameter
  writes were issued. The existing preview must restart to load the new Python
  code; this is not a claim that the old running node has become READY.

## Parameter-pull failure and session isolation (2026-11-15)

The failed preview reported MAVROS ParamPull success=false. Process inspection
found an older Gazebo/bridge/clock/MAVROS session still alive after its launcher
and SITL had exited. Its router/UAS components overlapped a newer session using
the same ROS names and parameter services. Only the four verified stale workspace
process groups were stopped; QGC and saved SITL state were preserved.

Startup now checks the ROS graph and refuses pre-existing MAVROS publishers or
nodes, even if their old launcher lock is gone. The launcher handles terminal
SIGHUP for cleanup, and `stop` waits for shutdown completion before returning.
Regression checks cover stale publishers, stale node names and a clean graph.
All 89 unittests passed, along with diff and syntax checks.

A clean disarmed session passed its camera/state check (maximum FCU-state gap
1.39 s) and a forced read-only parameter pull returned success=true with all
1,360 parameters. No mission upload, parameter writes, arming or flight commands
were sent. The clean simulator was left running with preview stopped.
The terminal-by-terminal command guide is `SIMULATION_COMMANDS.md`.

## LAND readback compatibility (2026-11-15)

The first real mission preparation uploaded the landing pair but failed strict
readback because LAND param4 was sent as 0 and ArduPlane returned 1. The local
`AP_Mission::mission_cmd_to_mavlink_int` source returns +1 for a nonnegative LAND
direction flag (used only for Plane deep-stall landing). The required LAND_TYPE=0
behavior is unchanged. Generated LAND items now explicitly use the canonical +1;
strict comparison still rejects altered coordinates, altitude or direction.
Two regression tests cover normalized readback/restart reuse and wrong-direction
rejection. All 88 unittests pass. No additional mission upload was performed.

## FCU-state stream repair (2026-11-15)

The step-2 failure was reproduced: camera/clock streams were present, but MAVROS
state delivery stalled and reported heartbeat timeouts. A direct localhost SITL
client received heartbeats while the combined MAVROS process delivered state
messages in bursts and consumed several CPU cores. Kernel TCP receive queues
were empty, so missing data was not explained by a backed-up TCP receive queue.

Simulator plumbing now uses the supported MAVROS router and UAS components in
separate processes, the 11 plugins required by this project, and eight UAS worker
threads. The original simulation clock is preserved for application/camera nodes;
a separate relay samples its exact timestamps at up to 50 Hz for MAVROS clocks.
Plugin reduction, clock sampling and additional workers alone did not eliminate
all stalls; the separate-process configuration passed the sustained test.

After the changes, a 45-second ROS check received 33 FCU-state messages, largest
observed gap 1.49 seconds, latest age 0.53 seconds, connected/disarmed/MANUAL.
Camera and calibration also passed. All required mission/parameter services and
state, VFR HUD, GPS, pose and velocity publishers were present. The check now
identifies missing streams and fails on any FCU gap above 3 seconds or a stale
last message. Optional pre-preview streams are identified as such. No safety
timeout, flight algorithm, aircraft spawn, or preview permission was relaxed.
A subsequent read-only forced ParamPull succeeded with all 1,360 parameters;
state messages stayed connected/disarmed/MANUAL (maximum observed gap 1.31 s).
All 86 existing unittests passed after the final changes. The corrected simulator
session was left running with the application stopped; no mission/parameter
writes or flight commands were issued.

## MAVROS startup repair (2026-11-15)

The user's startup failure was reproduced from the log: ROS argument parsing
rejected the empty CLI override `-p gcs_url:=`. Connection parameters now reside
in `simulation/mavros.yaml`, where `gcs_url: ""` is an explicitly typed empty
string. The launcher waits up to 45 seconds for SITL's SERIAL1 port before
starting MAVROS, without consuming its TCP client connection.
An additional full-session test found an already running QGC instance. The
launcher now reuses QGC rather than starting a second instance, which exits and
would otherwise trigger session shutdown. Existing QGC remains user-owned.

A bounded Gazebo/SITL/MAVROS test passed startup and the ROS stream check:
`connected=True`, `armed=False`, `mode=MANUAL`, with clock, images and calibration
present. All 86 unittests passed. A later heartbeat timeout and reconnection,
plus automatic mission/parameter pull timeouts, were observed in this test;
sustained telemetry and mission preparation remain unverified. No mission or
parameter writes, arming, mode changes or flight setpoints were requested.

## Merlin landing simulator setup (2026-11-15)

- Generated a combined world from the external landing scene and Merlin model,
  retaining Merlin's original `(0, 0, 0.095, 0, 0, 0)` spawn pose. External model
  and world sources were not edited. Added missing normals to generated copies
  of the scene OBJ meshes after reproducing a DART collision-engine crash.
- Configured `mission.yaml` for the panel south of the origin: latitude
  -35.3637111, longitude 149.165237, southbound heading 180 degrees and a 500 m
  approach. Existing mission-builder output places approach at latitude
  -35.35921449197041, longitude 149.165237, altitude 100 m above HOME
  (updated from 40 m at the user's request).
- Read-only SITL telemetry confirmed disarmed MANUAL, original latitude/longitude,
  eastward heading, and HOME altitude 584.09 m AMSL. Set the LAND altitude to
  1.26 m above HOME to target the panel centre at 585.35 m AMSL.
- Confirmed JSON physics exchange, headless Gazebo and GUI startup, QGC startup
  and its local UDP target, and live ROS clock/image/calibration streams.
  Images are 1280x720 RGB8, frame `merlinv2/body/nose_camera`, with plumb_bob
  intrinsics fx=fy=762.722. Perception outputs publish; the net is outside the
  initial east-facing camera view, so no valid detection was claimed.
- Read-only MAVLink requests confirmed all named simulation profile parameters
  are present. LAND_TYPE=0, LAND_FLARE_ALT=3, LAND_FLARE_SEC=2,
  LAND_FLARE_AIM=50, LAND_PF_ALT=10, LAND_PF_SEC=6, LAND_PF_ARSPD=0.
  The simulation profile preserves these values. No parameter write, mission
  upload, mode request, arming, or actuation command was sent.
- **86 unittests passed**, including new generated-world/spawn preservation and
  configured 500 m / 16-to-18 / duplicate-prevention checks. Generated SDF
  validation, shell syntax, and diff whitespace checks passed.
- Managed test sessions were stopped, without broad process-kill commands.

Dependency status update: both `ros-humble-mavros` and `ros-humble-mavros-extras`
are now installed. MAVROS startup and initial connection have passed. Full ROS/MAVROS
parameter verification and QGC mission append/readback have not been tested.
QGC also emitted OpenSSL and missing ARMING_CHECK UI warnings. Flight dynamics,
takeoff and GUIDED handoff remain unvalidated. See `simulation/README.md` for
the reproducible setup, explicit mission-upload command, and limitations.

## Workspace build recovery (2026-11-15)

Verified on Ubuntu 22.04 with ROS 2 Humble at `/opt/ros/humble` and
`/usr/bin/python3` 3.10.12:

- All four packages built in `build_handoff` / `install_handoff`; no output trees
  were deleted. `--cmake-clean-cache` regenerated the interface package's cache.
  A second build also passed with cvis/pyenv paths, `PYTHONHOME`, `PYTHONPATH`,
  Conda variables and stale workspace prefixes deliberately supplied to the script.
- The old cache selected `/home/abin/miniconda3/envs/cvis/bin/python3`. Original
  build logs recorded active `cvis`, nonexistent legacy `install/...` prefixes,
  and `catkin_ws`. The original generated setup chained `catkin_ws` as an underlay.
  The regenerated cache selects system Python and setup chains only Humble.
- `rosdep check --from-paths src --ignore-src --rosdistro humble` passed. Ubuntu's
  Empy 3.3.4 was already installed as `python3-empy`; no dependencies were installed
  or upgraded. Disabling Python user packages avoids an observed NumPy 2.2.6 /
  binary `cv_bridge` conflict and uses Ubuntu's NumPy 1.21.5 and OpenCV 4.5.4.
- All four installed package prefixes and all seven expected executables were
  discovered after sourcing `install_handoff/setup.bash` in the isolated shell.
- All seven executable modules, generated messages/service, and compiled interface
  type support imported against real ROS libraries. Executable shebangs use
  `/usr/bin/python3`. This caught and fixed the MAVROS message spelling `VFR_HUD`
  to Humble's `VfrHud`; test doubles now match the installed API.
- All 84 existing unittests passed with system Python, NumPy and OpenCV, including
  preview interlocks and early handoff/recovery tests. Shell syntax and
  `git diff --check` passed.
- `bash tools/run.sh --show-args` confirmed parameter writes, mission upload,
  handoff and live output still default to false. No application nodes or aircraft
  connections were started.

See README build instructions for the isolated shell and reproducible commands.
The historical report below describes earlier checks. Build, installed package
discovery, interface generation, and import checks above supersede those earlier
limitations; DDS/QoS, live MAVROS/FCU, camera and flight validation remain untested.

## Original implementation checks

Base: `origin/handoff` at `0b02a9b8da038cecaf5a02a592c27a7a17fb0e6f`.

Implemented: four ROS packages, seven executable nodes, six message types, one
ownership service, launch/configuration, and non-ROS tests.

Parameter revision retains the existing early handoff. It adds typed YAML profile
loading, native landing-parameter discovery, baseline backup, sequential writes,
forced FCU readback, profile export, readonly monitoring, and parameter readiness
interlocks. No post-flare/settled-sink-rate transition is introduced.

Checks completed in the creation environment:

- **84 unittest tests passed**, no skipped tests with OpenCV available.
- The full handoff/recovery pipeline test also passed when run independently.
- Parameter profile tests cover null preservation, invalid/unknown/duplicate keys,
  integer/float handling, native airspeed constraints, preview, rejected writes,
  clamped readback, baseline backups/export, arming mid-transaction, late RPCs,
  readonly monitoring in flight, drift/change events and readiness interlocks.
- Ruff undefined-name/import checks passed.
- Black formatting check passed.
- Python syntax and XML manifests parsed; YAML parameters matched node declarations.
- The blue/purple panel was detected in the supplied reference image. Approximate
  corners were (160,134), (653,131), (647,302), (167,317) pixels.
- Synthetic calibrated image → detection → metric body-frame pose test passed.
- Mission list 16→18, duplicate prevention, upload/readback comparison, and response/
  topic delivery ordering were exercised with transport doubles.
- Service-before-status message ordering, actual GUIDED confirmation, watchdog recovery,
  pilot override, and stationary capture event were exercised with doubles.

Test runtime: Python 3.12, NumPy, OpenCV 5.0.0. Target build recipe is ROS 2 Humble /
Ubuntu 22.04 / Python 3.10; code uses Python 3.10-compatible syntax. OpenCV APIs are
selected from the 4.x-compatible API, but the Ubuntu packaged version was not run.

Not verified here:

- colcon build, generated ROSIDL code, ROS discovery/QoS or installed entry points;
- Docker build (Docker is unavailable);
- live MAVROS/FCU mode and mission transactions;
- firmware-specific attitude mask on the user's actual ArduPlane version;
- camera calibration, real lighting/occlusions, airspeed health;
- closed-loop SITL dynamics, terminal capture/impact, real aircraft flight.

The ROS node tests use transport/message doubles. They are useful regression tests,
not a substitute for the outstanding integration and flight-dynamics checks.
