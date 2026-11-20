# Validation report

## Native landing airspeed-priority repeat (2026-11-18)

Changed the simulator-only profile from `TECS_LAND_SPDWGT=-1` to `2.0` using a
backed-up, disarmed MANUAL parameter transaction and forced-pull readback. Only
that native value was written; the 15 / 13.3 m/s targets, 50 m / 500 m approach,
PID gains, ROS controller and handoff guards were preserved. Added bounded
trial-loader coverage; all **152 Python tests passed**. Details and evidence
are in [ARDUPILOT_PARAMETER_TUNING.md](ARDUPILOT_PARAMETER_TUNING.md).

An agent-service restart interrupted the initial run shortly after takeoff.
The detached repeat, `20261118-153218-8542`, reached the net in AUTO at 15.00 m/s
but 10.61 m above its centre. No GUIDED/live ROS control or net contact occurred.
Native auto-disarm was recorded after the aircraft stopped on the ground about
103 m beyond the net. Gazebo GUI exit then shut down SITL; the observer reported
simulator-disconnected and did not receive the final disarmed ROS state. The
speed comparison is recorded; net capture and a validated landing are not
established. Test ROS nodes were cleaned up after simulator shutdown.

## Terminal commitment and contact experiment (2026-11-17)

Implemented the explicit simulator terminal phase described in
[TERMINAL_GUIDANCE.md](TERMINAL_GUIDANCE.md): predicted-crossing/uncertainty
commitment gates, a regularized guidance horizon, bounded terminal bank, thrust
ramp toward zero, phase-aware fault/missed-plane holds and blue-panel contact
sensing. Normal launch flags remain false. The test runner preserves terminal
hold rather than undoing it with cleanup RTL. No source aircraft/world file,
original spawn, mission geometry or early-handoff threshold was changed.

All **151 Python tests passed**. Added checks cover rejected/stable commitment,
the decision boundary, finite bounded close-range commands, throttle authority,
immediate zero-thrust terminal watchdog response, pilot override, collision
identity and rejecting ground-level capture confirmation. All four ROS packages
built with system Python, and all seven expected executables were discovered.
The disarmed SITL attitude contract passed. A temporary non-aircraft probe
verified the real Gazebo → ROS contact bridge, then was removed before takeoff.
Harmonic's scoped contact topic was required; its short `<topic>` setting did
not supply data on the originally configured bridge topic.

Actual flight `20261117-220700-22801` completed takeoff and the saved mission,
entered GUIDED with vision ownership, committed at simulator time 196.703 s,
and observed blue-panel/fuselage contact at 198.434 s. The supervisor reported
NET_CONTACT at 198.453 s. The recorder saw 38 valid estimates, 83 live attitude
messages, 8 held-target previews, 3 valid close-range terminal previews and 69
zero-thrust messages. Its last close-range preview was valid at 1.553 m forward
range and 0.0794 s time to target. No ABORT, TERMINAL_FAULT, MISSED_NET or RTL
was observed. Largest recorded FCU-state gap was 0.800 s, below the unchanged
3 s watchdog. The runner preserved the running terminal hold and returned a
nonzero result because capture was not confirmed.

Physical result: the fuselage contacted the rigid panel near x=-0.006 m,
y=-49.985 m, z=1.760 m, then fell inverted to ground level (final body z=0.09 m).
No net restraint or successful capture was demonstrated. Last preimpact airspeed
was 19.56 m/s despite the configured 15 m/s approach and 13.3 m/s preflare/custom
targets; deceleration remains unresolved. The FCU DataFlash log confirms GUIDED
without a subsequent RTL and both motor channels (3 and 7) at their 1100 PWM
minimum after the zero-thrust demand. Zero thrust is not instantaneous propeller
stoppage or a validated impact policy.

Evidence: `.simulation/flight_runs/20261117-220700-22801/` (telemetry, contacts,
configuration, actions and summary), `.simulation/contact_bridge_check.json`,
`.simulation/analysis/terminal_result.json`,
`.simulation/analysis/terminal_motor_outputs.json`, and
`.simulation/sitl/logs/00000014.BIN`. Gazebo, front camera and the armed simulated
GUIDED zero-thrust hold were left running for inspection. No automatic disarm
or operator capture confirmation was issued. These results do not establish
flight readiness or a safe real-aircraft recovery policy.

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

## 2026-11-19: four-item airborne preparation, headless integration

Implemented per-leg wind-projected original-route timing through END, verified
FCU AIRSPEED_CRUISE with explicit ISA EAS-to-TAS conversion, tangent-circle
Adjustment/Alignment placement and four independently read-back partial appends.
Removed fixed planning/deadline indices and the mandatory staging/corridor
position gate. No measured-alignment gate or flight-control retune was added.

- 181 Python regression checks passed; all four ROS packages built with system
  Python. All seven expected autoland/perception executables were discoverable.
- Headless Gazebo camera, clock and MAVROS streams passed; the disarmed attitude
  contract passed. No GUI, QGC or camera-viewer process was launched.
- First run `20261119-092426-13803`: all four appends passed, then a false
  profile-change invalidation caused ABORT and confirmed RTL. Recorded values
  and profile ID remained unchanged. Added regression-tested handling for an
  identical verified heartbeat arriving up to 50 ms ahead of its local clock:
  retain the preceding strictly fresh lease without renewing it. Changed,
  invalid, expired and future-dated profiles remain unacceptable. Cross-topic
  clock ordering is a plausible explanation, not directly measured per-node.
- Repeat `20261119-111555-17068`: original six-item prefix remained unchanged;
  only new items 6,7,8,9 were transmitted by companion 1.192, commands
  16,16,16,21. Four readbacks passed. Calm wind predicted APP distance 509.17 m,
  alignment setback 228.83 m and 63.13 s remaining original-route budget before
  the last append (minimum 30 s). Native TAKEOFF and original waypoints completed.
- **Full landing failed:** while AUTO targeted item 6 (Adjustment), altitude
  dropped from 51.85 m at sim time 134.792 s to 0.68 m at 143.074 s. Roll reached
  approximately 139.5 degrees at 140.973 s. There was no GUIDED mode, vision
  ownership or live ROS attitude output, and no LAND execution or net capture.
  This identifies a native AUTO turn/airframe stability failure before the
  landing approach; it does not establish whether tuning, dynamics or another
  native flight constraint caused it. Ordinary L1 waypoints do not enforce the
  reference circle, so geometric feasibility alone is insufficient validation.
- The repeat was explicitly interrupted after the ground-level failure; its
  harness reports INTERRUPTED and confirmed RTL telemetry. That mode confirmation
  does not imply recovery from the impact. The managed simulator was then stopped.

Artifacts: `.simulation/flight_runs/20261119-111555-17068/summary.json`,
`adaptive_analysis.json`, telemetry, context snapshots and application log.
Ground restoration preserved backups under `.simulation/mission_backups/` and
removed only the previously generated staging fixture after matching its saved
manifest. Physical capture, wind performance and native turn stability remain
unvalidated; no flight-readiness claim is made.

## 2026-11-19: native AUTOTUNE and same-route landing retry

`tools/simulation_autotune.py --execute` implements an explicitly authorized,
managed-local-SITL-only trial. It backs up rate gains/filters/time constants,
verifies configured GCS identity and received RC input, uses native TAKEOFF and
finite launch assist, then excites roll/pitch in AUTOTUNE. Envelope violations
or stale telemetry stop excitation and request RTL. Reference exports and
procedure are in `AUTOTUNE.md`.

Several preliminary attempts exposed an empty parameter cache, unsupported
MAVROS TAKEOFF mode name, an early takeoff turn, ignored RC overrides from the
companion identity, and insufficient altitude for completion. These were not
reported as successful tunes. The corrected trial uses a forced parameter pull,
native mode 13 with heartbeat confirmation, the configured GCS identity with
RC reception confirmation, and a long straight climb to 600 m.

Completed run: `.simulation/flight_runs/20261119-115827-25668/`.
Firmware reported `Roll: Finished` and `Pitch: Finished`; exit to RTL was
confirmed. Persisted gains were checked after restart and a forced FCU pull.
The original takeoff settings were restored before the landing retry.

| Gain | Original | Learned |
|---|---:|---:|
| RLL_RATE_P | 0.08 | 0.3256968 |
| RLL_RATE_I | 0.15 | 0.2109115 |
| RLL_RATE_D | 0 | 0.01373462 |
| RLL_RATE_FF | 0.345 | 0.2109115 |
| PTCH_RATE_P | 0.08 | 0.6048656 |
| PTCH_RATE_I | 0.60 | 0.6599450 |
| PTCH_RATE_D | 0 | 0.02321999 |
| PTCH_RATE_FF | 0.60 | 0.6599450 |

AUTOTUNE also changed filters and attitude time constants (roll 0.8 s, pitch
1.2 s). It was not an experiment varying just one PID coefficient.
All gains/filters are exported in `simulation/autotune_gains.yaml`; the initial
reference set is in `simulation/autotune_original_gains.yaml`. These remain
simulator-specific candidates, not mathematically optimal or flight-proven.

Same-route retry: `.simulation/flight_runs/20261119-120718-27384/`.
The same six-item original mission and four appended items were verified.
The aircraft passed the previously failing AUTO turn, visited indices 1 through
9, and entered native LAND. The earlier inversion/ground-impact failure did not
recur before the handoff. This supports control tuning as a contributor to that
failure, but a single trial and combined gain/filter changes do not prove sole
causation.

The full landing still **failed**: supervisor entered HANDOFF_PENDING at sim
255.001 s, when the controller preview became invalid with `Airspeed outside
operating envelope`. Recorded airspeed was about 8.751 m/s, below the configured
10 m/s minimum, at approximately 3.12 m above HOME. At 255.052 s the supervisor
aborted with `Control target unavailable (including retained net target)`.
Valid visual estimates were present immediately before this event; the specific
observed blocker was controller airspeed readiness, not proven detection loss.
No GUIDED telemetry, vision ownership, live ROS attitude messages or net capture
were observed. RTL was confirmed; this does not establish a safe terminal
recovery from low altitude. The simulator was stopped after recording results.

183 regression tests passed, including RC frame identity/order/checksum and
release validation. Landing behaviour and minimum-airspeed checks were not
changed to force this trial through. Native speed/descent behaviour near
handoff remains to be diagnosed. Native AUTOTUNE does not tune TECS.

## 2026-11-19: zero ROS minimum airspeed, Gazebo/QGC experiment

Requested experiment set only the ROS vision controller's minimum airspeed to
0 m/s. The normal `src/start/config/control.yaml` retains 10 m/s. An explicit
`simulation/settings.yaml` control_config points to
`simulation/control_zero_minimum.yaml` for full simulated autoland launches.
The guidance validator now allows an explicitly configured zero floor, while
rejecting negative/nonfinite limits and limits at/above the target. ArduPilot
AIRSPEED_MIN, learned gains and other control settings were unchanged.
Runtime ROS parameter readback confirmed minimum_airspeed_mps = 0.0.

All four ROS packages built and 185 regression checks passed. Gazebo and QGC
were launched. Run `.simulation/flight_runs/20261119-183011-8895/` verified all
four airborne appends and flew original/appended mission indices 1 through 9.
It reached HANDOFF_PENDING near 4.9 m above HOME but aborted with
`No matching fresh bounded preview command`, returned by the independent FCU
interface. Recorded airspeed around that event was approximately 11 m/s, so
this was not the previous below-10-m/s rejection. Valid preview messages were
recorded nearby; the recorder does not establish the exact command/clock state
inside the FCU interface at its ownership callback. The precise readiness
failure therefore remains to be traced, rather than attributed to one cause.

RTL was confirmed. No GUIDED telemetry, vision ownership, live ROS attitude
messages or net contact were observed. Removing the airspeed floor did not
produce a successful net landing. The application exited after the test;
Gazebo/QGC and native SITL recovery were left running for the user to inspect.

Set `control_config: null` in simulation/settings.yaml to return future full
simulator launches to normal control configuration. This is a simulator-only
experiment; normal tools/run.sh remains unchanged.

## 2026-11-19: headless stall-onset and throttle characterization

Two characterization flights completed with confirmed RTL and no GUI/QGC/camera
windows. No landing app, mission edit or PID tuning occurred. Roll/pitch rate
gains matched before/after. Temporary takeoff settings were restored and verified
while disarmed; the final simulator session was stopped.

Model-angle-based stall-onset proxy observations: 13.37 and 13.56 m/s, near level
and wings-level before subsequent significant height loss. This supports an
approximate 13.4–13.6 m/s region in this SITL configuration, not a certified or
real-aircraft stall speed. Main-wing LiftDrag alpha_stall is 0.16 rad with
alpha offset 0.065 rad; the estimate neglects wing pressure-centre angular
velocity and telemetry alignment uncertainty.

Refined accepted near-level points: 65% throttle → 14.79 m/s, 70% → 16.33 m/s,
75% → 17.21 m/s, 80% → 20.63 m/s. The 62% segment was marginal/inconsistent
between runs and is not included in the reliable refined map. Descending and
unsettled points were rejected. These are measured mean segments, not an exact
universal mapping or a choice of THR_MIN.

See FLIGHT_PERFORMANCE.md. Run IDs: 20261119-184815-11677 and
20261119-185506-12964. CSV, JSON and PNG exports are under
`.simulation/analysis/characterization_20261119/`. All 187 regression tests passed.

## 2026-11-19: autoland_1.plan and 15 m/s native approach/preflare

Requested original plan `/home/abin/td/waypoints/autoland_1.plan` was imported
while disarmed MANUAL, backed up and strictly verified. Actual FCU HOME and the
original spawn were preserved; plannedHomePosition is not an instruction to
relocate the aircraft. The plan contains one TAKEOFF and three waypoints at
50 m. Ground import selects TAKEOFF item 1, so it intentionally does not
preserve the active index of the replaced mission. The external plan is unedited.

Added `sim.sh import-plan --file PATH --apply` for restricted SimpleItem
WAYPOINT/TAKEOFF import. It normalizes numeric fields to ROS float types and
QGC null waypoint yaw to ArduPlane's canonical stored zero. Complex items,
unsupported frames/commands, fence/rally contents and invalid coordinates are
rejected. Mission comparison tolerances are unchanged. Positionless TAKEOFF
0,0 is skipped in original-route time prediction; it must not create a forecast
leg to 0°,0°. Added regression coverage for this and the import.

Native targets were applied with backup/readback: TECS_LAND_ARSPD=15 and
LAND_PF_ARSPD=15. `simulation/fc_parameters.yaml` records those values. The
provisional TAS profile in mission.yaml is now [15.45,15.45,15.45], giving a
515 m calm-air approach for 33.333 s. Custom ROS target remains 13.3 m/s and
its active experimental minimum remains 0 m/s. THR_MIN, LAND_OPTIONS, PID/FF,
TECS landing weighting, landing reference and handoff conditions were unchanged.

Run `.simulation/flight_runs/20261119-191511-17289/` with Gazebo and QGC open:

- Four partial writes/readbacks succeeded at indices 5,6,7,8; original five-item
  HOME/base prefix unchanged. Alignment setback 205.79 m, approach 515 m.
- HANDOFF_PENDING at sim 299.457 s, recorded airspeed approximately 13.68 m/s.
- VISION_FINAL at 299.609 s: actual GUIDED and vision ownership confirmed.
- 45 live ROS attitude/thrust messages recorded; no net contact or terminal
  commitment observed.
- ABORT at 301.752 s: `Terminal commitment rejected before decision boundary`.
  The latest valid preview had TTC 0.920 s, vertical crossing error 1.706 m,
  lateral error 0.096 m and position standard deviations 0.75 m. Terminal-ready
  was false. This is a later interception/commit-readiness failure, not a refusal
  to enter GUIDED. Do not interpret a successful handoff as successful capture.
- RTL telemetry confirmed; landing application cleaned up. GUI/QGC were left
  running as requested. Mode confirmation alone does not establish safe recovery.

Source plan copy, telemetry, context snapshots, adaptive_analysis.json and
summary.json are in the run folder. Native target baseline is under
`.simulation/parameter_trials/20261119-191353-17054/`; mission backups are under
`.simulation/mission_backups/`. Four ROS packages built; 189 tests passed.
This trial changed both the base route and preflare speed, so improvement in
handoff cannot be attributed to either change in isolation.

## 2026-11-19: unchanged new-plan GUI repeat

Run `.simulation/flight_runs/20261119-195753-24562/` repeated autoland_1.plan
with Gazebo and QGC open, native approach/preflare 15 m/s, custom target
13.3 m/s and experimental ROS minimum 0. No guidance gains or limits changed.
Base import/readback, all four appends and the disarmed contract check passed.

GUIDED and vision ownership were observed, with 50 live messages. Net capture
was not observed. The test aborted with `Terminal commitment rejected before
decision boundary`; RTL telemetry was confirmed. The last valid GUIDED preview
had TTC 0.939 s and body-up crossing error +1.462 m. No terminal commitment.

Recorded response comparison (`guidance_response.csv` and PNG in run folder):
first valid GUIDED sample requested pitch −0.95°, with measured native pitch
approximately −1.09°, vertical velocity −1.32 m/s, height above configured net
centre 4.33 m. Last valid GUIDED sample requested +2.27°, while measured native
pitch was approximately −2.10°, vertical velocity −1.69 m/s, height above net
centre 0.11 m and airspeed 14.31 m/s. GUIDED telemetry spanned about 2.52 s.
These asynchronous telemetry samples show a demand/response discrepancy; they
do not isolate whether controller dynamics, command acceptance, telemetry timing,
or airframe response caused it. No PID change is justified from this comparison
alone. Actual native pitch here is negative ROS FLU Euler pitch.

The application cleaned up after confirmed recovery. Gazebo and QGC were left
running as requested. No source/control change was made for the repeat.

### Headless vertical gain trial

Run `20261119-201116-26874` used simulator-only `vertical_gain: 1.4`, up from
0.7, and the same base mission and 15 m/s native approach/preflare targets.
All four appends, the disarmed attitude check and 189 tests passed. GUIDED
ownership was confirmed and 51 live attitude messages were recorded.

The last valid GUIDED preview predicted a +1.17 m body-up error at 0.90 s
remaining. It requested +6.17° pitch, while measured native pitch was −1.16°
and descent was 1.50 m/s. In the preceding 0.7-gain run, the corresponding
samples were +1.46 m at 0.94 s, +2.27° requested, −2.10° measured, and 1.69 m/s
descent. The two runs used different GUI load, and the samples were not taken
at identical flight states, so the comparison does not isolate gain effects.

The supervisor rejected terminal commitment at simulation time 216.356 s and
RTL was confirmed. The independent Gazebo recorder logged fuselage-to-panel
contact at 217.178 s, near the lower panel edge, after the abort. This was a
collision after the abort, with no confirmed capture. See `guidance_response.csv`,
`guidance_response.png`, `contacts.jsonl` and `summary.json` in the run folder.

### Consolidated diagnostics from the existing gain trial

The native DataFlash file 00000032.BIN was matched to run
20261119-201116-26874 using its GUIDED/RTL transitions and the simulator clock.
A preserved copy, SHA-256 manifest and native CSV exports are stored in that
run's diagnostics folder. They include 5089 ATT samples, 5089 PIDP samples,
servo PWM, normalized elevator, energy/navigation control, IMU/EKF, simulated
state and 1386 parameter records. pitch_pipeline.csv joins ROS requests with
preceding native samples within 0.1 s and records each sample's age. Raw ROS
telemetry and native binary are retained for independent timing analysis.
The existing native log already recorded the required pitch-control layers,
so another flight was unnecessary for collecting them. Raw camera images were
not recorded in that completed flight.

Future full auto-test runs preserve changed native logs automatically alongside
ROS recordings. Raw MAVLink capture includes attitude/servo/nav/RC messages,
SET_ATTITUDE_TARGET and PID_TUNING when the firmware emits them. CameraInfo is
also recorded. A logging regression test verifies changed-log selection, byte
preservation and SHA-256 generation. 190 tests passed before the CameraInfo
subscription addition. No control parameters changed during this collection.

### Comprehensive headless diagnostic repeat

Run `20261119-212129-35010` exercised the complete capture path. GUIDED and
vision ownership were confirmed, with 51 live messages. Terminal commitment
was rejected at roughly 0.91 s remaining and +1.16 m predicted body-up error.
RTL telemetry was confirmed. The headless simulator was stopped and native log
00000033.BIN finalized, preserved and decoded. The run folder includes all ROS
records, native CSVs, raw binary, pitch_pipeline.csv/png and FINDINGS.md.

At approximately 227.8 s, ROS requested +5.56° and native ATT recorded demand
+5.30° with measured pitch −1.17°. Native PIDP target was +2.51°/s and actual
+2.13°/s. Earlier measured pitch rate was still about −3°/s after takeover.
Servo outputs near 1599 us were inside 1100–1900 us limits. Pitch TCONST was
1.2 s and acceleration/input shaping was enabled. Thus transport/sign were
confirmed, and the remaining angle discrepancy lies downstream of the accepted
attitude demand. Shaping, rate dynamics and aircraft response contribute to
observed delay. No single gain was proven to be the sole cause.

Automatic end-of-run native snapshot preservation worked. The final stopped
native log was then copied for complete offline analysis. Raw camera images
were not recorded. No control parameter changes were made for this repeat.

### Requested 0.2-second pitch time constant trial

Run `20261119-223232-44789` applied PTCH2SRV_TCONST=0.2 with backed-up/read-back
native parameters. The recorded previous value was 1.2000000477 s. Parameter
metadata documents 0.4–1.0 s; the requested value is below that range. Installed
runtime code accepts values above its 0.05 s floor. The simulator-only native
trial allowlist now accepts 0.2–1.2 s, including restoration of the original
AUTOTUNE result. The simulator profile records the new value. No PID gains
were changed. ROS vertical_gain remained 1.4, custom target 13.3 m/s and its
experimental minimum 0. Native approach/preflare targets remained 15 m/s.

Gazebo was opened. Original mission import and all four airborne appends passed.
GUIDED and ownership confirmed at 209.263 s. TERMINAL_COMMITTED at 210.951 s
with predicted body-up error +0.115 m and TTC 1.836 s. NET_CONTACT at 212.803 s.
No supervisor abort/fault was recorded. There were 113 live attitude messages
and 68 zero-thrust messages. The application retained the terminal zero-thrust
hold; capture remains unconfirmed and no automatic disarm was performed.

Native log 00000034.BIN confirms PTCH2SRV_TCONST=0.200000003. Near 210.907 s,
ROS requested +1.26°, native demand was +1.21° and measured pitch +1.02°.
Compared with the preceding trial's large angle discrepancy, this demonstrates
better tracking at this point in this run. Because changing the time constant
also changes AUTO approach behaviour, the comparison does not isolate only
post-handoff dynamics.

Independent Gazebo contacts began at 212.776 s between the fuselage and blue
panel, with sampled contact positions around z=1.21 m, versus panel centre
z=1.35 m. This is simulator collision evidence near the centre, not confirmation
of physical retention in a flexible net. The model uses a rigid panel. No RTL
was requested after terminal commitment/contact. Gazebo and the terminal hold
were left running for the user.

The complete native snapshot and CSVs, ROS telemetry, contacts, configuration
snapshots and pitch_pipeline.csv/png are in the run folder. All 191 tests passed.
Baseline: `.simulation/parameter_trials/20261119-223158-44652/baseline.yaml`.
## Headless wind handoff diagnostic repeat — 20 November 2026

Run `.simulation/flight_runs/20261120-170434-7199` used the updated two-item
QGC base plan, 4 m/s northward wind, and existing tuning/control overrides.
All four airborne appends were verified and the original mission prefix was
unchanged. Planned approach distance was 381.67 m; simulator-only remaining-route
preparation minimum was 20 s (normal source configuration remains 30 s).

The supervisor entered HANDOFF_PENDING at simulation time 180.904 s, confirmed
VISION_FINAL at 181.003 s, committed at 183.655 s and reported NET_CONTACT at
184.800 s. GUIDED was observed, with 118 live attitude messages and no recorded
supervisor faults. Independent contact messages first identified
`merlinv2::body::fuselage_collision` against the blue panel at 184.782 s, with
contact points about 1.20–1.38 m above world ground. The latest airspeed and
groundspeed samples before contact were 15.11 and 11.11 m/s. Zero thrust was
commanded and the application retained terminal hold. Capture/retention is
unconfirmed; no operator-confirmed COMPLETE was claimed.

Only diagnostic logging and its test were added to the flight application for
this repeat. The previous preview-command rejection did not recur, so a
clock/order timing cause remains a hypothesis. Rejection logs now include the
local command's stamp/receipt ages, validity, frame, plan ID, reason and bounds
check. All four ROS packages built and 194 Python tests passed using
`PYTHONPATH=tools /usr/bin/python3 -m unittest discover -s tests -q`.

An earlier attempt, `20261120-165943-4948`, was interrupted by the tool server
restart before handoff. Its partial recording and interruption note are retained;
it is not counted as a landing outcome. The completed repeat used independently
running simulator/recorder processes. Native log 00000039.BIN and the ROS/contact
records are preserved with the completed run.

## Tailwind repeat — 20 November 2026

Run `20261120-171516-9420` changed only the configured wind from northward
4 m/s to southward 4 m/s, retaining the headless setup, updated base plan,
50 m waypoint heights, native tuning and control/handoff settings. All four
appends passed, the original mission prefix stayed unchanged, and Approach
was placed 648.33 m from LAND (headwind comparison: 381.67 m).

The aircraft reached AUTO LAND and HANDOFF_PENDING at simulation time 244.704 s,
then ABORT at 244.855 s. The diagnostic in `application.log` records a valid,
bounded preview in `map` with matching plan ID. Its ROS stamp age was 0.054 s,
but local wall-clock receipt age was 0.300481315 s, above the 0.2 s timeout.
This identifies the rejecting wall-clock freshness check for this attempt;
it does not establish the cause of the delivery/processing pause or prove the
same cause for the earlier uninstrumented failure. No future stamp was present.

GUIDED, vision ownership, live ROS attitude messages and net contact were not
observed. RTL recovery was confirmed; the application stopped. No watchdog was
relaxed and no gains were changed to force a pass. Two targeted wind tests
passed. ROS telemetry, parsed rejection details, unchanged-prefix/partial-write
analysis and native log 00000040.BIN are preserved in the run directory and its
diagnostic archive. The headless simulator was left running in native RTL.

## Instrumented tailwind repeat — 20 November 2026

Added opt-in, asynchronous timing traces to the vision controller, FCU interface
and supervisor. The local simulator auto-test enables them and generates
`command_timing_analysis.json`; normal launch defaults and all flight checks,
limits and tuning remain unchanged. Wire messages are unchanged. The installed
Humble executor passes only messages to callbacks, so DDS timestamps are
explicitly unavailable. Same-host/boot monotonic records join publication and
reception, and include callback wall/CPU durations, independent clock/process
heartbeats, scheduler counters and loss counts.

Run `20261120-173823-13596` verified all four appends and the original prefix,
confirmed GUIDED/vision ownership, reached terminal commitment and independent
blue-panel/fuselage contact at simulation time 417.371 s. It recorded 94 live
attitude messages and no supervisor faults. Latest pre-contact airspeed and
groundspeed were 13.8003 and 17.8450 m/s. Zero thrust was commanded. Capture and
retention were not confirmed. The completed simulation/application were stopped
after recording; post_test_cleanup.json records that cleanup separately from
the flight-end terminal-hold summary.

At accepted handoff the local command receipt age was 0.010374304 s and ROS stamp
age 0.012 s. Matching publication-to-reception-callback delay was 0.001497252 s.
All three timing recorders reported zero dropped records. Before arming, a
controller publication interval spanned 0.052 ROS seconds but 0.232469389 wall
seconds, while nearby callbacks took at most 0.001144846 s. This demonstrates
that slow simulation-clock progress can stretch normal ROS-timer publications
past the wall-clock watchdog without a long controller calculation. The earlier
0.300481315 s handoff failure was not reproduced, so its exact delivery or
scheduling history cannot be reconstructed retroactively.

All four packages built and 199 Python tests passed. A live, preview-only Humble
smoke check confirmed the corrected message-only callback and timing output.
Initial attempt `20261120-173544-12896` stopped before arming due to the new
logger expecting unsupported message metadata; that integration was corrected
and regression-tested before the completed flight. No launch pulse was consumed
by the failed preflight attempt. Native log 00000041.BIN, raw node timing files,
ROS/contact evidence, launch-time source snapshots and the timing analysis are
retained in the completed run directory and diagnostic archive.
