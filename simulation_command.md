# Full autolanding experiment: Gazebo, ArduPlane SITL and QGC

## Instrumented tailwind repeat — 20 November 2026

Run `20261120-173823-13596` confirmed all four appends, GUIDED ownership,
terminal commitment and fuselage contact with the blue panel in 4 m/s tailwind.
ROS commanded zero thrust. Capture/retention remains unconfirmed. The completed
headless simulator and retained ROS application were then stopped to close the
logs and free resources. At contact, the latest recorded airspeed/groundspeed
were 13.80 / 17.85 m/s.

`auto-test` now automatically enables timing diagnostics for vision_controller,
fcu_interface and landing_supervisor and saves them in each run's `timing/`
directory. It also writes `command_timing_analysis.json`. No extra flags are
needed, and normal launches keep diagnostics disabled. Commands are matched
using their timestamp and contents without changing ROS message definitions.
Logs include publication and reception, callback start/end and CPU duration,
ownership requests/decisions, wall/ROS clocks, background heartbeats, scheduler
counters and diagnostic drop counts. Disk writes run in background threads with
bounded queues; stop a completed experiment to close its recording files.

The successful handoff used a command received 10.37 ms earlier; matching
publication-to-callback delay was 1.50 ms, with no diagnostic records dropped.
The earlier 300 ms handoff rejection did not recur. Elsewhere before arming,
a 52 ms ROS-time publication interval took 232 ms of wall time while nearby
callbacks took at most 1.15 ms. This demonstrates slow simulation progress
stretching publication intervals past the 200 ms wall watchdog; it does not
prove the exact cause of the earlier unrecorded delay. Limits and control
algorithms were not relaxed or retuned.

This installed Humble executor does not expose DDS message-info timestamps to
subscription callbacks. Those fields are marked unavailable; the report can
measure the combined publication-to-callback delay and compare callback/process
activity, but cannot separate every middleware-internal stage. Repeated identical
command fingerprints, missing records and clock-comparison limits are documented
in the report. To regenerate it after a test:

```bash
cd /home/abin/td/autoland1
/usr/bin/python3 tools/analyse_command_timing.py .simulation/flight_runs/20261120-173823-13596
```

All four packages built, 199 Python tests passed, and a preview-only live Humble
smoke check verified the logger's message-only callback. An initial diagnostic
attempt (`20261120-173544-12896`) exposed and helped correct the metadata-callback
compatibility error before arming; that preflight attempt is retained separately.
Use the headless commands below to start a fresh test with the unchanged tailwind
configuration and updated QGC base plan.

## Current tailwind trial — 20 November 2026

`simulation/settings.yaml` now sets `wind_enu_mps: [0.0, -4.0, 0.0]`:
4 m/s toward south, behind the southbound landing approach. The updated QGC
base mission, native tuning, controller settings and handoff checks were kept.
The existing short-route profile `simulation/mission_headwind.yaml` is reused
unchanged; despite its filename, it only provides mission settings and the
20 s preparation margin, and does not define wind direction.

Headless run `20261120-171516-9420` verified all four appends and the unchanged
original mission. It planned a 648.33 m approach, compared with 381.67 m in the
headwind run. The aircraft reached native AUTO LAND but aborted at handoff:
the FCU's locally received preview was **0.30048 wall seconds old**, exceeding
the **0.2 s** receipt timeout. Its simulation stamp age was only 0.054 s;
validity, bounds, frame and plan ID all passed. The diagnostic proves which
gate rejected this attempt, but does not identify why reception/processing
paused. No GUIDED, live attitude output or net contact was observed; RTL was
confirmed. The simulator remains running in RTL, with the landing app stopped.

The headless commands below also apply to the tailwind configuration. Start a
fresh simulator before repeating the one-use launch-assisted flight. Evidence:
`.simulation/flight_runs/20261120-171516-9420/summary.json`,
`handoff_rejection_analysis.json`, `application.log`, and `diagnostics/`.
The targeted wind tests passed; no controller or watchdog limits were changed.

## Headless wind diagnostic repeat — 20 November 2026

Run `20261120-170434-7199` confirmed GUIDED ownership, terminal commitment and
fuselage contact with the blue panel. The earlier `No matching fresh bounded
preview command` rejection did not recur. The suspected clock/message-ordering
cause remains unconfirmed. Only rejection diagnostics were added; handoff
requirements, control gains and command limits were unchanged for this repeat.
The app commanded zero thrust and retained its terminal hold. Net contact is
not confirmation of capture or retention; the test deliberately reports that
distinction instead of declaring a completed capture.

That headwind run used a 4 m/s northward wind (headwind on the southbound approach),
your updated `autoland_1.plan` with TAKEOFF and one ordinary waypoint, and a
381.67 m wind-adjusted approach at 50 m. The simulator-only
`simulation/mission_headwind.yaml` uses a 20 s remaining-route preparation
minimum to accommodate this short base mission; the normal mission YAML keeps
30 s. The simulator controller override still uses a 0 m/s minimum and
13.3 m/s target; native approach/preflare targets remain 15 m/s and the pitch
response time constant remains 0.2 s. These are the existing experimental settings.

For a fresh, stopped simulator, Terminal 1:

```bash
cd /home/abin/td/autoland1
bash tools/build.sh
bash tools/sim.sh start --headless --no-qgc --no-preview --launch-assist
```

Terminal 2, while disarmed in MANUAL:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh check --require-mavros
bash tools/sim.sh import-plan --file /home/abin/td/waypoints/autoland_1.plan --apply
bash tools/sim.sh verify-attitude
# Continue only after PASS and MANUAL restored:
bash tools/sim.sh auto-test --execute --airborne-preparation --duration 1200
```

No windows are opened. Do not run a second landing application concurrently.
The latest test leaves the simulator and application running in zero-thrust
terminal hold after contact; stop that completed experiment before a fresh run.
Results are in `.simulation/flight_runs/20261120-170434-7199/summary.json`,
`handoff_contact_analysis.json`, `contacts.jsonl`, and `diagnostics/`.
If a future handoff is rejected, `application.log` now includes
`Handoff preview rejection:` with the FCU interface's local command timestamp,
receipt age, frame, plan ID, validity, reason and envelope check.

## Current trial: autoland_1.plan, 15 m/s native approach/preflare

Native TECS_LAND_ARSPD and LAND_PF_ARSPD are both 15 m/s. The custom ROS target
remains 13.3 m/s and the active simulator minimum remains 0 m/s. THR_MIN,
LAND_OPTIONS, tuned gains and TECS landing weighting were not changed.

Terminal 1:

```bash
cd /home/abin/td/autoland1
bash tools/build.sh
bash tools/sim.sh start --no-preview --launch-assist
```

This opens Gazebo and QGC. Leave the simulator running. In Terminal 2, with FCU
disarmed MANUAL and no other landing app running:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh import-plan --file /home/abin/td/waypoints/autoland_1.plan --apply
bash tools/sim.sh native-parameters --file simulation/15mps_approach_preflare_trial.yaml --apply
bash tools/sim.sh native-parameters --file simulation/pitch_response_02_trial.yaml --apply
bash tools/sim.sh verify-attitude
```

Only after the attitude check prints PASS:

```bash
bash tools/sim.sh auto-test --execute --airborne-preparation --duration 1200
```

The import backs up the existing mission and replaces it on the ground with
one TAKEOFF and three ordinary waypoints from the file, preserving actual FCU
HOME and aircraft spawn. It selects the new TAKEOFF while disarmed and verifies
readback. QGC's planned HOME is not applied to the FCU; its cruiseSpeed field is
planner metadata, not a MAV_CMD_DO_CHANGE_SPEED. Null waypoint yaw is normalized
to ArduPlane's stored zero; mission comparison tolerances are unchanged.
The imported TAKEOFF uses 0,0 as a current-position placeholder, so route-time
prediction skips that placeholder rather than inventing a leg to 0°,0°.
The airborne planner then appends Adjustment, Alignment, Approach and LAND.
The nominal constant TAS profile is now 15.45 m/s for the provisional 33.333 s
reference, predicting a 515 m calm-air approach. Actual duration is not guaranteed.
Do not edit/upload a mission or parameters from QGC during the experiment.


**Active simulator experiment:** the ROS minimum airspeed is 0 m/s through
`simulation/control_zero_minimum.yaml`, selected by `control_config` in
`simulation/settings.yaml`. Normal `tools/run.sh` retains 10 m/s. To restore
normal simulated launches, set `control_config: null`. The GUI/QGC experiment
reached HANDOFF_PENDING but aborted with `No matching fresh bounded preview
command`; it did not achieve GUIDED ownership or capture. See VALIDATION.md.

**Latest test:** native roll/pitch AUTOTUNE completed; the same-route retry
passed the earlier AUTO turn failure and reached LAND, but aborted during
HANDOFF_PENDING when airspeed fell below the ROS controller's 10 m/s minimum.
No GUIDED ownership or net capture was observed. See [AUTOTUNE.md](AUTOTUNE.md)
for gains, backups and the headless tuning procedure. The learned gains are
currently persisted in this workspace's SITL state; external model files were
not changed.

## Headless full test with the four-waypoint airborne planner

This mode adds **Adjustment → Alignment → Approach → LAND** during the original
mission. It keeps the early vision-qualified handoff and existing terminal
policy. No Gazebo, QGC or camera window is opened.

Terminal 1 (leave it running):

```bash
cd /home/abin/td/autoland1
bash tools/build.sh
bash tools/sim.sh start --headless --no-qgc --no-preview --launch-assist
```

Terminal 2, after startup:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh check --require-mavros
bash tools/sim.sh adaptive-base --apply
bash tools/sim.sh verify-attitude
```

Check that `verify-attitude` prints **PASS** and restores MANUAL. Then run:

```bash
bash tools/sim.sh auto-test --execute --airborne-preparation --duration 1200
```

This command explicitly permits simulated arming, AUTO, a finite catapult pulse,
mission writes and live guidance in the managed local simulator. It waits for
a verified FCU profile and original base mission before takeoff, then enables
the supervisor only after all four airborne appends pass readback. Do not run
another preview/app, QGC upload or mission edit concurrently.

The base must contain HOME, one TAKEOFF and ordinary waypoints, with no LAND.
For a fresh mission, upload that base using QGC before the headless test, close
QGC and leave the FCU disarmed in MANUAL. The `adaptive-base` command backs up
and reconciles only recognized previous appends; it does not create a missing
base mission or add a staging point. It rejects unknown mission changes.

Configure `t_reference_s`, `minimum_route_time_s`, `turn_radius_m`,
`alignment_straight_time_s` and `alignment_margin_m` in
`src/start/config/mission.yaml`; details are in [ADAPTIVE_APPROACH.md](ADAPTIVE_APPROACH.md).
Adjustment, Alignment and Approach stay at 50 m above HOME. The reference
circles help waypoint placement; ordinary L1 tracking does not enforce the arcs.

To inspect status in a third terminal:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh shell
# The command above opens a new shell. At its bash prompt run:
ros2 topic echo /autoland/airborne_preparation
# Ctrl+C stops echo. Then optionally:
ros2 topic echo /autoland/status
```

Read the printed final result and `.simulation/flight_runs/<run>/summary.json`.
`VERIFIED` with `verified_appends: 4` confirms mission preparation; it does not
mean the aircraft hit the net. Existing native tuning may still cause a height
miss. Do not stop the simulator mid-flight to restart the experiment; allow the
test to finish or confirm its recovery first. After the experiment:

```bash
bash tools/sim.sh stop
```

The ordinary static preflight workflow below remains available.

**Current native landing trial:** `TECS_LAND_SPDWGT=2.0` gives airspeed maximum
pitch priority throughout native LAND instead of the previous `-1` scaling.
Approach remains 15 m/s, preflare/custom ROS target 13.3 m/s, approach distance
500 m and waypoint height 50 m. This can flatten the descent and cause a height
miss. The setting applies to native AUTO landing; ROS guidance remains unchanged
after GUIDED takeover. The guarded trial saves the previous values in
`.simulation/parameter_trials/<run>/baseline.yaml` for explicit restoration.

**Airspeed-priority result, `20261118-153218-8542`:** the aircraft crossed the
net plane in AUTO at **15.00 m/s**, **10.61 m above the net centre**. Over the
last 100 m before the net, speed stayed between 14.999 and 15.004 m/s. It had
35 valid visual estimates, but no valid controller preview coincided with the
handoff height/time trigger. No GUIDED, live ROS commands or net contact occurred.
Native flare was logged at 10.6 m above the landing reference near the net;
the aircraft stopped on the ground about 103 m beyond it. The FCU log records
native auto-disarm. The GUI then exited and the launcher shut down SITL; the
observer returned a simulator-disconnected error rather than a clean landing
result. This improved approach speed tracking but did not achieve net landing.
See `.simulation/analysis/airspeed_priority_result.json` and
`.simulation/analysis/airspeed_priority_comparison.png`.

**Terminal-phase update:** full simulator runs now explicitly enable
`terminal_enabled` and `simulation_contact_enabled`. Normal launches keep both
disabled. After GUIDED ownership, the supervisor can enter `TERMINAL_COMMITTED`
when the predicted crossing fits the opening/uncertainty limits for 0.15 s,
with an accepted target no older than 0.25 s. Commitment must occur before the
1 s decision boundary; otherwise recovery is requested before commitment.
This is a prototype point-crossing check, not validated full-airframe clearance.

After commitment, guidance uses a minimum 0.7 s calculation horizon, keeps bank
within 5 degrees and retains pitch/slew limits. Reaching 2.5 m range or 0.2 s
time to target alone no longer aborts the attempt. Thrust ramps toward zero.
A terminal fault, missed target plane, or observed aircraft/net contact switches
to a zero-thrust attitude hold instead of automatically requesting a late RTL.
Pilot mode changes still release control. This is not a validated safe recovery.

The contact sensor watches the blue panel, excluding ground and decorative
figures. `NET_CONTACT` means contact, **not capture**. The panel is rigid; no net
restraint/deformation model exists. `COMPLETE` still requires operator
confirmation, fresh speed below 1 m/s, contact and target height. Ground impact
alone cannot satisfy confirmation. The runner keeps the application running
after terminal events while armed so its hold is not interrupted. It never
automatically disarms or reports contact as capture success.

**Latest terminal test, `20261117-220700-22801`:** takeoff and the mission
completed, actual GUIDED/vision ownership was confirmed, and terminal commitment
was followed by blue-panel/fuselage contact. The recorder saw 38 valid estimates,
83 live attitude messages, 8 held-target previews and 3 valid close-range terminal
previews. The last close preview remained valid at 1.55 m / 0.079 s with zero
thrust. No supervisor fault or RTL occurred. Contact was near ENU
`(-0.006, -49.985, 1.760)`; the aircraft then fell inverted to the ground.
This is **contact, not successful capture**. Preimpact airspeed was about
19.6 m/s, so the configured 15 / 13.3 m/s targets are not yet demonstrated at
arrival. The runner intentionally returned nonzero because capture was not
confirmed. Evidence: `.simulation/analysis/terminal_result.json`,
`.simulation/analysis/terminal_motor_outputs.json` and the run's `contacts.jsonl`.

**Perception update:** the current pipeline uses centre-pixel detection, a ray
intersection with the surveyed vertical net plane, and a stationary-target
Kalman filter. See [CENTER_RAY_ESTIMATION.md](CENTER_RAY_ESTIMATION.md) for
configuration, assumptions, message changes and limits. Rebuild all packages
before use; the historical flight results below used the previous PnP estimator.
The new estimator also needs fresh GPS covariance, local-pose history and a
verified landing plan. Existing handoff/freshness/control guards are unchanged.

**Centre-ray test result:** run `20261117-105653-25990` produced **20 valid
filtered estimates**, confirmed GUIDED and vision ownership, and sent **29 live
attitude messages**. It produced **15 valid held-target previews** through
detection gaps. The final abort was "Insufficient interception time" at the
unchanged 0.2 s time-to-target guard; RTL telemetry was confirmed. There was no
confirmed capture or demonstrated safe physical go-around. This validates a
handoff and exercised target holding, not complete net landing. Gazebo and the
camera remained open. Evidence:
`.simulation/analysis/center_ray_handoff_timeline.json` and
`.simulation/analysis/center_ray_result.json`.

**Previous PnP flight (15 m/s approach, 13.3 m/s preflare/ROS):** takeoff and the
mission completed, but no GUIDED/vision ownership or live custom commands
occurred. The trigger was true during valid previews, but vision bursts lasted
only about 0.46, 0.28, 0.31 and 0.35 seconds, below the 0.5 s requirement; each
ended with "Detection stream unavailable". Controller timing/freshness guards
also rejected some updates. AUTO continued. The aircraft stopped about 0.5 m
before the rigid panel and fell to ground level, consistent with a collision;
no controlled capture or net-plane crossing was established. The last reported
airspeed before the abrupt stop was about 19.7 m/s, so the configured 15 m/s did
not establish tracking that speed at arrival. The native FCU later disarmed.
Gazebo and the front camera were left open. Evidence:
`.simulation/flight_runs/20261117-092232-10239/` and
`.simulation/analysis/approach15_handoff_checks.json`.

**Previous flight (23 m/s approach, 13.3 m/s preflare/ROS):** takeoff and native AUTO landing completed,
but there was no GUIDED handoff or net capture. The aircraft crossed the net
position about **5.78 m above its centre**, continued in AUTO, reached the ground
about 200 m beyond the net and the native FCU disarmed. The handoff trigger was
false during every recorded valid controller preview; vision was also broken
into runs shorter than the required 0.5 s. The planned detector pause never ran
because it requires confirmed GUIDED vision ownership. Gazebo and the front
camera viewer were left open. Evidence:
`.simulation/flight_runs/20261117-001017-65851/` and
`.simulation/analysis/preflare13_handoff_checks.json`.

**Previous flight (preflare disabled, ROS target 15 m/s):** the updated supervisor successfully confirmed GUIDED
and vision ownership and sent 31 live attitude commands. It then hit the existing
minimum time-to-target limit ("Insufficient interception time") and requested RTL;
RTL telemetry was confirmed, but no successful capture or physical go-around was
established. No detector gap occurred before the limit, so retained-target
operation was verified in regression tests, not exercised in that flight.
The controlled detector-gap repeat was not launched before the next parameter
change below. Evidence: `.simulation/flight_runs/20261116-235627-62883/`.

**Current verified speeds:** native approach
`TECS_LAND_ARSPD` is now **15 m/s**, changed from 23 m/s in the recorded flight
above. The FCU accepted and read back 15.0 while disarmed; the previous value is
backed up at `.simulation/parameter_trials/20261117-092005-9758/baseline.yaml`.
Native preflare
`LAND_PF_ARSPD` and ROS `target_airspeed_mps` are both **13.3 m/s**. This is the
Merlin model README's estimated stall speed **11.3 m/s at 5 kg + 2 m/s**, not a
measured stall speed. The simulator's preflare value was written while disarmed
and read back successfully. Preflare changes speed only when the native preflare
conditions are reached. The 13.3 m/s targets were used in the earlier flight,
with approach still at 23 m/s; they did not establish a
successful vision handoff or net landing. The camera viewer was also opened,
so this was not a controlled speed-only comparison of processing performance.

**Historical results:** autonomous takeoff and the mission work. ArduPilot-only
parameter trials have reduced the recorded height miss from about 53 m to
21.84 m in the historical 100 m approach trials. The new 50 m mission descended
to 10.81 m above HOME, then aborted about 130 m before the net because vision
was unavailable at the early handoff boundary. RTL was confirmed. No net landing
or vision handoff has been demonstrated. See
[ARDUPILOT_PARAMETER_TUNING.md](ARDUPILOT_PARAMETER_TUNING.md) for the current
trial settings and measurements. No vision handoff or net
capture was achieved. See [SIMULATION_TEST_RESULTS.md](SIMULATION_TEST_RESULTS.md).
The commands below reproduce an experimental attempt; they are not a proven
autolanding procedure.

**Updated policy after that recorded attempt:** missing vision at the handoff
threshold now leaves the aircraft in native AUTO. GUIDED is requested only when
the threshold and all existing vision, preview, range and alignment checks pass.
If no handoff occurs, native AUTO continues landing. During an authorized handoff
or after takeover, detector gaps now use the last accepted stationary net
position, updated with every new valid observation. Invalid control commands,
stale telemetry and the other recovery checks still request recovery. This policy change
has regression checks; the historical flight above used the earlier policy.

This workflow enables the existing early AUTO → GUIDED handoff and vision
commands **in the managed local simulator**. You still arm and select AUTO in
QGC in the operator workflow. Only the explicit autonomous test below can arm
the managed local simulator. Use three terminals and keep
Terminal 1 and Terminal 2 running throughout the flight.

This is a complete flight-sequence experiment, not a promise of net capture.
The controller, aircraft dynamics and terminal interception are experimental.
Before commitment, the controller retains its 2.5 m / 0.2 s limits and ordinary
recovery checks. After commitment, the terminal policy above applies. A real
collision can occur without `COMPLETE`; net restraint and the terminal policy
still require validation.

## Autonomous run (explicit simulator-only opt-in)

After building, the following alternative runs the experiment without QGC button
presses. It preserves your existing uploaded mission; exactly one NAV_TAKEOFF
must exist. Headless Gazebo reduces laptop rendering load. Stop an older
application/session first.

Terminal 1:

```bash
cd /home/abin/td/autoland1
bash tools/build.sh
bash tools/sim.sh start --no-preview --launch-assist
```

Terminal 2, once startup has completed:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh mission-altitude --altitude 50 --apply
bash tools/sim.sh native-parameters --file simulation/airspeed_priority_parameter_trial.yaml --apply
bash tools/sim.sh verify-attitude
bash tools/sim.sh auto-test --execute --duration 900
```

`--execute` explicitly authorizes local SITL takeoff parameter writes, selecting
the current TAKEOFF item, arming, AUTO selection and one finite catapult launch.
It backs up the starting native parameters to the run directory, sets/rechecks
the four reviewed simulator takeoff settings, starts the full application,
waits for verified profile/mission, enables monitoring and applies all existing
launch guards. It never force-arms or disables checks. It records the result
and requests telemetry-confirmed RTL for faults or interruption before terminal
commitment. After commitment it preserves the terminal hold, including when
recording ends. Passing LAND without takeover does not request RTL.
At the normal observation time limit, an aircraft still in uninterrupted native
AUTO is left in AUTO and the application is stopped; the result explicitly says
landing was not confirmed. A native FCU disarm after LAND is recorded separately
and is not proof of net capture. No test command automatically disarms it.
Without `--execute`, `auto-test` prints its scope and takes no actions.

The `mission-altitude` command is a separate, explicit preflight edit of the
saved local simulator mission: all WAYPOINT and TAKEOFF items become **50 m
above HOME**, including the approach. HOME and the **1.26 m LAND** item are
preserved. Coordinates, commands, other item fields and the current index are
preserved and read back after the full upload. The original mission is saved
under `.simulation/mission_backups/`. Omit `--apply` to inspect without writing.
Run this only with the application stopped and SITL disarmed in MANUAL. The
normal mission manager still preserves the original mission and appends or
reuses its final approach/LAND pair.

Arming refusal, invalid configuration or stale telemetry stops the attempt;
do not bypass checks. Inspect the run's `application.log`, `actions.json`,
`telemetry.jsonl` and `summary.json`. A failed pre-arm attempt leaves the
catapult unused. After a flight attempt, restart the simulator before another
launch. The saved simulator takeoff settings persist; their original values
are recorded in `native_parameter_baseline.json`.

The simulator now requests genuine FCU heartbeat messages at 10 Hz in FCU time,
in addition to the telemetry rates below. This avoids default 1 Hz heartbeats
exceeding the unchanged three-second wall watchdog when Gazebo runs slowly.
It does not fabricate state messages or extend readiness leases.

The sections below describe the operator-driven QGC workflow.

## ArduPilot parameter-only tuning

Current experimental profiles, measurements and restore instructions are in
[ARDUPILOT_PARAMETER_TUNING.md](ARDUPILOT_PARAMETER_TUNING.md). The `native-parameters`
tool backs up native values and verifies readback while disarmed. No model,
firmware-source or ROS guidance changes are made by these profiles. A tuning
trial must not be interpreted as a successful landing procedure.

## 1. Build — Terminal 1

```bash
cd /home/abin/td/autoland1
bash tools/build.sh
bash tools/sim.sh doctor
```

Rebuild is required because `LandingStatus` now reports whether supervisor
monitoring is enabled. Scripts select system Python and Humble themselves;
Conda and pyenv can remain installed. Do not start a second MAVROS instance.

If an old landing application is running, stop it with Ctrl+C first. To stop
an old managed simulator:

```bash
bash tools/sim.sh stop
```

Wait for shutdown to finish. This preserves the saved SITL mission/parameters.

## 2. Open Gazebo and QGC — Terminal 1

```bash
bash tools/sim.sh start --no-preview --launch-assist
```

Leave it running. This opens Gazebo and QGC and starts SITL, MAVROS and the
camera/clock bridges. It builds the optional launch-assist plugin if needed.
An already running QGC window is reused. Merlin remains at its original spawn,
facing east (about 90° compass heading). The later landing is southbound (180°).
The launch plugin does nothing until an explicit guarded launch request.

To view the forward camera in a separate terminal, run:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh shell
```

At the new shell prompt, run:

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Keep that window open during the flight. It displays the real simulated nose
camera feed; it does not control the aircraft. Closing it stops only the viewer.

## 3. Check connections — Terminal 2

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh check --require-mavros --timeout 15
```

Require clock, camera, calibration and `connected=True, armed=False,
mode=MANUAL`. Missing detection/estimate/profile before starting the application
is expected. If disconnected or stale, fix that before continuing.

## 4. Verify the attitude interface — Terminal 2

Keep the aircraft **disarmed in MANUAL**, with no landing program running:

```bash
bash tools/sim.sh verify-attitude
```

This explicitly selects GUIDED while disarmed, sends small positive/negative
roll and pitch demands with **zero thrust**, checks actual navigation-demand
telemetry, then restores and confirms MANUAL. It uses the same conversion as
`fcu_interface` and the real MAVROS route. Separately, it compiles and tests the
local ArduPlane handler's mask-132 roll/pitch, yaw-ignore, thrust-decoding and
GUIDED-only rules. Positive thrust and airborne response are not tested here.

Require `PASS` and verify QGC shows disarmed/MANUAL. On failure, do not manually
set `partial_attitude_verified` to bypass it. Inspect the error and QGC state.

The report `.simulation/attitude_contract.json` is tied to this SITL process,
FCU binary, local handler, conversion code and MAVROS library. Repeat this step
after restarting the simulator or changing those files.

## 5. Prepare the base mission and takeoff settings in QGC

Keep disarmed/MANUAL. Upload your normal base mission:

1. Include a NAV_TAKEOFF item (command 22). A 50 m takeoff altitude and 15°
   minimum pitch are experimental starting values to review for Merlin.
2. Set every normal waypoint to **50 m relative to HOME** for the current test.
   Finish with a normal waypoint;
   the program appends the approach and LAND. Avoid an incompatible existing
   LAND, terminal RTL or unlimited loiter.
3. Plan enough room for Merlin to turn from its original east-facing launch
   toward the north end of the southbound approach. Do not put a sharp reversal
   immediately before the approach. Flight/turn tuning is still unvalidated.
4. Select the TAKEOFF item as current before flight. The helper requires
   command 22 active; it never changes the active mission index.

If reusing the saved mission from the earlier 100 m trials, stop the landing
application and run this while disarmed in MANUAL:

```bash
bash tools/sim.sh mission-altitude --altitude 50 --apply
```

It backs up the mission and verifies all flight waypoints at 50 m, preserving
HOME and the LAND height of 1.26 m. Download the changed mission into QGC to
refresh its display; do not overwrite it with an older 100 m plan.

Review/set these **SITL-only** native parameters in QGC before profile verification:

| Parameter | Required simulator launch value |
| --- | ---: |
| TKOFF_THR_MINACC | 15 |
| TKOFF_THR_MINSPD | 0 |
| TKOFF_THR_DELAY | 2 |
| TKOFF_ACCEL_CNT | 1 |

The operator launch helper checks the takeoff values without writing them.
The autonomous test explicitly writes and verifies the four simulator takeoff
settings after backing up the original values; native landing tuning is preserved. They hold native takeoff
throttle until the simulated acceleration trigger, followed by the native
0.2 s delay. These are simulator takeoff values, not an aircraft tune. An
experiment setting TECS_APPR_SMAX=5 did not solve the high approach and was
restored to the original 0; it is not required by this workflow. Do not disable arming checks to
bypass a refusal. Resolve the reported cause in QGC.

## 6. Start the full landing program — Terminal 2

```bash
bash tools/sim.sh autoland --upload-mission
```

Leave it running. This starts all seven nodes with simulation time, the generated
Merlin camera settings and the simulator parameter profile. It explicitly enables
mission upload, handoff and live simulated attitude output. Native parameter
writes remain disabled; the simulator profile preserves verified FCU values.
Normal `preview`, `start` and `tools/run.sh` defaults remain disabled.

`autoland` requires the successful attitude check from this session, a live
managed SITL, fresh disarmed/MANUAL telemetry and no competing application or
attitude writer. It does not arm or select AUTO.

The manager pulls your base mission, appends the final pair, uploads the complete
mission and verifies readback. An already matching pair is reused. The aircraft
must remain disarmed and outside AUTO until both verifications finish.

## 7. Verify readiness — Terminal 3

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh shell
```

**Wait for the new `bash-5.1$` prompt**, then enter the following commands there,
one at a time:

```bash
ros2 topic echo /autoland/parameter_profile
```

Wait for `verified: true`, then Ctrl+C stops only the topic display.

```bash
ros2 topic echo /autoland/landing_plan
```

Again require `verified: true`, then Ctrl+C. Read `reason` if either fails.
Do not arm while verification or a mission transaction is in progress.

Download/read the vehicle mission in QGC and inspect the final two items:

| Item | Latitude | Longitude | Altitude relative to HOME |
| --- | --- | --- | --- |
| Approach | -35.35921449 | 149.165237 | 50 m |
| LAND | -35.3637111 | 149.165237 | 1.26 m |

These are the configured **500 m**, southbound **180°** approach and net centre.
LAND altitude accounts for the simulator's HOME offset; it is not simply the
net's 1.35 m height above local ground. The 50 m to 1.26 m descent over 500 m
is about 5.6°. Earlier flight reports used 100 m and remain historical results.
The existing mission must bring the aircraft to 50 m before the approach;
changing the YAML does not lower the original QGC mission waypoints.
Do not upload the old base plan afterward and overwrite the appended pair.

Check the three full-mode flags:

```bash
ros2 param get /landing_supervisor handoff_enabled
ros2 param get /fcu_interface live_output
ros2 param get /fcu_interface partial_attitude_verified
```

All must be true. Then explicitly enable monitoring:

```bash
ros2 service call /landing_supervisor/enable std_srvs/srv/Trigger '{}'
ros2 topic echo /autoland/status
```

Require `success: true` from the service and `MISSION_READY` with
`monitoring_enabled: true` on the status topic. Ctrl+C stops the topic display.
A disabled supervisor can also show MISSION_READY; the boolean distinguishes it.

## 8. Check launch readiness — Terminal 3

```bash
bash tools/sim.sh launch --full-autoland
```

This is a dry run and applies no force. While disarmed/MANUAL, the arming/AUTO
blockers are expected. Resolve other blockers first: verified plan/profile,
enabled supervisor, reviewed takeoff parameters, original ground pose and fresh
telemetry. TAKEOFF must be active when AUTO is selected.
The original east-facing camera need not see the southward net at rest; vision
is checked by the supervisor during the landing approach.

## 9. Launch and record — Terminal 3, then QGC

```bash
bash tools/sim.sh launch --full-autoland --execute --wait 60 --record-duration 600
```

While this command waits, **you** arm and select AUTO in QGC. Require the active
TAKEOFF item. If all guards pass within 60 seconds, one Gazebo launch request
applies 120 N forward and 80 N upward for 0.75 simulation seconds. It does not
teleport or rotate the aircraft. The one-shot force expires independently of
this terminal. No second launch is possible until a simulator restart.

Watch QGC and Gazebo throughout. Native AUTO flies TAKEOFF and your original
mission, then the appended approach and LAND. With valid alignment, fresh stable
vision and a bounded preview command, the supervisor requests GUIDED near the
existing flare thresholds. It waits for actual GUIDED telemetry before granting
vision ownership. It does **not** wait for native flare completion or settled
sink rate. Custom control then attempts the net interception.

If vision is absent, weak, unstable or cannot produce a valid preview, AUTO
continues. A later detection can permit takeover only while all handoff checks
still pass, including the 8–80 m range and progress before LAND. A lost detection
before takeover restarts the 0.5 s stability check. During the GUIDED request or
after takeover, detection gaps use the retained net target. Invalid commands
request RTL before commitment; after commitment they demand a zero-thrust hold.
The target is stored in local coordinates, not as a frozen
camera/body vector. Its relative position is recalculated as the aircraft moves.

`target_hold_timeout_s` in `src/start/config/control.yaml` defaults to **0.0**:
no age expiry while the handoff/control remains active. A positive value limits
both wall and ROS observation age. This assumes a stationary net and continuous
local positioning; holding an erroneous observation does not correct it.
The controller clears its target on plan change, inactive/reset state or clock
rewind. The separate terminal policy is described at the top of this guide and
in [TERMINAL_GUIDANCE.md](TERMINAL_GUIDANCE.md).

Inspect the current target mode with:

```bash
ros2 topic echo /autoland/control_setpoint
```

`target_held: true` means the last accepted position is in use. `target_age_s`
continues increasing until a new valid image is accepted; the estimate's original
timestamp is not replaced with a fresh one. `valid: false` and `reason` explain
any guidance failure.

The observer records for up to 600 wall-clock seconds. Recording expiry or
Ctrl+C stops only the observer; neither stops the aircraft nor disables control.

## 10. Monitor, recover and interpret the result

Use a fourth terminal if Terminal 3 is recording:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh shell
```

After the new shell prompt, run one display at a time:

```bash
ros2 topic echo /autoland/status
ros2 topic echo /autoland/control_ownership
ros2 topic echo /mavros/state
ros2 topic echo /autoland/net_estimate
ros2 topic echo /autoland/control_setpoint
```

Use Ctrl+C between displays. The expected successful handoff sequence is
`MISSION_READY → AUTO_APPROACH → HANDOFF_PENDING → VISION_FINAL`, FCU
`AUTO → GUIDED`, then ownership `VISION` and live simulated commands.
A GUIDED service acknowledgement alone does not establish ownership. A successful
terminal gate then adds `TERMINAL_COMMITTED`. `NET_CONTACT` records physical
contact only. `MISSED_NET` or `TERMINAL_FAULT` requires operator assessment;
neither is a successful landing.

To request recovery while the application remains running:

```bash
ros2 service call /landing_supervisor/abort std_srvs/srv/Trigger '{}'
```

Before commitment, confirm actual recovery mode in QGC/FCU telemetry (normally
RTL). After commitment, this service requests the terminal zero-thrust hold;
it does not request a late RTL. You can also
select an appropriate recovery/pilot mode in QGC; a mode outside AUTO/GUIDED
releases custom ownership. Do not use Ctrl+C on the landing program as an
in-flight recovery command: that removes its ability to request recovery.

An ABORT is a meaningful failed attempt, not a completed landing. Read `reason`.
Check vision, track alignment, profile freshness and controller validity instead
of removing their checks. Commitment rejection should occur before the decision
boundary; the controller's original range/time limits remain before commitment.
After commitment, inspect the terminal hold/contact result rather than expecting
automatic RTL.

Only after a physically stopped, captured aircraft is observed, manual capture
confirmation is available (fresh ground speed below 1 m/s, observed net contact
and height within the configured target opening):

```bash
ros2 service call /landing_supervisor/confirm_capture std_srvs/srv/Trigger '{}'
```

`COMPLETE` is operator confirmation, not autonomous capture detection. Handle
simulated motor stop/disarming in QGC; no automatic arming/disarming is added.

Flight records are under `.simulation/flight_runs/<timestamp>-<pid>/`:

- `context.json`: configuration snapshots, hashes and attitude-check evidence.
- `native_takeoff_parameters.json`: read-only native takeoff settings.
- `telemetry.jsonl`: state, mission, vision, supervisor, ownership and live targets.
- `launch_result.json`: launch-plugin result after an accepted request.
- `summary.json`: observed takeoff, GUIDED, ownership, live-command count and faults.

The summary reports observations, not capture success or flight readiness.
For additional recording while the same managed session is alive:

```bash
bash tools/sim.sh record-flight --full-autoland --duration 600
```

## 11. Stop or repeat

After recovering/stopping the simulated aircraft, Ctrl+C in Terminal 2 stops
the application. Stop observers, then in another terminal:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh stop
```

Wait for shutdown. Restart from step 2 for another launch and repeat the
attitude check; reports from a previous SITL process are rejected. EEPROM and
mission files are retained. A reused QGC window stays open.

If profile/mission preparation fails while disarmed, after fixing the cause:

```bash
ros2 service call /parameter_manager/sync std_srvs/srv/Trigger '{}'
```

Wait for the profile to verify, then if needed:

```bash
ros2 service call /mission_manager/prepare std_srvs/srv/Trigger '{}'
```

Do not overlap pulls or repeatedly retry a busy transaction. An existing
nonmatching LAND must be edited explicitly in QGC. A parameter cache refresh
may temporarily revoke readiness; recovery requires a complete matching
readback while disarmed. In-flight readiness failures require recovery and
preflight verification, not bypasses.

For the native-AUTO-only comparison test, use
[NATIVE_AUTO_FLIGHT_TEST.md](NATIVE_AUTO_FLIGHT_TEST.md). In that workflow use
`preview --upload-mission` and omit `--full-autoland` from the launch helper.

## Latest headless result (19 November 2026)

Four airborne appends and original-prefix verification passed. The aircraft
then lost stability in native AUTO while turning toward Adjustment and reached
ground level before Alignment/Approach/LAND. No GUIDED handoff or live ROS
attitude commands occurred. The full landing test therefore **failed**; see
[VALIDATION.md](VALIDATION.md) for evidence and run IDs. The new geometry and
reference-circle radius do not guarantee that L1 flies that circle or that the
airframe handles the actual turn. Investigate native AUTO turn stability before
treating this route as a working net-landing demonstration.

## Headless throttle and low-speed characterization

See [FLIGHT_PERFORMANCE.md](FLIGHT_PERFORMANCE.md) for commands, measurement
conditions, backups and results. The measured model-angle-based stall-onset
region was approximately 13.4–13.6 m/s; it is not a real-aircraft stall-speed
measurement. The characterization procedure runs separately from landing and
retains the current gains. It does not change landing airspeed targets or THR_MIN.

### Result of the new-plan 15 m/s trial

Run `20261119-191511-17289` completed all four appends, confirmed GUIDED and
vision ownership, and sent 45 live ROS messages. It then aborted before terminal
commitment: predicted vertical crossing error was about 1.71 m with TTC about
0.92 s. No net capture occurred; RTL was confirmed. Gazebo and QGC were left
open. Full evidence is in VALIDATION.md. The custom ROS target remains 13.3 m/s;
this trial changed only the native approach/preflare targets and original route.

### Current pitch-response trial result

PTCH2SRV_TCONST is now 0.2 s in the simulator profile and persisted FCU state,
from the requested 1.2 s baseline. This is below the documented 0.4 s lower
bound. Run 20261119-223232-44789 confirmed GUIDED, terminal commitment and
simulator NET_CONTACT near the panel centre. The application applied zero
thrust and retained its terminal hold. Capture/retention remains unconfirmed.
Full diagnostics and the backup reference are in VALIDATION.md.
