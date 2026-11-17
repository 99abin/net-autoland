# Full autolanding experiment: Gazebo, ArduPlane SITL and QGC

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
In particular, the existing controller rejects estimates below 2.5 m forward
range or 0.2 s time to target; loss of valid commands invokes recovery (normally
RTL). These limits are preserved. There is no automatic contact detection,
net restraint model, impact motor cutoff, or validated close-range commitment
policy. A real collision can occur without `COMPLETE`.

## Autonomous run (explicit simulator-only opt-in)

After building, the following alternative runs the experiment without QGC button
presses. It preserves your existing uploaded mission; exactly one NAV_TAKEOFF
must exist. Headless Gazebo reduces laptop rendering load. Stop an older
application/session first.

Terminal 1:

```bash
cd /home/abin/td/autoland1
bash tools/build.sh
bash tools/sim.sh start --headless --no-qgc --no-preview --launch-assist
```

Terminal 2, once startup has completed:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh mission-altitude --altitude 50 --apply
bash tools/sim.sh native-parameters --file simulation/approach_airspeed_parameter_trial.yaml --apply
bash tools/sim.sh verify-attitude
bash tools/sim.sh auto-test --execute --duration 900
```

`--execute` explicitly authorizes local SITL takeoff parameter writes, selecting
the current TAKEOFF item, arming, AUTO selection and one finite catapult launch.
It backs up the starting native parameters to the run directory, sets/rechecks
the four reviewed simulator takeoff settings, starts the full application,
waits for verified profile/mission, enables monitoring and applies all existing
launch guards. It never force-arms or disables checks. It records the result
and requests telemetry-confirmed RTL for faults, interruption or an unfinished
handoff. Passing LAND without takeover no longer ends the run or requests RTL.
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
after takeover, detection gaps use the retained net target; invalid commands
still abort to RTL. The target is stored in local coordinates, not as a frozen
camera/body vector. Its relative position is recalculated as the aircraft moves.

`target_hold_timeout_s` in `src/start/config/control.yaml` defaults to **0.0**:
no age expiry while the handoff/control remains active. A positive value limits
both wall and ROS observation age. This assumes a stationary net and continuous
local positioning; holding an erroneous observation does not correct it.
The controller clears its target on plan change, inactive/reset state or clock
rewind. This adds no close-range commitment, capture detection or motor cutoff.

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
A GUIDED service acknowledgement alone does not establish ownership.

To request recovery while the application remains running:

```bash
ros2 service call /landing_supervisor/abort std_srvs/srv/Trigger '{}'
```

Confirm actual recovery mode in QGC/FCU telemetry (normally RTL). You can also
select an appropriate recovery/pilot mode in QGC; a mode outside AUTO/GUIDED
releases custom ownership. Do not use Ctrl+C on the landing program as an
in-flight recovery command: that removes its ability to request recovery.

An ABORT is a meaningful failed attempt, not a completed landing. Read `reason`.
Check vision, track alignment, profile freshness and controller validity instead
of removing their checks. The current minimum-range/time-to-target limits can
cause close-range recovery before impact; this experiment does not add a new
terminal-control or motor-cutoff policy.

Only after a physically stopped, captured aircraft is observed, manual capture
confirmation is available (fresh ground speed must be below 1 m/s):

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
