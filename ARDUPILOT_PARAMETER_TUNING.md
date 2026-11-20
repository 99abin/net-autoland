# ArduPilot-only simulator parameter tuning — 2026-11-16

## Current approach target and comparison (2026-11-17)

The requested native AUTO approach target is now TECS_LAND_ARSPD=15 m/s, with
preflare and ROS targets retained at 13.3 m/s. The managed disarmed SITL accepted
and read back 15.0; only this FCU parameter required a write. Baseline before
the change: `.simulation/parameter_trials/20261117-092005-9758/baseline.yaml`.
Both source FCU profiles explicitly specify 15, and the reproducible trial is
`simulation/approach_airspeed_parameter_trial.yaml`. Gains, geometry and
readiness checks were unchanged.

Flight `20261117-092232-10239` ran with Gazebo and the camera window visible.
It completed native takeoff and NAV_LAND, with no GUIDED, vision ownership or
live commands. The trigger was true during valid controller previews; initial
vision stability was the blocker: valid estimate runs of approximately 0.46,
0.28, 0.31 and 0.35 wall seconds each ended with "Detection stream unavailable".
Some previews also failed timestep/telemetry freshness checks. No detector
fault injection was used. The aircraft stopped approximately 0.5 m before the
panel at about 1.98 m above HOME, then fell to ground level, consistent with
contact against the rigid panel. No confirmed capture or net-plane crossing was
established. Last reported airspeed before the stop was about 19.7 m/s, so the
requested target does not establish that actual arrival speed was 15 m/s.
The native FCU disarmed; the utility issued no disarm or RTL command. Evidence:
`.simulation/analysis/approach15_handoff_checks.json` and
`.simulation/analysis/approach15_initial_metrics.json`.

## Earlier preflare and ROS target change (2026-11-17)

At the operator's request, native `LAND_PF_ARSPD` and the custom ROS controller's
`target_airspeed_mps` are now both **13.3 m/s**. Basis: the model README at
`/home/abin/td/aircraft_model/merlinv2/README.md` lists an estimated analytical
stall speed of 11.3 m/s at 5 kg; the requested margin is 2 m/s. This does not
establish an actual measured stall speed. AIRSPEED_MIN is not a measured stall
speed and was not used as one. Other native targets, including the previous
TECS_LAND_ARSPD=23 trial, remain unchanged.

The managed local SITL accepted LAND_PF_ARSPD=13.3 in disarmed MANUAL and forced
FCU readback returned 13.300000190734863. Its previous value 0.0 is backed up at
`.simulation/parameter_trials/20261117-000645-65121/baseline.yaml`.
The source and installed ROS control YAML both specify 13.3. This enables native
preflare with the existing LAND_PF_ALT=10 and LAND_PF_SEC=6 thresholds. These
speeds were used in flight `20261117-001017-65851`: native AUTO completed
takeoff and landing, but passed the net 5.78 m above its centre without GUIDED.
All valid controller previews occurred outside the configured handoff trigger;
valid estimate runs also repeatedly ended before 0.5 s. No detector pause was
performed. The FCU later disarmed on the ground about 200 m past the net. No
capture was established. The camera viewer was added for this run, so processing
load was another changed condition. Evidence is in
`.simulation/analysis/preflare13_handoff_checks.json`. The earlier results use
the previous preflare setting and ROS target, not these new speeds.

Gazebo's aircraft model, firmware source, native pitch-angle limits, ROS guidance,
and early-handoff logic were unchanged in the parameter comparisons below.
Those comparisons used a 100 m / 500 m approach. The current mission now uses
50 m for TAKEOFF and every flight waypoint, including the 500 m approach, at the
operator's request. LAND remains 1.26 m above HOME. This geometry change needs
its own flight result; it does not retroactively improve the historical trials.
These profiles are simulator experiments, not aircraft-specific flight tunes.

## Pitch response trial

`simulation/pitch_parameter_trial.yaml` changes PTCH_RATE_FF from approximately
0.345 to 0.6 and PTCH_RATE_I from approximately 0.15 to 0.6. P=0.04 and D=0
are unchanged, as are PTCH2SRV_TCONST=0.5 and the unlimited up/down rate settings.

The original and trial DataFlash logs were compared on the straight native LAND
leg (absolute roll below 10 degrees, raw height demand above LAND level). Rate
error samples additionally require absolute requested rate of at least 5 deg/s.

| Metric | Original gains | FF=0.6, I=0.6 |
| --- | ---: | ---: |
| Pitch-rate error RMS | 7.55 deg/s | 3.42 deg/s |
| Pitch-angle error RMS | 5.72 degrees | 6.35 degrees |
| Height above net near LAND | 52.97 m | 43.46 m |

Rate tracking improved, but overall angle tracking did not. This is a mixed
result, not a completed tune. The native flight completed takeoff and mission
items, then missed the target without GUIDED/vision ownership and confirmed RTL.
The test used the visible Gazebo GUI; the historical baseline was headless, so
these runs are not a controlled airframe-identification experiment.

ArduPilot describes FF as the main rate-response contribution and recommends
matching I to FF as an initial trim-response setting in its
[controller tuning guide](https://ardupilot.org/plane/docs/new-roll-and-pitch-tuning.html).
The specific 0.6 values here were selected as an experimental trial from the
recorded sluggish response, not prescribed as a tune by that documentation.

Evidence:

- Flight: `.simulation/flight_runs/20261116-211829-41699/`.
- Original gain backup: `.simulation/parameter_trials/20261116-211618-41362/baseline.yaml`.
- Native log: `.simulation/sitl/logs/00000006.BIN`.
- Comparison metrics: `.simulation/analysis/pitch_gain_comparison.json`.

## Landing airspeed configuration

Inspection found LAND_OPTIONS=0 and AIRSPEED_CRUISE=16. In this installed
firmware, AP_Landing::type_slope_get_target_airspeed constrains the landing target
to cruise unless LAND_OPTIONS bit 1 is set. Thus the earlier trial which wrote
TECS_LAND_ARSPD=23 with LAND_OPTIONS=0 did not actually request a 23 m/s target.
Its logged TECS true-airspeed demand remained around 16.5 m/s at this elevation.
This changes the interpretation of that earlier trial; it did not establish
the aircraft's ability to track a true 23 m/s landing target.

The current paired trial, `simulation/landing_airspeed_parameter_trial.yaml`,
keeps FF/I at 0.6 and requests LAND_OPTIONS=2 and TECS_LAND_ARSPD=23. Existing
AIRSPEED_MAX=23 is not increased. Other option bits are preserved by validation.
The actual speed demand and descent must be checked in the new flight log;
setting/readback of a number alone is not sufficient evidence of its effect.

Relevant installed source:
`/home/abin/ardupilot/libraries/AP_Landing/AP_Landing_Slope.cpp`, lines 382–384,
and `AP_Landing.h` option ON_LANDING_USE_ARSPD_MAX. The
[upstream landing implementation](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Landing/AP_Landing_Slope.cpp)
contains the corresponding target constraint.

## Corrected landing-speed flight result

Flight `.simulation/flight_runs/20261116-213918-44800/` completed takeoff, the
original mission and NAV_LAND, then confirmed RTL recovery. The aircraft passed
LAND 22.35 m above net centre, compared with 43.46 m in the pitch-gain-only run.
The native TECS log (`00000007.BIN`) now shows true-airspeed demand reaching
approximately 23.7 m/s, confirming the higher target was actually used.
Maximum observed airspeed was 27.18 m/s; this is not established speed-limit
tracking or a validated tune.

There were 15 valid estimates on the LAND leg, but no valid controller previews
or GUIDED/vision ownership. Preview reasons included "Initial attitude outside
final-phase envelope". Thus corrected native parameter configuration improved
the descent but did not establish a successful handoff. The ROS final-phase
pitch bounds and all readiness checks remain unchanged.

Straight-approach native pitch angle error RMS was 5.51 degrees and pitch-rate
error RMS was 3.74 deg/s using the same filtering as above. A further comparison
was run with `simulation/pitch_feedback_parameter_trial.yaml`: P increased from
0.04 to 0.08 while the other tested native settings were held constant. Flight
`.simulation/flight_runs/20261116-215650-48148/` passed LAND 21.84 m above net
centre, with 18 valid estimates but no GUIDED/vision ownership or live commands.
RTL was confirmed. The approximately 0.5 m difference is not a substantial fix.

## Current 50 m mission

The saved mission was backed up to
`.simulation/mission_backups/20261116-231053-55291/before.json`. The local FCU
accepted and read back 50 m for TAKEOFF and every WAYPOINT. HOME, LAND at 1.26 m,
coordinates and current index were preserved. The YAML approach is also 50 m;
the 500 m distance and 180 degree heading are unchanged. The 50 m retest retains
the preceding P=0.08, FF/I=0.6, LAND_OPTIONS=2 and TECS_LAND_ARSPD=23 settings so
that the geometry change can be compared without another parameter change.

The 50 m flight `.simulation/flight_runs/20261116-231145-55537/` completed
takeoff and the normal mission at approximately 49–51 m. Native AUTO descended
to 10.81 m above HOME (9.55 m above net centre), but the supervisor aborted about
130 m before the net: "Handoff boundary reached without valid aligned vision
control". All 415 recorded LAND-leg estimates were invalid; all 420 controller
previews were invalid (primarily "Vision unavailable"). No GUIDED, vision
ownership or live commands occurred. RTL was telemetry-confirmed.

The descent now reaches the early handoff boundary before the net. However,
this is not a net-crossing result and cannot be reported as a measured reduction
of the height miss at the net. At abort, descent was approximately 3.83 m/s,
making the time trigger about 9.59 m above net centre. The estimated along-track
range was still approximately 130 m, beyond the configured 80 m handoff range.
The range/trigger compatibility and perception readiness now need investigation;
no handoff condition, detector threshold or controller bound was bypassed.
Evidence: `.simulation/analysis/50m_handoff_timeline.json` and
`.simulation/analysis/50m_approach_result.json`.

## Reproduce or restore a trial

Start a managed local simulator with --launch-assist and --no-preview. Keep it
disarmed in MANUAL, with the application stopped. The parameter tool refuses
armed/flight states, unrelated parameter names, unsupported option-bit changes,
invalid gains, airspeed outside the existing native limits, and a higher-than-
cruise landing target without the required option bit.

Inspect without writing:

```bash
bash tools/sim.sh native-parameters --file simulation/pitch_parameter_trial.yaml
```

Apply the reviewed simulator trial, creating a baseline before the first write
and forcing an FCU pull before readback verification:

```bash
bash tools/sim.sh native-parameters --file simulation/landing_airspeed_parameter_trial.yaml --apply
bash tools/sim.sh verify-attitude
bash tools/sim.sh auto-test --execute --duration 1500
```

`native-pitch` remains an alias of `native-parameters`. Neither mode arms or
writes by default. The separate auto-test --execute explicitly authorizes the
managed local simulator flight. Ordinary application defaults remain disabled.

To restore, restart the simulator into disarmed MANUAL first and apply the
appropriate generated baseline.yaml with the same native-parameters --apply
command. The paired-airspeed trial baseline restores its preceding state (which
already had the trial pitch gains). Apply the earlier pitch baseline as well to
restore the original FF/I values. Backups and action/readback records are in
`.simulation/parameter_trials/`. Failed/partial writes are recorded and are not
silently rolled back.

## Airspeed-priority comparison (2026-11-18)

The current simulator profile explicitly sets `TECS_LAND_SPDWGT=2.0`, replacing
the previous native `-1.0` scaling toward height priority near LAND. This gives
airspeed maximum pitch priority during native AUTO landing. It does not change
the custom ROS guidance after GUIDED takeover and does not establish a suitable
landing tune. A flatter descent or height overshoot is possible.

Only that native value was written. The targets remain `TECS_LAND_ARSPD=15.0`
and `LAND_PF_ARSPD=13.3`, with a 50 m / 500 m approach. Original values are backed
up in `.simulation/parameter_trials/20261118-152756-6616/baseline.yaml`.
The repeatable trial is `simulation/airspeed_priority_parameter_trial.yaml`;
`simulation/fc_parameters.yaml` also explicitly expects the new weighting.
The guarded parameter helper accepts its firmware range, -1 through 2, and
rejects values outside it. All 152 Python tests passed.

Run `20261118-152836-6810` was interrupted by an agent-service restart shortly
after takeoff. The simulator and observer stopped; no landing conclusion can be
drawn from it. The repeat uses detached simulator/test processes to keep their
lifetime independent of the tool connection.

Repeat `20261118-153218-8542` completed takeoff and the mission. The FCU DataFlash
log `00000016.BIN` confirms the new weighting and records speed weighting 1 /
height weighting 0 internally (after TECS normalizes its weights). At the net
plane, interpolated ROS local-position/airspeed data gives 15.00 m/s and 10.61 m
above the net centre, cross-track -0.15 m. Airspeed over the final 100 m before
the net was 14.999–15.004 m/s. Native flare was logged at 176.919 s, 10.6 m above
the landing reference, near the net position. No PREFLARE stage was logged
before it: the aircraft remained too high for that trigger before crossing.

There were 35 valid net estimates and 54 valid controller previews, but none
coincided with the handoff height/time window. It remained in AUTO; no live ROS
commands or net contact occurred. It stopped on the ground at ENU approximately
(0.064, -153.063, 0.09), about 103 m beyond the net. The FCU log records native
auto-disarm at 208.519 s. The Gazebo GUI then exited normally, causing managed
SITL shutdown; the observer ended with a simulator-not-alive error before it
received the final disarmed ROS state. No software capture confirmation was made.

Compared with the previous 19.56 m/s precontact reading, approach speed control
improved, but the net was missed high. Full speed priority is not a completed
landing tune. Evidence: `.simulation/analysis/airspeed_priority_result.json`,
`.simulation/analysis/airspeed_priority_comparison.png`, the run's telemetry,
and `.simulation/sitl/logs/00000016.BIN`. All 152 Python tests passed; ordinary
preview flags, the early handoff, geometry, PID gains and ROS controller were
unchanged in this comparison.
