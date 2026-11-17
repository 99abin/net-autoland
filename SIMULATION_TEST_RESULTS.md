# Autonomous simulation test results — 2026-11-16

The original telemetry/takeoff failures have been fixed and autonomous flight
attempts have been executed against the managed local Gazebo/ArduPlane SITL.
**A successful net landing has not been demonstrated.**

Newer ArduPilot-only tuning trials are documented in
[ARDUPILOT_PARAMETER_TUNING.md](ARDUPILOT_PARAMETER_TUNING.md). The corrected
landing-speed configuration reduced the miss to 22.35 m; the earlier three
flights below remain the historical baseline.

The original aircraft spawn, 500 m approach, 100 m approach altitude and early
AUTO → GUIDED design are preserved. No real aircraft was connected or commanded.

## Confirmed fixes

- Request genuine FCU HEARTBEAT messages at 10 Hz in simulation time. The old
  default 1 Hz produced wall-clock gaps over three seconds when Gazebo ran
  slowly. The three-second watchdog remains unchanged; state is never fabricated.
- The explicit autonomous test backs up and verifies the four native takeoff
  trigger settings, including TKOFF_THR_MINACC=15 instead of the previous 0.
- Preserve the first supervisor abort reason. Later invalid plan messages no
  longer replace a telemetry failure with “Mission changed during landing” or
  repeatedly request the same abort.
- Calculate supervisor timestamp age in integer nanoseconds. This avoids a tiny
  negative floating-point result when two timestamps are exactly equal.
- Wait a bounded time for fresh readiness before arming and launching. No
  readiness, profile, ownership or vision check was removed.
- Add `auto-test --execute` for the explicitly authorized simulator-only flight
  experiment and `recover` for independent local SITL recovery. The default
  command without `--execute` cannot connect, write parameters or arm. Normal
  application launch defaults remain preview-only.
- Record native parameter baselines, actions, pre-arm/pre-launch checks, live
  telemetry and observed outcomes. A missed LAND point without vision ownership
  ends the experiment in recovery, not a reported capture success.

## Recorded flights

| Attempt | Takeoff and original mission | Altitude above net near LAND | Valid estimates on LAND leg | GUIDED / vision ownership |
| --- | --- | ---: | ---: | --- |
| Baseline native profile | Completed | 52.97 m | 0 | Neither observed |
| Explicit approach sink limit 5 m/s | Completed | 53.47 m | 0 | Neither observed |
| Landing airspeed 23 m/s, speed weight 0, approach limit 5 m/s | Completed | 25.70 m | 18 | Neither observed |

The net centre is 1.26 m relative to HOME. The measured aircraft altitude near
LAND was about 54–55 m relative to HOME in the first two runs and 26.96 m
in the third. These are actual recorded trajectories,
not a projection from YAML. The controller was not permitted to take ownership
without a valid target.

![Recorded approach comparison](.simulation/analysis/landing_approach.png)

The onboard TECS log shows the filtered height demand lagging the raw glide-path
demand by tens of metres. Changing TECS_APPR_SMAX from 0 to 5 did not resolve it;
that hypothesis must not be treated as a proven fix. The native control/model
tracking cause still needs investigation. The third trial improved the path and
produced valid vision, but still missed the height/time handoff envelope before
passing LAND. Later inspection found its requested landing airspeed was capped at cruise
because LAND_OPTIONS bit 1 was disabled. The configured 23 m/s value was not
the actual requested landing speed. Its maximum observed airspeed was 28.23 m/s; the nominal 23 m/s
target did not establish controlled speed tracking. These trial settings are
not a supported landing tune and were restored to their saved originals.

Maximum observed FCU state gaps were 0.660 s, 0.740 s and 0.734 s, respectively, with no
repeat of the original heartbeat/profile-readiness failure. ArduPlane reported
takeoff complete near 49.6 m, and the missions reached NAV_LAND.

Native trial values were restored and verified by a forced FCU parameter pull:
TECS_APPR_SMAX=0, TECS_LAND_ARSPD=-1, TECS_LAND_SPDWGT=-1. The takeoff
fix TKOFF_THR_MINACC=15 is retained. Evidence is in
`.simulation/analysis/native_trial3_restoration.json`.

The baseline recorder lost its ROS context after approximately 300 wall seconds.
RTL was subsequently commanded and confirmed through an independent ROS client.
The repeat ran in a dedicated background process, detected the missed target,
confirmed RTL automatically and stopped the application. The test utility now
also attempts independent recovery if its original ROS context closes.

## Evidence retained in this workspace

- Baseline: `.simulation/flight_runs/20261116-180737-20719/`
- Approach-limit trial: `.simulation/flight_runs/20261116-182151-23760/`
- Airspeed/weight trial: `.simulation/flight_runs/20261116-184014-26968/`
- Native DataFlash logs: `.simulation/sitl/logs/00000003.BIN` and `00000004.BIN`.
- Metrics: `.simulation/analysis/flight_metrics.json`.
- Camera snapshot: `.simulation/flight_runs/20261116-180737-20719/approach_camera.png`.
- Parameter restoration and further trial baselines: `.simulation/analysis/`.

The camera snapshot shows the target was not in view at that point in the
high approach. This is not evidence that vision guidance can intercept the net.

## Remaining blocker and limits

The simulator can take off and fly the mission autonomously. A reliable native
approach and a real transport-level vision handoff have not been achieved under
the preserved 100 m / 500 m configuration. The first two runs had no valid net
estimate on the LAND leg. The third had valid estimates but arrived too high
for the existing early-handoff window. No valid basis exists to force GUIDED.

Further work must establish a controllable native descent with this aircraft
model and its native speed/height controls, or review a gentler approach. A
longer leg or lower approach altitude would change the user's explicit geometry;
neither was silently applied. The detector, guidance law, readiness checks and
flare/handoff conditions were not altered to manufacture a passing result.

The normal application is still preview-only. Only `auto-test --execute` can
arm its verified local SITL; it never force-arms or disables arming checks.
All 112 Python tests pass, including the existing ownership/watchdog pipeline
with transport doubles. Those tests do not demonstrate airborne interception.

## Reproduce an experiment

Use [simulation_command.md](simulation_command.md), which includes the explicit
autonomous alternative and the operator-driven QGC workflow. A full experiment
may fail and recover. A successful build, a takeoff or observing GUIDED alone
must not be interpreted as a successful landing.

Do not remove vision/ownership checks to force a handoff. Establish a native
approach that brings the target into view and reaches the handoff envelope,
then validate the actual vision handoff. Close-range commitment, capture/contact
recognition and impact motor cutoff remain separate unvalidated prototype work.
