# Merlin SITL low-speed and throttle characterization — 19 November 2026

Two headless test flights estimated a stall-onset region of **13.4–13.6 m/s**
for this simulated aircraft/configuration. This is a model-angle-based estimate,
not a certified aerodynamic stall speed or a measurement of the real aircraft.

## Refined near-level throttle map

| Reported throttle | Mean speed | Speed standard deviation | Mean vertical speed |
|---:|---:|---:|---:|
| 65% | 14.79 m/s | 0.12 m/s | +0.01 m/s |
| 70% | 16.33 m/s | 0.02 m/s | −0.04 m/s |
| 75% | 17.21 m/s | 0.06 m/s | +0.25 m/s |
| 80% | 20.63 m/s | 0.15 m/s | −0.42 m/s |

These are accepted near-level segments, not exact equilibrium measurements.
They use the last 10 simulated seconds of each 35-second throttle segment.
Acceptance requires speed standard deviation <0.5 m/s, acceleration magnitude
<0.15 m/s², mean vertical speed magnitude <0.5 m/s and mean height error <10 m.
Reported throttle is the FCU throttle percentage, not RC PWM or measured thrust.
There is no fitted universal throttle-to-speed function.

In the coarse flight 62% briefly sustained about 13.94 m/s; in the refinement
it crossed the stall-angle proxy and lost height. Treat that point as marginal,
not a reliable minimum-throttle choice. The coarse 87% segment was still
 descending and was excluded. At 50% the coarse flight lost height; its speed
increased during the ensuing descent, which is not a level-flight mapping point.

## Stall-onset evidence

The main-wing LiftDrag plugins specify `a0=0.065 rad` and
`alpha_stall=0.16 rad`. Gazebo's plugin adds a0 to geometric angle of attack
before selecting its post-stall lift-coefficient branch. Thus body angle near
0.095 rad (5.44 degrees) is the approximate main-wing branch threshold here.
Reference: [Gazebo LiftDrag source](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/lift_drag/LiftDrag.cc).

The estimate uses aircraft orientation and local velocity to calculate body
angle of attack. It assumes calm air and neglects rotation-induced velocity at
the wing pressure centre, wind-estimation errors and telemetry timing errors.
It is not a direct measurement of the simulator plugin's internal stall state.

- Coarse flight: threshold proxy crossed at 13.37 m/s, vertical speed −0.07 m/s.
- Refined flight: threshold proxy crossed at 13.56 m/s, vertical speed +0.09 m/s.
- Each was followed by significant height loss while attempting to hold height.
- The estimator and reported FCU airspeed were close in these observations.

The flights were straight and nearly wings-level. Banked-turn stall speed,
other payloads/CG, gusts, flare performance and real-flight airspeed calibration
remain unmeasured. A 13.3 m/s target has no demonstrated stall margin against
these observations. No airspeed target or throttle minimum was changed based
on this test.

## Procedure and artifacts

The landing app was stopped. A fresh managed headless SITL session used the
existing finite launch assist and a long straight TAKEOFF climb to 600 m above
HOME. FBWA provided native attitude control; a bounded slow altitude loop issued
RC pitch requests while throttle was reduced in steps. Actual speed, throttle,
height error, vertical speed, pitch, roll and estimated body angle were logged.
Freshness/mode/attitude/minimum-height checks remained active. At a sustained
level-flight capability loss the sweep stopped, released RC and confirmed RTL.
No deliberate ground impact, gain tuning or mission edit was required.

Runs:

- `.simulation/flight_runs/20261119-184815-11677/` — coarse sweep.
- `.simulation/flight_runs/20261119-185506-12964/` — refined sweep.

Each has `characterization.csv`, baseline/after parameters, complete telemetry
and a result. Historical runs use `autotune_result.json` because they reuse the
launch/backup harness; future characterization runs use
`characterization_result.json`. `passed` means measurement collection completed,
not that level flight succeeded at every throttle or that the aircraft is safe.

Exports: `.simulation/analysis/characterization_20261119/throttle_map.png`,
`throttle_map.csv`, and `results.json`. The PNG is a standalone chart.

All recorded roll/pitch rate gains were unchanged before/after both flights.
Original takeoff settings (including 50 m height and 200 m distance) were
restored and read back in a fresh disarmed MANUAL session. The simulator was
then stopped. No windows or external aircraft connections were opened.

Repeat, in separate terminals with no other app or RC source running:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh start --headless --no-preview --no-qgc --launch-assist
```

```bash
cd /home/abin/td/autoland1
bash tools/ros_env.sh --overlay /usr/bin/python3 tools/simulation_autotune.py --execute --characterize
```

The second command explicitly permits simulated arming, takeoff, temporary
parameter writes and RC inputs. It backs up settings but does not automatically
restore them in flight. Restore the recorded takeoff settings while disarmed
before another landing experiment; do not use the temporary 620 m/5000 m
TAKEOFF-mode settings as an aircraft configuration.
