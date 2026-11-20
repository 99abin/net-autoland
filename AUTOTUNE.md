# Managed SITL AUTOTUNE trial

A completed ArduPlane AUTOTUNE trial was obtained headlessly on 19 November
2026. The firmware reported `Roll: Finished` and `Pitch: Finished`, then RTL
was confirmed. These are learned simulator gains, not mathematically optimal
values or aircraft flight approval. AUTOTUNE changes rate PID/FF, filters and
attitude time constants; it does not tune TECS or the landing guidance law.

Completed run: `.simulation/flight_runs/20261119-115827-25668/`.
The exact read-back gains are in [simulation/autotune_gains.yaml](simulation/autotune_gains.yaml).
The original gain set is in [simulation/autotune_original_gains.yaml](simulation/autotune_original_gains.yaml).
Both files are reference exports; `parameter-trial` intentionally retains its
narrower allowlist and will not blindly apply these exports.

| Axis | Gain | Original | Learned |
|---|---|---:|---:|
| Roll | P | 0.0800 | 0.325697 |
| Roll | I | 0.1500 | 0.210911 |
| Roll | D | 0 | 0.013735 |
| Roll | FF | 0.3450 | 0.210911 |
| Pitch | P | 0.0800 | 0.604866 |
| Pitch | I | 0.6000 | 0.659945 |
| Pitch | D | 0 | 0.023220 |
| Pitch | FF | 0.6000 | 0.659945 |

Roll time constant became 0.8 s; pitch time constant became 1.2 s. Filters also
changed. Gains survived restart and a forced FCU parameter pull. Temporary
TKOFF settings and AUTOTUNE axes/level were restored before the landing retry.
The airframe/model, landing geometry, native landing speed/TECS settings and
ROS guidance law were not changed for that comparison.

## How the trial works

`tools/simulation_autotune.py --execute` requires a fresh managed local simulator,
no landing app or competing attitude writer, and connected disarmed MANUAL.
It backs up all available roll/pitch rate parameters, time constants and selected
flight settings before writes. It uses the existing MAVROS router, not a new
vehicle connection. It reads the configured GCS ID and confirms received RC
channels before arming; an earlier companion-ID attempt was ignored by firmware.
Do not run QGC or another RC source concurrently.

The test explicitly arms the simulated aircraft, uses the finite launch assist,
and climbs on a long straight TAKEOFF leg to 600 m above HOME before AUTOTUNE.
Numeric mode 13 is confirmed from raw HEARTBEAT because Humble's MAVROS does not
name this newer TAKEOFF mode. Alternating RC demands excite roll and pitch in
separate blocks. ArduPlane's AUTOTUNE controls gain learning. The test stops if
telemetry is stale, mode changes, airspeed falls below 14 m/s, height falls below
65 m, roll exceeds 65 degrees or pitch exceeds 40 degrees. It requires both
completion messages; intermediate estimates are not a completed result.

A failed/incomplete trial can leave parameter changes. Inspect its baseline and
after files; do not assume automatic rollback. Cleanup releases RC overrides and
requests RTL with mode confirmation when possible. RTL confirmation does not
establish that a crashed aircraft recovered. Reset the local simulator before
another trial. No automatic disarm or aircraft connection is performed.

## Repeat headlessly

Terminal 1:

```bash
cd /home/abin/td/autoland1
bash tools/sim.sh start --headless --no-qgc --no-preview --launch-assist
```

Terminal 2, after startup:

```bash
cd /home/abin/td/autoland1
bash tools/ros_env.sh --overlay /usr/bin/python3 tools/simulation_autotune.py --execute
```

The command explicitly permits simulated parameter writes, arming and RC inputs.
The landing application must be stopped. Logs, baseline, after values, completion
messages and result are saved under the printed `.simulation/flight_runs/` path.
After confirmed recovery, stop and restart SITL before a landing comparison.
Restore the backed-up temporary takeoff settings while disarmed, retain the
verified completed gains, and follow [simulation_command.md](simulation_command.md).

Official procedure: [ArduPlane AUTOTUNE](https://ardupilot.org/plane/docs/automatic-tuning-with-autotune.html).

## Landing retry result

Run `.simulation/flight_runs/20261119-120718-27384/` used the persisted completed
gains and the same original six-item route. It passed the earlier AUTO turn
failure, flew all appended items and entered LAND. This supports tuning as a
contributor to the previous instability; AUTOTUNE also changed filters and time
constants, so it does not isolate one PID coefficient.

Net landing still failed. During HANDOFF_PENDING the airspeed fell to about
8.75 m/s, below the ROS controller's unchanged 10 m/s operating minimum. Preview
became invalid and the supervisor aborted, then RTL was confirmed. There was no
observed GUIDED telemetry, live ROS attitude command or net capture. The valid
visual estimates immediately before the abort distinguish this from a proven
net-detection-loss failure. The next investigation is native speed/descent
behaviour near handoff; these gains alone do not solve the full landing.
