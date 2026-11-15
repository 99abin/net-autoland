# Quick start: parameter-enabled handoff build

1. Switch your repository to `handoff` and extract the complete archive in its root.
   Merge any local edits to the same files before overwriting them.
2. Source ROS 2, install dependencies, and rebuild (message interfaces changed):

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash tools/build.sh
source install_handoff/setup.bash
```

3. Review `src/start/config/fc_parameters.yaml`, `mission.yaml`, `perception.yaml`,
   and `control.yaml`. Set each applicable `configured` flag only after reviewing
   the values. A null FCU parameter preserves its current value. Set a validated
   positive `LAND_PF_ARSPD` if native preflare is desired.
4. Run MAVROS and the calibrated camera driver separately. Keep the FCU disarmed
   and outside AUTO during parameter sync and mission preparation.
5. Preview using the source configuration files:

```bash
bash tools/run.sh
```

6. To apply the reviewed FCU profile and extend the mission automatically:

```bash
bash tools/run.sh parameter_write_enabled:=true upload_enabled:=true
```

Wait for `/autoland/parameter_profile` to report verified READY and for the mission
readback to verify. No arming or takeoff is automatic. Handoff and live actuation
remain disabled unless separately enabled, as described in README.md.

To export the complete native landing profile once verified:

```bash
ros2 service call /parameter_manager/export_profile std_srvs/srv/Trigger '{}'
```

The original early-handoff condition is retained. This revision does not wait for
flare sink-rate settling. It remains an experimental SITL prototype; real ROS/
MAVROS integration and aircraft/net capture validation are not complete.
