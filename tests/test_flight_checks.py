"""Launch authorization checks; no ROS, vehicle or Gazebo processes are started."""

import copy
import importlib.util
import math
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
import sys

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("flight_checks", ROOT / "tools/flight_checks.py")
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)


class FlightChecksTests(unittest.TestCase):
    def test_supervisor_snapshot_accepts_exact_zero_nanosecond_age(self):
        sys.path.insert(0, str(ROOT / "tools"))
        sys.path.insert(0, str(ROOT / "src/autoland"))
        from simulation_flight import FlightObserver
        observer = object.__new__(FlightObserver)
        stamp = NS(sec=53, nanosec=650000000)
        header = NS(stamp=stamp)
        observer.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=53650000000)))
        observer.received = {"supervisor": __import__("time").monotonic()}
        observer.messages = {
            "profile": NS(verified=True, profile_id="profile", header=header),
            "plan": NS(verified=True, parameter_profile_id="profile", plan_id="plan", header=header),
            "supervisor": NS(state="MISSION_READY", monitoring_enabled=True, header=header,
                             plan_id="plan", parameter_profile_id="profile"),
        }
        self.assertTrue(observer.snapshot({}, {"ready": True})["supervisor_fresh"])

    def snapshot(self):
        result = dict(connected=True, armed=True, mode="AUTO", current_command=22,
                      profile_verified=True, plan_verified=True, profile_id="verified",
                      plan_profile_id="verified", mission_matches=True, live_output=False,
                      profile_stamp_age_s=0.1, plan_stamp_age_s=0.1,
                      handoff_enabled=False, speed_mps=0., relative_alt_m=0., roll_deg=0.,
                      pitch_deg=0., yaw_deg=0., plugin_ready=True)
        result["native_takeoff_ok"] = True
        result.update({name + "_received": 99.9 for name in
                       ("state", "pose", "velocity", "altitude", "profile", "plan")})
        return result

    def test_complete_operator_armed_auto_takeoff_is_authorized(self):
        self.assertEqual(checks.launch_blockers(self.snapshot(), 100), [])

    def test_airborne_mode_requires_verified_base_and_explicit_recovery_flags(self):
        s=self.snapshot()
        s.update(live_output=True,handoff_enabled=True,partial_attitude_verified=True,
            airborne_preparation_enabled=True,fcu_airborne_preparation_enabled=True,
            preparation_state='BASE_READY',base_matches=True,base_profile_id='verified',
            preparation_received=99.9,preparation_stamp_age_s=.1,
            plan_verified=False,mission_matches=False,supervisor_state='WAITING',supervisor_enabled=False)
        self.assertFalse(checks.launch_blockers(s,100,True,True))
        for key,value in dict(base_matches=False,base_profile_id='other',preparation_received=90,
            preparation_stamp_age_s=-1,airborne_preparation_enabled=False,
            fcu_airborne_preparation_enabled=False,preparation_state='FAILED').items():
            self.assertTrue(checks.launch_blockers({**s,key:value},100,True,True))

    def test_each_arming_mode_readiness_and_live_control_interlock(self):
        bad = dict(connected=False, armed=False, mode="GUIDED", current_command=16,
                   profile_verified=False, plan_verified=False, profile_id="",
                   plan_profile_id="different", mission_matches=False, live_output=True,
                   handoff_enabled=True, plugin_ready=False, native_takeoff_ok=False)
        for key, value in bad.items():
            with self.subTest(key=key):
                snapshot = self.snapshot()
                snapshot[key] = value
                self.assertTrue(checks.launch_blockers(snapshot, 100))

    def test_missing_flag_services_cannot_authorize_launch(self):
        for key in ("live_output", "handoff_enabled"):
            snapshot = self.snapshot()
            del snapshot[key]
            self.assertTrue(checks.launch_blockers(snapshot, 100))

    def test_each_required_stream_must_be_fresh_and_not_future_dated(self):
        for key in ("state", "pose", "velocity", "altitude", "profile", "plan"):
            for stamp in (90, 101):
                snapshot = self.snapshot()
                snapshot[key + "_received"] = stamp
                self.assertTrue(checks.launch_blockers(snapshot, 100))
        for key in ("profile_stamp_age_s", "plan_stamp_age_s"):
            for age in (-0.1, 3.01, math.inf):
                snapshot = self.snapshot()
                snapshot[key] = age
                self.assertTrue(checks.launch_blockers(snapshot, 100))

    def test_ground_speed_height_and_attitude_limits(self):
        for key, value in (("speed_mps", 2.01), ("relative_alt_m", 0.81),
                           ("roll_deg", 15.1), ("pitch_deg", -20.1), ("yaw_deg", 11),
                           ("speed_mps", math.nan), ("relative_alt_m", math.inf)):
            with self.subTest(key=key, value=value):
                snapshot = self.snapshot()
                snapshot[key] = value
                self.assertTrue(checks.launch_blockers(snapshot, 100))

    def test_force_configuration_has_hard_bounds(self):
        config = dict(duration_s=.75, forward_force_n=120., upward_force_n=80.,
                      maximum_initial_speed_mps=2., maximum_initial_height_m=.35)
        checks.validate_assist_config(config)
        for key, value in (("duration_s", 1.01), ("duration_s", 0),
                           ("forward_force_n", 201), ("upward_force_n", -1),
                           ("upward_force_n", 101), ("maximum_initial_speed_mps", 3),
                           ("maximum_initial_height_m", 1), ("duration_s", math.nan)):
            with self.subTest(key=key, value=value):
                invalid = copy.copy(config)
                invalid[key] = value
                with self.assertRaises(ValueError):
                    checks.validate_assist_config(invalid)

    def test_flight_logs_are_standard_json_with_nonfinite_values_removed(self):
        self.assertEqual(checks.finite_json({"values": [math.nan, math.inf, 1.]}),
                         {"values": [None, None, 1.]})

    def test_native_throttle_waits_for_reviewed_acceleration_trigger(self):
        values = {"TKOFF_THR_MINACC": 15., "TKOFF_THR_MINSPD": 0.,
                  "TKOFF_THR_DELAY": 2, "TKOFF_ACCEL_CNT": 1}
        self.assertEqual(checks.native_takeoff_blockers(values), [])
        for key, value in (("TKOFF_THR_MINACC", 0), ("TKOFF_THR_MINSPD", 20),
                           ("TKOFF_THR_DELAY", 0), ("TKOFF_ACCEL_CNT", 2)):
            invalid = dict(values)
            invalid[key] = value
            self.assertTrue(checks.native_takeoff_blockers(invalid))
        self.assertTrue(checks.native_takeoff_blockers({}))

    def test_full_autoland_requires_explicit_verified_control_and_supervisor(self):
        snapshot = self.snapshot()
        self.assertTrue(checks.launch_blockers(snapshot, 100, full_autoland=True))
        snapshot.update(live_output=True, handoff_enabled=True,
                        partial_attitude_verified=True, supervisor_state="MISSION_READY",
                        supervisor_fresh=True, supervisor_enabled=True)
        self.assertEqual(checks.launch_blockers(snapshot, 100, full_autoland=True), [])
        # Selecting the full experiment must not weaken any existing launch guard.
        for key, value in (("armed", False), ("current_command", 16),
                           ("live_output", False), ("handoff_enabled", False),
                           ("partial_attitude_verified", False),
                           ("supervisor_state", "WAITING"), ("supervisor_fresh", False),
                           ("supervisor_enabled", False),
                           ("mission_matches", False), ("profile_verified", False)):
            invalid = dict(snapshot)
            invalid[key] = value
            self.assertTrue(checks.launch_blockers(invalid, 100, full_autoland=True), key)
        self.assertTrue(checks.launch_blockers(snapshot, 100))


if __name__ == "__main__":
    unittest.main()
