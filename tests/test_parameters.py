import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/autoland"))
import ros_stubs as ros
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterEvent
from autoland.parameter_profile import (
    load_profile,
    selected_names,
    typed_target,
    decode_value,
    validate_effective,
    equal_value,
    profile_id,
)
from autoland.parameter_manager import ParameterManager
from autoland.mission_manager import MissionManager
from autoland.landing_supervisor import LandingSupervisor
from autoland.fcu_interface import FCUInterface

BASELINE = dict(
    LAND_TYPE=0,
    LAND_FLARE_ALT=3.0,
    LAND_FLARE_SEC=2.0,
    LAND_PF_ALT=10.0,
    LAND_PF_SEC=6.0,
    LAND_PF_ARSPD=0.0,
    TECS_LAND_SINK=0.5,
    LAND_OPTIONS=0,
    AIRSPEED_MIN=10.0,
    AIRSPEED_MAX=25.0,
)
PROFILE = dict(LAND_TYPE=0, LAND_FLARE_ALT=3.0, LAND_FLARE_SEC=2.0, LAND_PF_ARSPD=15.0)


def value(raw):
    if type(raw) is int:
        return ParameterValue(type=2, integer_value=raw)
    return ParameterValue(type=3, double_value=float(raw))


class ProfileTests(unittest.TestCase):
    def test_duplicate_yaml_keys_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.yaml"
            path.write_text(
                "schema_version: 1\nconfigured: true\nparameters:\n  LAND_TYPE: 0\n  LAND_FLARE_ALT: 3.0\n  LAND_FLARE_ALT: 9.0\n  LAND_FLARE_SEC: 2.0\n"
            )
            with self.assertRaises(ValueError):
                load_profile(path)

    def test_discovery_preserves_native_parameters(self):
        p = NS(parameters=PROFILE)
        names = selected_names(p, list(BASELINE) + ["INS_GYRO_FILTER"])
        self.assertIn("TECS_LAND_SINK", names)
        self.assertIn("AIRSPEED_MIN", names)
        self.assertNotIn("INS_GYRO_FILTER", names)

    def test_unknown_parameter_rejected_before_write(self):
        with self.assertRaises(ValueError):
            selected_names(NS(parameters={"LAND_MISTAKE": 1}), list(BASELINE))

    def test_numeric_types_and_integer_precision(self):
        self.assertEqual(typed_target(4.0, 1, 2), 4)
        self.assertEqual(typed_target(None, 7, 2), 7)
        with self.assertRaises(ValueError):
            typed_target(4.5, 1, 2)
        with self.assertRaises(ValueError):
            decode_value(ParameterValue(type=0))
        self.assertFalse(equal_value(4000000, 4000001))
        self.assertTrue(equal_value(0.5, 0.50000001))

    def test_finite_and_native_configuration(self):
        for change in (
            {"LAND_TYPE": 1},
            {"LAND_FLARE_SEC": -1.0},
            {"LAND_PF_ARSPD": 9.0},
            {"LAND_PF_ARSPD": 26.0},
            {"LAND_FLARE_ALT": float("nan")},
        ):
            with self.assertRaises(ValueError):
                validate_effective({**BASELINE, **change})

    def test_yaml_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / "profile.yaml"
            for payload in (
                {},
                dict(schema_version=1, configured=True, parameters={"LAND_TYPE": 0}),
                dict(schema_version=1, configured=True, parameters={**PROFILE, "LAND_BAD": "text"}),
                dict(
                    schema_version=1, configured=True, parameters={**PROFILE, "LAND_OPTIONS": True}
                ),
            ):
                p.write_text(yaml.safe_dump(payload))
                with self.assertRaises(ValueError):
                    load_profile(p)
            p.write_text(
                yaml.safe_dump(
                    dict(
                        schema_version=1,
                        configured=True,
                        parameters={**PROFILE, "LAND_PITCH_DEG": None},
                    )
                )
            )
            self.assertIsNone(load_profile(p).parameters["LAND_PITCH_DEG"])


class ParameterNodeTests(unittest.TestCase):
    def setUp(self):
        ros.Node.overrides = {}
        ros.clock.value = 1000.0
        self.clock = patch("time.monotonic", side_effect=lambda: ros.clock.value)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.file = Path(self.temp.name) / "profile.yaml"
        self.write_profile(PROFILE)

    def write_profile(self, parameters, configured=True):
        self.file.write_text(
            yaml.safe_dump(dict(schema_version=1, configured=configured, parameters=parameters))
        )

    def manager(self, writes=False, baseline=None):
        ros.Node.overrides = {
            "parameter_manager": dict(
                profile_path=str(self.file),
                parameter_write_enabled=writes,
                apply_on_start=False,
                backup_directory=str(Path(self.temp.name) / "backups"),
            )
        }
        node = ParameterManager()
        node.state_cb(ros.State(connected=True, armed=False, mode="MANUAL"))
        self.native = dict(BASELINE if baseline is None else baseline)
        self.handled = {id(c): 0 for c in (node.pull, node.list, node.get, node.set)}
        return node

    def respond(self, node, clamp=None, reject=False):
        for client in (node.pull, node.list, node.get, node.set):
            count = self.handled[id(client)]
            while len(client.calls) > count:
                request, future = client.calls[count]
                count += 1
                if client is node.pull:
                    future.finish(success=True, param_received=len(self.native))
                elif client is node.list:
                    future.finish(result=NS(names=list(self.native)))
                elif client is node.get:
                    future.finish(
                        values=[
                            value(self.native[k]) if k in self.native else ParameterValue(type=0)
                            for k in request.names
                        ]
                    )
                else:
                    parameter = request.parameters[0]
                    if not reject:
                        actual, _ = decode_value(parameter.value)
                        self.native[parameter.name] = (
                            clamp(parameter.name, actual) if clamp else actual
                        )
                    future.finish(
                        results=[
                            NS(successful=not reject, reason="test rejection" if reject else "")
                        ]
                    )
            self.handled[id(client)] = count

    def drive(self, node, **kw):
        for _ in range(80):
            node.tick()
            self.respond(node, **kw)
            if node.phase == "IDLE" and node.status.state in ("READY", "FAILED", "PREVIEW"):
                return
        self.fail("Parameter transaction did not finish")

    def test_preview_reports_changes_without_writes_or_backup(self):
        node = self.manager()
        self.assertTrue(node.sync_cb(None, NS()).success)
        self.drive(node)
        self.assertEqual(node.status.state, "PREVIEW")
        self.assertFalse(node.status.verified)
        self.assertFalse(node.set.calls)
        self.assertEqual(node.backup_path, "")

    def test_apply_then_forced_fcu_readback(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        self.assertTrue(node.status.verified)
        self.assertEqual(self.native["LAND_PF_ARSPD"], 15.0)
        self.assertEqual(len(node.set.calls), 1)
        self.assertEqual(len(node.pull.calls), 2)
        self.assertTrue(all(req.force_pull for req, _ in node.pull.calls))
        saved = yaml.safe_load(Path(node.backup_path).read_text())
        self.assertEqual(saved["parameters"]["LAND_PF_ARSPD"], 0.0)
        self.assertEqual(saved["parameters"]["TECS_LAND_SINK"], 0.5)
        verified = json.loads(node.status.parameters_json)
        self.assertEqual(verified["TECS_LAND_SINK"], 0.5)
        self.assertEqual(node.status.profile_id, profile_id(verified))

    def test_already_matching_profile_ready_without_write_permission(self):
        node = self.manager(False, {**BASELINE, "LAND_PF_ARSPD": 15.0})
        node.sync_cb(None, NS())
        self.drive(node)
        self.assertTrue(node.status.verified)
        self.assertFalse(node.set.calls)

    def test_preserved_null_is_not_overwritten(self):
        self.write_profile({**PROFILE, "LAND_PF_ARSPD": None})
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        self.assertTrue(node.status.verified)
        self.assertEqual(self.native["LAND_PF_ARSPD"], 0.0)
        self.assertFalse(node.set.calls)

    def test_unknown_parameter_aborts_before_any_write(self):
        self.write_profile({**PROFILE, "LAND_MISTAKE": 1})
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        self.assertEqual(node.status.state, "FAILED")
        self.assertFalse(node.set.calls)

    def test_unsafe_preflare_speed_aborts_before_write(self):
        self.write_profile({**PROFILE, "LAND_PF_ARSPD": 8.0})
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        self.assertFalse(node.status.verified)
        self.assertFalse(node.set.calls)

    def test_integer_fcu_parameter_uses_integer_wire_type(self):
        self.write_profile({**PROFILE, "LAND_OPTIONS": 1})
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        request = next(req for req, _ in node.set.calls if req.parameters[0].name == "LAND_OPTIONS")
        self.assertEqual(request.parameters[0].value.type, 2)
        self.assertEqual(request.parameters[0].value.integer_value, 1)

    def test_clamped_readback_blocks_readiness(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node, clamp=lambda n, v: 14.0 if n == "LAND_PF_ARSPD" else v)
        self.assertEqual(node.status.state, "FAILED")
        self.assertIn("mismatch", node.status.reason)

    def test_rejected_write_leaves_backup_and_blocked_state(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node, reject=True)
        self.assertEqual(node.status.state, "FAILED")
        self.assertTrue(Path(node.backup_path).exists())

    def test_no_sync_while_armed(self):
        node = self.manager(True)
        node.state_cb(ros.State(connected=True, armed=True, mode="AUTO"))
        self.assertFalse(node.sync_cb(None, NS()).success)
        self.assertFalse(node.set.calls)

    def test_arm_during_transaction_stops_further_writes(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        node.tick()
        node.state_cb(ros.State(connected=True, armed=True, mode="AUTO"))
        node.tick()
        self.assertEqual(node.status.state, "FAILED")
        self.assertFalse(node.set.calls)

    def test_late_rpc_prevents_overlapping_retry(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        node.tick()
        ros.clock.value += 70.0
        node.state_cb(ros.State(connected=True, armed=False, mode="MANUAL"))
        node.tick()
        self.assertEqual(node.status.state, "FAILED")
        self.assertFalse(node.sync_cb(None, NS()).success)
        node.pull.calls[-1][1].finish(success=True, param_received=len(BASELINE))
        node.tick()
        self.assertTrue(node.sync_cb(None, NS()).success)

    def test_verified_profile_monitors_while_armed_without_writes(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        before = len(node.set.calls)
        ros.clock.value += 2.1
        node.state_cb(ros.State(connected=True, armed=True, mode="AUTO"))
        node.tick()
        self.respond(node)
        node.tick()
        self.assertTrue(node.status.verified)
        self.assertEqual(len(node.set.calls), before)

    def test_external_parameter_event_invalidates_profile(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        parameter = Parameter(name="LAND_FLARE_ALT", value=value(9.0))
        node.event_cb(ParameterEvent(node="/mavros/param", changed_parameters=[parameter]))
        self.assertFalse(node.status.verified)
        self.assertIn("changed", node.status.reason)

    def test_cached_readback_drift_invalidates_profile(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        self.native["LAND_FLARE_ALT"] = 9.0
        ros.clock.value += 2.1
        node.state_cb(ros.State(connected=True, armed=True, mode="AUTO"))
        node.tick()
        self.respond(node)
        node.tick()
        self.assertFalse(node.status.verified)

    def test_cache_refresh_revokes_then_rechecks_without_writes(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        before = len(node.set.calls)
        node.event_cb(ParameterEvent(node="/mavros/param", deleted_parameters=[
            Parameter(name="LAND_FLARE_ALT")]))
        self.assertFalse(node.status.verified)
        self.assertEqual(node.phase, "CACHE_RECHECK_REQUEST")
        self.drive(node)
        self.assertTrue(node.status.verified)
        self.assertEqual(len(node.set.calls), before)

    def test_cache_refresh_rejects_actual_missing_parameter(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        del self.native["LAND_FLARE_ALT"]
        node.event_cb(ParameterEvent(node="/mavros/param", deleted_parameters=[
            Parameter(name="LAND_FLARE_ALT")]))
        self.drive(node)
        self.assertFalse(node.status.verified)
        self.assertEqual(node.status.state, "FAILED")

    def test_cache_refresh_rejects_changed_parameter(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        self.native["LAND_FLARE_ALT"] = 9.0
        node.event_cb(ParameterEvent(node="/mavros/param", deleted_parameters=[
            Parameter(name="LAND_FLARE_ALT")]))
        self.drive(node)
        self.assertFalse(node.status.verified)
        self.assertIn("mismatch", node.status.reason)

    def test_cache_deletion_in_flight_requires_preflight_sync(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        node.state_cb(ros.State(connected=True, armed=True, mode="AUTO"))
        node.event_cb(ParameterEvent(node="/mavros/param", deleted_parameters=[
            Parameter(name="LAND_FLARE_ALT")]))
        self.assertFalse(node.status.verified)
        self.assertEqual(node.phase, "IDLE")
        self.assertIn("preflight sync required", node.status.reason)

    def test_changed_event_during_cache_recheck_cannot_restore_readiness(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        node.event_cb(ParameterEvent(node="/mavros/param", deleted_parameters=[
            Parameter(name="LAND_FLARE_ALT")]))
        node.tick()  # Authoritative get is now in flight.
        node.event_cb(ParameterEvent(node="/mavros/param", changed_parameters=[
            Parameter(name="LAND_FLARE_ALT", value=value(9.0))]))
        self.respond(node)
        node.tick()
        self.assertFalse(node.status.verified)
        self.assertEqual(node.status.state, "FAILED")

    def test_export_creates_full_editable_yaml(self):
        node = self.manager(True)
        node.sync_cb(None, NS())
        self.drive(node)
        response = node.export_cb(None, NS())
        self.assertTrue(response.success)
        exported = load_profile(response.message)
        self.assertFalse(exported.configured)
        self.assertEqual(exported.parameters["TECS_LAND_SINK"], 0.5)
        self.assertEqual(exported.parameters["LAND_PF_ARSPD"], 15.0)

    def test_profile_must_be_reviewed(self):
        self.write_profile(PROFILE, False)
        node = self.manager(True)
        self.assertFalse(node.sync_cb(None, NS()).success)
        self.assertFalse(node.pull.calls)


class InterlockTests(unittest.TestCase):
    def setUp(self):
        ros.Node.overrides = {}
        ros.clock.value = 1000.0
        self.clock = patch("time.monotonic", side_effect=lambda: ros.clock.value)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def test_mission_prepare_requires_parameter_profile(self):
        ros.Node.overrides = {
            "mission_manager": dict(configured=True, land_lat=0.0, land_lon=0.0, approach_alt=40.0)
        }
        node = MissionManager()
        node.state_cb(ros.State(connected=True, armed=False, mode="MANUAL"))
        self.assertFalse(node.prepare_cb(None, NS()).success)
        self.assertFalse(node.pull.calls)

    def test_verified_fcu_flare_thresholds_feed_supervisor(self):
        node = LandingSupervisor()
        node.profile_cb(
            ros.verified_profile(
                node, {"LAND_TYPE": 0, "LAND_FLARE_ALT": 4.0, "LAND_FLARE_SEC": 3.0}
            )
        )
        self.assertEqual(node.config.flare_alt_m, 4.0)
        self.assertEqual(node.config.flare_time_s, 3.0)
        self.assertEqual(node.config.handoff_margin_m, 1.5)  # Early handoff remains unchanged.

    def test_wrong_profile_blocks_mission_enable(self):
        node = LandingSupervisor()
        profile = ros.verified_profile(node)
        node.profile_cb(profile)
        plan = ros.LandingPlan(verified=True, plan_id="abc", parameter_profile_id="wrong")
        node.plan_cb(plan)
        self.assertFalse(node.verified())

    def test_no_vision_ownership_without_parameter_lease(self):
        node = FCUInterface()
        self.assertFalse(node.owner_cb(NS(owner="VISION", plan_id="abc"), NS()).accepted)

    def test_expired_parameter_lease_invalidates_mission(self):
        node = LandingSupervisor()
        profile = ros.verified_profile(node)
        node.profile_cb(profile)
        plan = ros.LandingPlan(
            verified=True, plan_id="abc", parameter_profile_id=profile.profile_id
        )
        node.plan_cb(plan)
        self.assertTrue(node.verified())
        ros.clock.value += 3.1
        self.assertFalse(node.verified())


if __name__ == "__main__":
    unittest.main()
