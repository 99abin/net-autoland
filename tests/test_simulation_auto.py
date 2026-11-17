"""Autonomous simulator actuation must require an explicit execution request."""
from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import simulation_auto


class AutonomousAuthorizationTests(unittest.TestCase):
    def test_only_normal_observation_expiry_without_handoff_preserves_native_auto(self):
        self.assertTrue(simulation_auto.preserve_native_fallback(
            "FLIGHT_RUNNING", "AUTO", {"MISSION_READY", "AUTO_APPROACH"}))
        for outcome, mode, phases in (
            ("ERROR: stale", "AUTO", set()), ("INTERRUPTED", "AUTO", set()),
            ("FLIGHT_RUNNING", "GUIDED", set()),
            ("FLIGHT_RUNNING", "AUTO", {"HANDOFF_PENDING"}),
            ("FLIGHT_RUNNING", "AUTO", {"VISION_FINAL"}),
            ("FLIGHT_RUNNING", "AUTO", {"ABORT"}),
        ):
            self.assertFalse(simulation_auto.preserve_native_fallback(outcome, mode, phases))

    def test_default_cannot_connect_write_or_arm(self):
        with patch.object(simulation_auto, "managed_session") as session, patch.object(
                simulation_auto, "FlightObserver") as observer, redirect_stdout(io.StringIO()):
            simulation_auto.run({}, NS(execute=False))
        session.assert_not_called()
        observer.assert_not_called()

    def test_execute_refuses_without_a_verified_local_session(self):
        with patch.object(simulation_auto, "managed_session", side_effect=RuntimeError("no local SITL")), patch.object(
                simulation_auto, "FlightObserver") as observer:
            with self.assertRaisesRegex(RuntimeError, "no local SITL"):
                simulation_auto.run({}, NS(execute=True, duration=300))
        observer.assert_not_called()
