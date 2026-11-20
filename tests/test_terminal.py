"""Terminal policy tests: close-range continuity, authority and contact identity."""
import math
import unittest
from types import SimpleNamespace as NS

import test_nodes as helpers
ros = helpers.ros
from autoland.terminal import TerminalConfig, commit_ready, net_contact_pair
from autoland.vision_guidance import GuidanceConfig, VisionGuidance
from autoland.state_conditions import Supervisor


class TerminalPolicyTests(unittest.TestCase):
    def test_commit_rejects_uncertainty_age_and_missed_opening(self):
        c = TerminalConfig()
        c.validate()
        self.assertTrue(commit_ready(c, 1.5, .1, .1, .75, .75, .1))
        for args in ((1.5, .1, .2, .75, .75, .1),
                     (1., .1, .1, .75, .75, .1),
                     (.9, .1, .1, .75, .75, .1),
                     (1.5, .1, .1, .75, .75, .3),
                     (1.5, 3., .1, .75, .75, .1),
                     (1.5, .1, .1, -1., .75, .1)):
            self.assertFalse(commit_ready(c, *args))

    def test_close_guidance_is_finite_bounded_and_reduces_thrust(self):
        g = VisionGuidance(GuidanceConfig())
        with self.assertRaisesRegex(ValueError, "range"):
            g.update((1., .1, .1), (15., 0., 0.), 15., 0., 0., .05)
        previous = None
        for x in (20., 10., 3., 1., .1, .001):
            command, _, _ = g.update((x, .1, .1), (15., 0., 0.), 15., 0., 0., .05, terminal=True)
            self.assertTrue(all(math.isfinite(v) for v in command))
            self.assertLessEqual(abs(command[0]), math.radians(5))
            self.assertTrue(math.radians(-8) <= command[1] <= math.radians(12))
            if previous:
                self.assertLessEqual(command[2], previous[2])
                self.assertLessEqual(abs(command[1]-previous[1]), math.radians(12)*.05+1e-12)
            previous = command

    def test_contact_excludes_ground_decorations_and_other_models(self):
        panel = "among_us_landing_panel::assembly::blue_panel_collision"
        self.assertTrue(net_contact_pair(panel, "merlinv2::body::collision"))
        self.assertFalse(net_contact_pair(panel, "contact_probe::body::collision"))
        self.assertFalse(net_contact_pair("ground::collision", "merlinv2::body::collision"))
        self.assertFalse(net_contact_pair(panel.replace("blue_panel", "red_body"), "merlinv2::body::collision"))

    def test_commit_changes_fault_policy_without_claiming_complete(self):
        s = Supervisor()
        self.assertFalse(s.commit())
        s.state = "VISION_FINAL"
        self.assertTrue(s.commit())
        s.abort("Telemetry lost")
        self.assertEqual(s.state, "TERMINAL_FAULT")


class TerminalNodeTests(unittest.TestCase):
    setUp = helpers.NodeTests.setUp
    stamp = helpers.NodeTests.stamp
    state = helpers.NodeTests.state
    status = helpers.NodeTests.status
    target = helpers.NodeTests.target
    interface = helpers.NodeTests.interface
    supervisor_on_land = helpers.NodeTests.supervisor_on_land
    good_vision = helpers.NodeTests.good_vision

    def terminal_supervisor(self):
        n = self.supervisor_on_land()
        n.p["terminal_enabled"] = True
        n.engine.state = "VISION_FINAL"
        n.save("fcu", self.state("GUIDED", True))
        n.owner_cb(ros.String(data="VISION"))
        self.good_vision(n)
        n.preview.terminal_ready = True
        n.preview.time_to_target_s = 1.5
        n.preview.position_std_y_m = n.preview.position_std_z_m = .75
        return n

    def test_commit_requires_continuously_acceptable_crossing(self):
        n = self.terminal_supervisor()
        n.tick()
        self.assertEqual(n.engine.state, "VISION_FINAL")
        ros.clock.value += .1
        n.preview.terminal_ready = False
        n.tick()
        self.assertIsNone(n.commit_since)
        n.preview.terminal_ready = True
        n.tick()
        ros.clock.value += .16
        self.good_vision(n)
        n.preview.terminal_ready = True
        n.preview.time_to_target_s = 1.3
        n.preview.position_std_y_m = n.preview.position_std_z_m = .75
        n.tick()
        self.assertEqual(n.engine.state, "TERMINAL_COMMITTED")

    def test_failed_gate_at_decision_boundary_recovers_before_commitment(self):
        n = self.terminal_supervisor()
        n.preview.terminal_ready = False
        n.preview.time_to_target_s = .9
        n.tick()
        self.assertEqual(n.engine.state, "ABORT")
        self.assertEqual(n.client.calls[-1][0].owner, "ABORT")

    def test_terminal_watchdog_cuts_thrust_without_late_rtl_and_pilot_can_override(self):
        n = self.interface(True)
        self.assertFalse(n.p["terminal_enabled"])
        n.p["terminal_enabled"] = True
        n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        n.state_cb(self.state("GUIDED", True))
        n.tick()
        calls = len(n.mode_client.calls)
        n.status_cb(self.status(n, "TERMINAL_COMMITTED"))
        n.command.terminal_mode = True
        n.tick()
        ros.clock.value += .25
        n.status_cb(self.status(n, "TERMINAL_COMMITTED"))
        n.tick()
        self.assertTrue(n.terminal_fault)
        self.assertEqual(n.pub.messages[-1].thrust, 0.)
        self.assertEqual(len(n.mode_client.calls), calls)
        result = n.owner_cb(NS(owner="ABORT", plan_id="abc"), NS())
        self.assertTrue(result.accepted)
        self.assertIn("zero-thrust", result.message)
        n.state_cb(self.state("MANUAL", True))
        n.tick()
        self.assertEqual(n.owner, "NONE")

    def test_zero_thrust_requires_terminal_authority(self):
        n = self.interface(True)
        n.command.thrust = 0.
        n.command.terminal_mode = True
        self.assertFalse(n.fresh_command())
        n.p["terminal_enabled"] = True
        n.terminal_latched = True
        n.status_cb(self.status(n, "TERMINAL_COMMITTED"))
        self.assertTrue(n.fresh_command())

    def test_sim_contact_does_not_confirm_capture_and_ground_capture_is_rejected(self):
        n = self.supervisor_on_land()
        n.p["terminal_enabled"] = n.p["simulation_contact_enabled"] = True
        n.engine.state = "VISION_FINAL"
        n.engine.commit()
        n.save("fcu", self.state("GUIDED", True))
        message = self.stamp(n, ros.LandingStatus())
        message.contacts = [NS(collision1=NS(name="among_us_landing_panel::assembly::blue_panel_collision"),
                               collision2=NS(name="merlinv2::body::collision"))]
        n.contact_cb(message)
        self.assertEqual(n.engine.state, "NET_CONTACT")
        self.assertTrue(n.net_contact)
        n.velocity.twist.linear.x = n.velocity.twist.linear.y = 0.
        n.alt_cb(ros.Float64(data=0.))
        self.assertFalse(n.capture_cb(None, NS()).success)
        n.alt_cb(ros.Float64(data=1.35))
        self.assertTrue(n.capture_cb(None, NS()).success)
