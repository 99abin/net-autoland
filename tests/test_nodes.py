import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
for package in ("autoland", "perception"):
    sys.path.insert(0, str(ROOT / "src" / package))
import ros_stubs as ros

ros.install()
from autoland.mission_manager import MissionManager
from autoland.landing_supervisor import LandingSupervisor
from autoland.fcu_interface import FCUInterface, ARDUPLANE_ROLL_PITCH_THRUST_MASK
from autoland.vision_controller import VisionController
from autoland.mission_builder import Item


class NodeTests(unittest.TestCase):
    def test_plan_invalidation_preserves_the_original_abort_without_repeat_requests(self):
        n = LandingSupervisor()
        n.plan_locked = "locked"
        n.engine.abort("Mission or telemetry unavailable")
        n.plan_cb(ros.LandingPlan(verified=False, reason="FCU heartbeat unavailable"))
        self.assertEqual(n.engine.reason, "Mission or telemetry unavailable")
        self.assertEqual(n.client.calls, [])

    def test_unverified_plan_reports_its_readiness_failure(self):
        n = LandingSupervisor()
        n.plan_locked = "locked"
        n.plan_cb(ros.LandingPlan(verified=False, reason="FCU heartbeat unavailable"))
        self.assertEqual(n.engine.reason, "Landing plan invalidated: FCU heartbeat unavailable")

    def setUp(self):
        ros.Node.overrides = {}
        ros.clock.value = 1000.0
        self.timer = patch("time.monotonic", side_effect=lambda: ros.clock.value)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def stamp(self, node, msg, frame=None):
        msg.header.stamp = node.get_clock().now().to_msg()
        if frame is not None:
            msg.header.frame_id = frame
        return msg

    def state(self, mode="MANUAL", armed=False):
        return ros.State(connected=True, armed=armed, mode=mode)

    def manager(self, upload=False):
        ros.Node.overrides = {
            "mission_manager": dict(
                configured=True,
                upload_enabled=upload,
                prepare_on_start=False,
                land_lat=-35.36,
                land_lon=149.16,
                land_alt=1.35,
                approach_heading_deg=0.0,
                approach_distance_m=300.0,
                approach_alt=40.0,
            )
        }
        m = MissionManager()
        m.state_cb(self.state())
        m.profile_cb(ros.verified_profile(m))
        return m

    def pull(self, m, items):
        m.tick()
        m.list_cb(ros.WaypointList(waypoints=items))
        m.pull.calls[-1][1].finish(success=True, wp_received=len(items))
        m.tick()

    def test_preview_mission_no_upload(self):
        m = self.manager()
        m.prepare_cb(None, NS())
        self.pull(m, [Item() for _ in range(16)])
        self.assertFalse(m.plan.verified)
        self.assertEqual(m.plan.item_count, 18)
        self.assertFalse(m.push.calls)

    def test_full_upload_and_verified_readback(self):
        m = self.manager(True)
        m.prepare_cb(None, NS())
        self.pull(m, [Item() for _ in range(16)])
        m.tick()
        request, f = m.push.calls[-1]
        self.assertEqual(request.start_index, 0)
        self.assertEqual(len(request.waypoints), 18)
        f.finish(success=True, wp_transfered=18)
        m.tick()
        self.pull(m, request.waypoints)
        self.assertTrue(m.plan.verified)
        self.assertEqual(m.plan.land_index, 17)

    def test_pull_response_before_topic_supported(self):
        m = self.manager()
        m.prepare_cb(None, NS())
        m.tick()
        m.pull.calls[-1][1].finish(success=True, wp_received=16)
        m.tick()
        self.assertEqual(m.phase, "PULL_WAIT")
        m.list_cb(ros.WaypointList(waypoints=[Item() for _ in range(16)]))
        m.tick()
        self.assertEqual(m.plan.item_count, 18)

    def test_mismatched_readback_rejected(self):
        m = self.manager(True)
        m.prepare_cb(None, NS())
        self.pull(m, [Item()])
        m.tick()
        req, f = m.push.calls[-1]
        f.finish(success=True, wp_transfered=3)
        m.tick()
        changed = copy.deepcopy(req.waypoints)
        changed[-1].z_alt += 5
        self.pull(m, changed)
        self.assertFalse(m.plan.verified)

    def test_no_mission_upload_while_armed(self):
        m = self.manager(True)
        m.state_cb(self.state("AUTO", True))
        result = m.prepare_cb(None, NS())
        self.assertFalse(result.success)
        self.assertFalse(m.push.calls)

    def test_mission_change_invalidates_plan(self):
        m = self.manager()
        m.prepare_cb(None, NS())
        self.pull(m, [Item()])
        m.list_cb(ros.WaypointList(waypoints=[Item()]))
        self.assertFalse(m.plan.verified)

    def target(self, node):
        return self.stamp(
            node,
            ros.ControlSetpoint(
                valid=True, plan_id="abc", roll_rad=0.0, pitch_rad=0.0, yaw_enu_rad=0.0, thrust=0.4
            ),
            "map",
        )

    def status(self, node, state="HANDOFF_PENDING"):
        return self.stamp(
            node,
            ros.LandingStatus(
                state=state,
                plan_id="abc",
                parameter_profile_id=ros.verified_profile(node).profile_id,
            ),
        )

    def interface(self, live=False):
        ros.Node.overrides = {
            "fcu_interface": dict(live_output=live, partial_attitude_verified=live)
        }
        node = FCUInterface()
        node.profile_cb(ros.verified_profile(node))
        node.state_cb(self.state("AUTO", True))
        node.status_cb(self.status(node))
        node.command_cb(self.target(node))
        pose = self.stamp(node, ros.PoseStamped(), "map")
        pose.pose.orientation.w = 1.0
        node.pose_cb(pose)
        return node

    def test_preview_never_sends_flight_commands(self):
        n = self.interface()
        result = n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        n.tick()
        self.assertFalse(result.accepted)
        self.assertFalse(n.pub.messages)
        self.assertFalse(n.mode_client.calls)
        self.assertTrue(n.preview_pub.messages)

    def test_authorized_abort_can_recover_before_guided_handoff(self):
        n = self.interface(True)
        n.status_cb(self.status(n, "AUTO_APPROACH"))
        result = n.owner_cb(NS(owner="ABORT", plan_id="abc"), NS())
        self.assertTrue(result.accepted)
        self.assertEqual(n.mode_client.calls[-1][0].custom_mode, "RTL")

    def test_initial_attitude_jump_rejected(self):
        n = self.interface(True)
        n.command.roll_rad = 0.3
        result = n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        self.assertFalse(result.accepted)
        self.assertFalse(n.mode_client.calls)

    def test_preview_diagnostic_identifies_future_stamp_without_accepting_it(self):
        n = self.interface(True)
        n.command.header.stamp.nanosec += 10000000
        detail = n.preview_diagnostic("abc")
        self.assertAlmostEqual(detail["stamp_age_s"], -0.01)
        self.assertTrue(detail["valid"])
        self.assertTrue(detail["envelope_ok"])
        self.assertEqual(detail["command_plan_id"], "abc")
        self.assertFalse(n.owner_cb(NS(owner="VISION", plan_id="abc"), NS()).accepted)
        self.assertFalse(n.mode_client.calls)

    def test_recovery_does_not_override_manual_mode(self):
        n = self.interface(True)
        n.state_cb(self.state("MANUAL", True))
        n.owner_cb(NS(owner="ABORT", plan_id="abc"), NS())
        self.assertFalse(n.mode_client.calls)

    def test_request_before_pending_status_waits_without_actuation(self):
        n = self.interface(True)
        n.status_cb(self.status(n, "AUTO_APPROACH"))
        response = n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        self.assertTrue(response.accepted)
        n.state_cb(self.state("GUIDED", True))
        n.tick()
        self.assertEqual(n.owner, "PENDING")
        self.assertFalse(n.pub.messages)
        n.status_cb(self.status(n, "HANDOFF_PENDING"))
        n.tick()
        self.assertEqual(n.owner, "VISION")
        self.assertTrue(n.pub.messages)

    def test_mode_sent_not_enough_confirmation(self):
        n = self.interface(True)
        result = n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        self.assertTrue(result.accepted)
        n.mode_client.calls[-1][1].finish(mode_sent=True)
        n.tick()
        self.assertEqual(n.owner, "PENDING")
        self.assertFalse(n.pub.messages)
        n.state_cb(self.state("GUIDED", True))
        n.tick()
        self.assertEqual(n.owner, "VISION")
        self.assertTrue(n.pub.messages)

    def test_command_watchdog_requests_recovery(self):
        n = self.interface(True)
        n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        n.state_cb(self.state("GUIDED", True))
        n.tick()
        ros.clock.value += 0.25
        n.status_cb(self.status(n, "VISION_FINAL"))
        n.tick()
        self.assertEqual(n.owner, "ABORTING")
        self.assertEqual(n.mode_client.calls[-1][0].custom_mode, "RTL")

    def test_pilot_override_does_not_get_overwritten(self):
        n = self.interface(True)
        n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        n.state_cb(self.state("GUIDED", True))
        n.tick()
        before = len(n.mode_client.calls)
        n.state_cb(self.state("MANUAL", True))
        n.tick()
        self.assertEqual(n.owner, "NONE")
        self.assertEqual(len(n.mode_client.calls), before)

    def test_wrong_plan_and_future_command_rejected(self):
        n = self.interface(True)
        n.command.plan_id = "wrong"
        self.assertFalse(n.owner_cb(NS(owner="VISION", plan_id="abc"), NS()).accepted)
        n.command = self.target(n)
        n.command.header.stamp.sec += 1
        self.assertFalse(n.owner_cb(NS(owner="VISION", plan_id="abc"), NS()).accepted)

    def test_partial_mask_selects_roll_pitch_not_yaw(self):
        enabled = ARDUPLANE_ROLL_PITCH_THRUST_MASK ^ 255
        attitude = enabled & 0b10000111
        self.assertTrue(attitude & 1)
        self.assertTrue(attitude & 2)
        self.assertFalse(attitude & 4)
        self.assertFalse(attitude & 128)
        self.assertTrue(enabled & 64)

    def test_stationary_capture_zero_thrust(self):
        n = self.interface(True)
        n.owner_cb(NS(owner="VISION", plan_id="abc"), NS())
        n.state_cb(self.state("GUIDED", True))
        n.tick()
        n.status_cb(self.status(n, "COMPLETE"))
        n.tick()
        self.assertEqual(n.pub.messages[-1].thrust, 0.0)

    def test_vision_controller_preview_with_motion_history(self):
        n = VisionController()
        s = self.status(n, "AUTO_APPROACH")
        n.save("status", s)
        pose = self.stamp(n, ros.PoseStamped(), "map")
        pose.pose.orientation.w = 1.0
        n.save("pose", pose)
        vel = self.stamp(n, ros.TwistStamped(), "map")
        vel.twist.linear.x = 15.0
        n.save("velocity", vel)
        air = ros.VfrHud(airspeed=15.0)
        n.save("air", air)
        net = self.stamp(n, ros.NetEstimate(valid=True, score=1.0, plan_id="abc"), "base_link")
        net.pose.position.x = 30.0
        n.save("net", net)
        n.tick()
        self.assertTrue(n.pub.messages[-1].valid)
        ros.clock.value += 0.4
        n.save("status", self.status(n, "AUTO_APPROACH"))
        n.tick()
        self.assertFalse(n.pub.messages[-1].valid)

    def tracking_controller(self):
        n = VisionController()
        n.save("status", self.status(n, "AUTO_APPROACH"))
        self.controller_telemetry(n)
        net = self.stamp(n, ros.NetEstimate(valid=True, score=1., plan_id="abc"), "base_link")
        net.pose.position.x = 30.
        n.save("net", net)
        n.tick()
        self.assertTrue(n.pub.messages[-1].valid)
        return n

    def controller_telemetry(self, n, x=0.):
        pose = self.stamp(n, ros.PoseStamped(), "map")
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        n.save("pose", pose)
        vel = self.stamp(n, ros.TwistStamped(), "map")
        vel.twist.linear.x = 15.
        n.save("velocity", vel)
        n.save("air", ros.VfrHud(airspeed=15.))

    def test_held_world_target_moves_relative_to_aircraft_and_reacquires(self):
        n = self.tracking_controller()
        for i in range(1, 270):
            ros.clock.value += .05
            n.save("status", self.status(n, "VISION_FINAL"))
            self.controller_telemetry(n, x=3.)
            n.save("net", self.stamp(n, ros.NetEstimate(valid=False), "base_link"))
            n.tick()
            self.assertTrue(n.pub.messages[-1].valid)
        self.assertTrue(n.pub.messages[-1].target_held)
        self.assertGreater(n.pub.messages[-1].target_age_s, 13.)
        # Source image is now outside pose history; its world position survives.
        self.assertEqual(n.target_world, (30., 0., 0.))
        self.assertEqual(n.target(((3., 0., 0.), (0., 0., 0., 1.)))[0], (27., 0., 0.))
        # Turning changes the body vector, not the stored world location.
        turn = (0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4))
        body = n.target(((3., 0., 0.), turn))[0]
        self.assertAlmostEqual(body[0], 0., places=6)
        self.assertAlmostEqual(body[1], -27., places=6)
        ros.clock.value += .05
        n.save("status", self.status(n, "VISION_FINAL"))
        self.controller_telemetry(n, x=3.)
        net = self.stamp(n, ros.NetEstimate(valid=True, score=1., plan_id="abc"), "base_link")
        net.pose.position.x = 25.
        n.save("net", net)
        n.tick()
        self.assertFalse(n.pub.messages[-1].target_held)
        self.assertEqual(n.target_world, (28., 0., 0.))
        self.assertEqual(n.pub.messages[-1].target_age_s, 0.)

    def test_cache_cannot_initiate_handoff_or_survive_a_new_plan(self):
        n = self.tracking_controller()
        ros.clock.value += .05
        self.controller_telemetry(n)
        n.save("net", self.stamp(n, ros.NetEstimate(valid=False), "base_link"))
        n.save("status", self.status(n, "AUTO_APPROACH"))
        n.tick()
        self.assertFalse(n.pub.messages[-1].valid)
        status = self.status(n, "VISION_FINAL")
        status.plan_id = "different"
        n.save("status", status)
        n.tick()
        self.assertFalse(n.pub.messages[-1].valid)
        self.assertIsNone(n.target_world)
        bridge = self.interface(True)
        bridge.command.target_held = True
        self.assertFalse(bridge.owner_cb(NS(owner="VISION", plan_id="abc"), NS()).accepted)
        self.assertEqual(bridge.mode_client.calls, [])

    def test_reset_epoch_clears_held_target_and_wrong_plane_is_rejected(self):
        n = self.tracking_controller()
        ros.clock.value += .05
        self.controller_telemetry(n)
        n.save("status", self.status(n, "VISION_FINAL"))
        reset = self.stamp(n, ros.NetEstimate(valid=False, plan_id="abc", tracking_epoch=1), "base_link")
        n.save("net", reset)
        n.tick()
        self.assertFalse(n.pub.messages[-1].valid)
        self.assertIsNone(n.target_world)
        wrong = self.stamp(n, ros.NetEstimate(valid=True, score=1., plan_id="old", tracking_epoch=1), "base_link")
        wrong.pose.position.x = 30.
        n.save("net", wrong)
        n.tick()
        self.assertFalse(n.pub.messages[-1].valid)
        self.assertIn("different landing plane", n.pub.messages[-1].reason)

    def test_hold_timeout_stale_telemetry_and_clock_reset_remain_invalid(self):
        for problem in ("timeout", "telemetry", "clock"):
            with self.subTest(problem=problem):
                n = self.tracking_controller()
                if problem == "timeout":
                    n.p["target_hold_timeout_s"] = .1
                ros.clock.value += .6 if problem == "telemetry" else .15
                if problem != "telemetry":
                    self.controller_telemetry(n)
                n.save("net", self.stamp(n, ros.NetEstimate(valid=False), "base_link"))
                if problem == "clock":
                    ros.clock.value -= 1.
                n.save("status", self.status(n, "VISION_FINAL"))
                n.tick()
                self.assertFalse(n.pub.messages[-1].valid)
                if problem == "clock":
                    self.assertIsNone(n.target_world)

    def test_bad_or_out_of_order_observations_do_not_replace_held_target(self):
        n = self.tracking_controller()
        for problem in ("covariance", "position", "old", "frame"):
            ros.clock.value += .05
            n.save("status", self.status(n, "VISION_FINAL"))
            self.controller_telemetry(n)
            net = self.stamp(n, ros.NetEstimate(valid=True, score=1., plan_id="abc"), "base_link")
            net.pose.position.x = 60.
            if problem == "covariance":
                net.covariance[0] = 100.
            elif problem == "position":
                net.pose.position.x = float("nan")
            elif problem == "old":
                net.header.stamp.sec -= 1
            else:
                net.header.frame_id = "wrong"
            n.save("net", net)
            n.tick()
            self.assertTrue(n.pub.messages[-1].valid)
            self.assertTrue(n.pub.messages[-1].target_held)
            self.assertEqual(n.target_world, (30., 0., 0.))

    def test_supervisor_accepts_held_command_only_after_authorization(self):
        n = self.supervisor_on_land()
        self.good_vision(n)
        n.tick()
        ros.clock.value += .6
        self.good_vision(n)
        n.preview.target_held = True
        n.preview.target_age_s = 1.
        n.tick()
        self.assertEqual(n.engine.state, "AUTO_APPROACH")
        self.assertEqual(n.client.calls, [])
        n.engine.state = "HANDOFF_PENDING"
        n.net.valid = False
        n.tick()
        self.assertEqual(n.engine.state, "HANDOFF_PENDING")

    def supervisor_on_land(self):
        ros.Node.overrides = {"landing_supervisor": {"handoff_enabled": True}}
        n = LandingSupervisor()
        n.profile_cb(ros.verified_profile(n))
        n.plan_cb(ros.LandingPlan(verified=True, plan_id="abc", land_index=17,
            approach_lat=-.003, approach_lon=0., land_lat=0., land_lon=0.,
            land_alt=1.35, parameter_profile_id=n.profile.profile_id))
        n.save("fcu", self.state("AUTO", True))
        n.save("gps", self.stamp(n, ros.NavSatFix(latitude=-.0006, longitude=0.), "map"))
        velocity = self.stamp(n, ros.TwistStamped(), "map")
        velocity.twist.linear.y = 15.
        velocity.twist.linear.z = -1.
        n.save("velocity", velocity)
        n.alt_cb(ros.Float64(data=5.))
        n.mission_cb(ros.WaypointList(current_seq=17))
        self.assertTrue(n.enable_cb(None, NS()).success)
        return n

    def good_vision(self, n):
        net = self.stamp(n, ros.NetEstimate(valid=True, score=1., plan_id="abc"), "base_link")
        net.pose.position.x = 30.
        n.save("net", net)
        n.save("preview", self.target(n))

    def test_supervisor_keeps_auto_without_vision_then_accepts_later_stable_vision(self):
        n = self.supervisor_on_land()
        n.tick()
        self.assertEqual(n.engine.state, "AUTO_APPROACH")
        self.assertEqual(n.client.calls, [])
        self.good_vision(n)
        n.tick()
        self.assertEqual(n.client.calls, [])
        ros.clock.value += .6
        self.good_vision(n)
        n.tick()
        self.assertEqual(n.engine.state, "HANDOFF_PENDING")
        self.assertEqual(n.client.calls[-1][0].owner, "VISION")

    def test_lost_detection_before_handoff_keeps_auto_and_resets_stability(self):
        n = self.supervisor_on_land()
        self.good_vision(n)
        n.tick()
        ros.clock.value += .2
        n.net.valid = False
        n.tick()
        self.assertEqual(n.engine.state, "AUTO_APPROACH")
        self.assertIsNone(n.stable_since)
        self.assertEqual(n.client.calls, [])
        self.good_vision(n)
        n.tick()
        self.assertEqual(n.client.calls, [])

    def test_lost_vision_after_handoff_request_or_ownership_still_aborts(self):
        for phase in ("HANDOFF_PENDING", "VISION_FINAL"):
            with self.subTest(phase=phase):
                n = self.supervisor_on_land()
                n.engine.state = phase
                if phase == "VISION_FINAL":
                    n.save("fcu", self.state("GUIDED", True))
                    n.owner_cb(ros.String(data="VISION"))
                n.tick()
                self.assertEqual(n.engine.state, "ABORT")
                self.assertEqual(n.client.calls[-1][0].owner, "ABORT")

    def test_supervisor_waits_for_correct_landing_item(self):
        n = LandingSupervisor()
        plan = ros.LandingPlan(
            verified=True,
            plan_id="abc",
            land_index=17,
            approach_lat=-0.003,
            approach_lon=0.0,
            approach_alt=40.0,
            land_lat=0.0,
            land_lon=0.0,
            land_alt=1.35,
        )
        n.profile_cb(ros.verified_profile(n))
        plan.parameter_profile_id = n.profile.profile_id
        n.plan_cb(plan)
        n.save("fcu", self.state("AUTO", True))
        gps = self.stamp(n, ros.NavSatFix(latitude=-0.0006, longitude=0.0), "map")
        n.save("gps", gps)
        vel = self.stamp(n, ros.TwistStamped(), "map")
        vel.twist.linear.y = 15.0
        vel.twist.linear.z = -1.0
        n.save("velocity", vel)
        n.alt_cb(ros.Float64(data=5.0))
        n.mission_cb(ros.WaypointList(current_seq=10))
        n.tick()
        self.assertFalse(n.pub.messages[-1].monitoring_enabled)
        n.enable_cb(None, NS())
        n.tick()
        self.assertEqual(n.engine.state, "MISSION_READY")
        self.assertTrue(n.pub.messages[-1].monitoring_enabled)
        n.mission_cb(ros.WaypointList(current_seq=17))
        n.tick()
        self.assertEqual(n.engine.state, "AUTO_APPROACH")
        # Mission list is retained/event-driven; its age must not time out alone.
        ros.clock.value += 4
        n.save("fcu", self.state("AUTO", True))
        n.save("gps", self.stamp(n, gps))
        n.save("velocity", self.stamp(n, vel))
        n.alt_cb(ros.Float64(data=5.0))
        self.assertTrue(n.telemetry_ok())


if __name__ == "__main__":
    unittest.main()
