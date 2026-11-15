import copy
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
        air = ros.VFR_HUD(airspeed=15.0)
        n.save("air", air)
        net = self.stamp(n, ros.NetEstimate(valid=True, score=1.0), "base_link")
        net.pose.position.x = 30.0
        n.save("net", net)
        n.tick()
        self.assertTrue(n.pub.messages[-1].valid)
        ros.clock.value += 0.4
        n.save("status", self.status(n, "AUTO_APPROACH"))
        n.tick()
        self.assertFalse(n.pub.messages[-1].valid)

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
        n.enable_cb(None, NS())
        n.tick()
        self.assertEqual(n.engine.state, "MISSION_READY")
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
