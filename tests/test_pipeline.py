"""Whole control handoff with ROS doubles; no transport or real vehicle involved."""

from pathlib import Path
import math
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
for package in ("autoland", "perception"):
    sys.path.insert(0, str(ROOT / "src" / package))
import ros_stubs as ros
from autoland.landing_supervisor import LandingSupervisor
from autoland.fcu_interface import FCUInterface
from autoland.vision_controller import VisionController
from autoland.geometry import euler_quaternion


class PipelineTests(unittest.TestCase):
    def test_auto_to_guided_holds_target_through_loss_reacquires_then_recovers_on_bad_command(self):
        ros.clock.value = 1000.0
        ros.Node.overrides = {
            "landing_supervisor": dict(handoff_enabled=True, vision_stable_s=0.2),
            "fcu_interface": dict(live_output=True, partial_attitude_verified=True),
        }
        with patch("time.monotonic", side_effect=lambda: ros.clock.value):
            supervisor = LandingSupervisor()
            guidance = VisionController()
            bridge = FCUInterface()
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
            mode = "AUTO"
            handled_requests = 0

            def stamp(node, m, frame="map"):
                m.header.stamp = node.get_clock().now().to_msg()
                m.header.frame_id = frame
                return m

            def feed(vision):
                state = ros.State(connected=True, armed=True, mode=mode)
                profile = ros.verified_profile(supervisor)
                plan.parameter_profile_id = profile.profile_id
                supervisor.profile_cb(profile)
                bridge.profile_cb(profile)
                supervisor.plan_cb(plan)
                supervisor.save("fcu", state)
                bridge.state_cb(state)
                gps = stamp(supervisor, ros.NavSatFix(latitude=-0.0006, longitude=0.0))
                vel = stamp(supervisor, ros.TwistStamped())
                vel.twist.linear.y = 15.0
                vel.twist.linear.z = -1.0
                supervisor.save("gps", gps)
                supervisor.save("velocity", vel)
                guidance.save("velocity", vel)
                supervisor.alt_cb(ros.Float64(data=10.0 if ros.clock.value < 1000.3 else 5.0))
                supervisor.mission_cb(ros.WaypointList(current_seq=17))
                pose = stamp(supervisor, ros.PoseStamped())
                q = euler_quaternion(0, 0, math.pi / 2)
                (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ) = q
                guidance.save("pose", pose)
                bridge.pose_cb(pose)
                guidance.save("air", ros.VfrHud(airspeed=15.0))
                if vision:
                    net = stamp(supervisor, ros.NetEstimate(valid=True, score=1.0, plan_id="abc"), "base_link")
                    net.pose.position.x = 30.0
                    supervisor.save("net", net)
                    guidance.save("net", net)
                else:
                    net = stamp(supervisor, ros.NetEstimate(valid=False), "base_link")
                    supervisor.save("net", net)
                    guidance.save("net", net)

            feed(True)
            self.assertTrue(supervisor.enable_cb(None, NS()).success)
            for step in range(18):
                ros.clock.value = 1000 + step * 0.05
                feed(True)
                supervisor.tick()
                status = supervisor.pub.messages[-1]
                guidance.save("status", status)
                bridge.status_cb(status)
                guidance.tick()
                target = guidance.pub.messages[-1]
                supervisor.save("preview", target)
                bridge.command_cb(target)
                if len(supervisor.client.calls) > handled_requests:
                    request, future = supervisor.client.calls[-1]
                    response = bridge.owner_cb(request, NS())
                    future.finish(accepted=response.accepted, message=response.message)
                    handled_requests = len(supervisor.client.calls)
                    if bridge.requested_mode == "GUIDED":
                        mode = "GUIDED"
                        bridge.state_cb(ros.State(connected=True, armed=True, mode=mode))
                bridge.tick()
                supervisor.owner_cb(bridge.owner_pub.messages[-1])
            self.assertEqual(supervisor.engine.state, "VISION_FINAL")
            self.assertTrue(bridge.pub.messages)
            self.assertEqual(bridge.mode_client.calls[0][0].custom_mode, "GUIDED")
            for step in range(10):
                ros.clock.value += 0.05
                feed(False)
                supervisor.tick()
                status = supervisor.pub.messages[-1]
                guidance.save("status", status)
                bridge.status_cb(status)
                guidance.tick()
                target = guidance.pub.messages[-1]
                supervisor.save("preview", target)
                bridge.command_cb(target)
                bridge.tick()
                supervisor.owner_cb(bridge.owner_pub.messages[-1])
            self.assertEqual(supervisor.engine.state, "VISION_FINAL")
            self.assertTrue(target.valid)
            self.assertTrue(target.target_held)
            self.assertGreater(target.target_age_s, .4)
            self.assertEqual(len(bridge.mode_client.calls), 1)
            ros.clock.value += .05
            feed(True)
            guidance.save("status", stamp(supervisor, supervisor.pub.messages[-1]))
            guidance.tick()
            self.assertTrue(guidance.pub.messages[-1].valid)
            self.assertFalse(guidance.pub.messages[-1].target_held)
            # Holding a target cannot bypass independent command/telemetry guards.
            ros.clock.value += .05
            feed(False)
            guidance.save("air", ros.VfrHud(airspeed=0.))
            guidance.tick()
            supervisor.save("preview", guidance.pub.messages[-1])
            bridge.command_cb(guidance.pub.messages[-1])
            supervisor.tick()
            bridge.tick()
            self.assertEqual(supervisor.engine.state, "ABORT")
            self.assertEqual(bridge.mode_client.calls[-1][0].custom_mode, "RTL")


if __name__ == "__main__":
    unittest.main()
