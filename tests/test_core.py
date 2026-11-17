import math
from pathlib import Path
import sys
import unittest
from dataclasses import replace

ROOT = Path(__file__).resolve().parents[1]
for package in ("autoland", "perception"):
    sys.path.insert(0, str(ROOT / "src" / package))
from autoland.geometry import (
    destination,
    leg_position,
    euler_quaternion,
    inverse_rotate,
    interpolate_pose,
    compensate_target,
)
from autoland.mission_builder import Item, LandingConfig, extend_mission, same_mission, fingerprint
from autoland.state_conditions import HandoffConfig, Supervisor, handoff_ready
from autoland.vision_guidance import GuidanceConfig, VisionGuidance


class GeometryTests(unittest.TestCase):
    def test_reciprocal_heading(self):
        lat, lon = destination(0, 0, 180, 300)
        progress, cross, bearing, length = leg_position(lat, lon, 0, 0, lat, lon)
        self.assertAlmostEqual(length, 300, places=3)
        self.assertAlmostEqual(bearing, 0, places=6)
        self.assertLess(lat, 0)
        self.assertAlmostEqual(cross, 0)

    def test_east_approach_is_west_of_land(self):
        lat, lon = destination(0, 0, 270, 300)
        self.assertLess(lon, 0)

    def test_longitude_wrap(self):
        _, lon = destination(0, 179.999, 90, 1000)
        self.assertTrue(-180 <= lon <= 180)

    def test_body_conversion_north_facing(self):
        q = euler_quaternion(0, 0, math.pi / 2)
        v = inverse_rotate(q, (0, 15, 0))
        self.assertAlmostEqual(v[0], 15)
        self.assertAlmostEqual(v[1], 0)

    def test_motion_compensation(self):
        q = euler_quaternion(0, 0, 0)
        p = compensate_target((20, 0, 0), ((0, 0, 0), q), ((2, 0, 0), q))
        self.assertEqual(p, (18, 0, 0))

    def test_pose_interpolation_and_staleness(self):
        q = (0, 0, 0, 1)
        p, r = interpolate_pose([(1, (0, 0, 0), q), (2, (2, 0, 0), q)], 1.5)
        self.assertEqual(p, (1, 0, 0))
        with self.assertRaises(ValueError):
            interpolate_pose([(1, (0, 0, 0), q)], 2)


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.c = LandingConfig(-35.36, 149.16, 1.35, 0.0, 300.0, 40.0)
        self.base = [Item(x_lat=-35.36, y_long=149.16, z_alt=40.0) for _ in range(16)]

    def test_append_and_preserve(self):
        mission, already = extend_mission(self.base, self.c)
        self.assertFalse(already)
        self.assertEqual(len(mission), 18)
        self.assertTrue(same_mission(mission[:16], self.base))
        self.assertEqual(len(self.base), 16)
        self.assertEqual(mission[-1].command, 21)
        self.assertFalse(mission[-1].is_current)
        self.assertEqual(mission[-1].frame, 3)

    def test_restart_idempotent(self):
        mission, _ = extend_mission(self.base, self.c)
        again, already = extend_mission(mission, self.c)
        self.assertTrue(already)
        self.assertEqual(len(again), 18)

    def test_existing_other_land_rejected(self):
        with self.assertRaises(ValueError):
            extend_mission(self.base + [Item(command=21)], self.c)

    def test_terminal_loiter_rejected(self):
        for cmd in (17, 20, 177):
            with self.assertRaises(ValueError):
                extend_mission(self.base + [Item(command=cmd)], self.c)

    def test_config_rejection(self):
        for kwargs in (
            dict(land_lat=100),
            dict(approach_distance_m=0),
            dict(approach_alt=1),
            dict(land_alt=math.nan),
        ):
            with self.assertRaises(ValueError):
                extend_mission(self.base, replace(self.c, **kwargs))

    def test_readback_tolerance_and_current_flag(self):
        mission, _ = extend_mission(self.base, self.c)
        changed = [replace(i) for i in mission]
        changed[0].is_current = True
        changed[-1].z_alt += 0.001
        self.assertTrue(same_mission(mission, changed))
        changed[-1].z_alt += 1
        self.assertFalse(same_mission(mission, changed))

    def test_fingerprint_stable_current_flag(self):
        changed = [replace(i, is_current=True) for i in self.base]
        self.assertEqual(fingerprint(self.base), fingerprint(changed))

    def test_arduplane_land_direction_readback_and_restart(self):
        expected, _ = extend_mission(self.base, self.c)
        self.assertEqual(expected[-1].param4, 1.0)
        # Match the observed FCU readback: positive direction and quantized GPS.
        readback = [replace(item) for item in expected]
        readback[-2].x_lat = int(readback[-2].x_lat * 1e7) / 1e7
        readback[-1].z_alt -= 1e-8
        self.assertTrue(same_mission(expected, readback))
        reused, already = extend_mission(readback, self.c)
        self.assertTrue(already)
        self.assertEqual(len(reused), len(expected))

    def test_land_readback_still_rejects_changed_direction(self):
        expected, _ = extend_mission(self.base, self.c)
        changed = [replace(item) for item in expected]
        changed[-1].param4 = -1.0
        self.assertFalse(same_mission(expected, changed))

    def test_gps_tolerance_boundary_survives_binary_roundoff(self):
        expected, _ = extend_mission(self.base, self.c)
        expected[-1].x_lat = -35.3637111
        readback = [replace(item) for item in expected]
        readback[-1].x_lat = -35.3637109
        self.assertTrue(same_mission(expected, readback))
        # The observed LAND readback must be reusable without appending again.
        config = replace(self.c, land_lat=-35.3637111)
        mission, _ = extend_mission(self.base, config)
        mission[-1].x_lat = -35.3637109
        reused, already = extend_mission(mission, config)
        self.assertTrue(already)
        self.assertEqual(len(reused), len(mission))

    def test_gps_outside_existing_tolerance_still_rejected(self):
        expected, _ = extend_mission(self.base, self.c)
        expected[-1].x_lat = -35.3637111
        readback = [replace(item) for item in expected]
        readback[-1].x_lat = -35.363710899
        self.assertFalse(same_mission(expected, readback))


class SupervisorTests(unittest.TestCase):
    def context(self, **kw):
        c = dict(
            verified=True,
            telemetry_ok=True,
            mode="AUTO",
            armed=True,
            landing_active=True,
            ready=False,
            allow_handoff=True,
            owned=False,
        )
        c.update(kw)
        return c

    def test_native_approach_then_confirmed_handoff(self):
        s = Supervisor()
        s.enable(True)
        self.assertIsNone(s.step(**self.context()))
        self.assertEqual(s.state, "AUTO_APPROACH")
        self.assertEqual(s.step(**self.context(ready=True)), "VISION")
        self.assertEqual(s.state, "HANDOFF_PENDING")
        s.step(**self.context(mode="GUIDED", owned=False))
        self.assertEqual(s.state, "HANDOFF_PENDING")
        s.step(**self.context(mode="GUIDED", owned=True))
        self.assertEqual(s.state, "VISION_FINAL")

    def test_missing_or_lost_vision_before_handoff_keeps_native_auto(self):
        s = Supervisor()
        s.enable(True)
        for _ in range(5):
            self.assertIsNone(s.step(**self.context(handoff_window=True)))
            self.assertEqual(s.state, "AUTO_APPROACH")
            self.assertTrue(s.enabled)
        self.assertIn("staying in AUTO", s.reason)
        self.assertEqual(s.step(**self.context(handoff_window=True, ready=True)), "VISION")
        self.assertEqual(s.state, "HANDOFF_PENDING")

    def test_preview_never_requests_mode(self):
        s = Supervisor()
        s.enable(True)
        self.assertIsNone(s.step(**self.context(ready=True, allow_handoff=False)))
        self.assertEqual(s.state, "AUTO_APPROACH")

    def test_stale_telemetry_aborts(self):
        s = Supervisor()
        s.enable(True)
        self.assertEqual(s.step(**self.context(telemetry_ok=False)), "ABORT")

    def test_pilot_override_releases(self):
        s = Supervisor()
        s.enable(True)
        s.state = "VISION_FINAL"
        self.assertEqual(s.step(**self.context(mode="MANUAL", owned=True)), "RELEASE")

    def test_no_handoff_without_vision_and_alignment(self):
        c = HandoffConfig()
        data = dict(
            height=4.0,
            sink=1.0,
            progress=0.8,
            cross=1.0,
            track_error=0.0,
            net_range=30.0,
            vision_ready=True,
        )
        self.assertTrue(handoff_ready(c, **data))
        for key, value in [
            ("vision_ready", False),
            ("cross", 20.0),
            ("progress", 1.1),
            ("net_range", math.nan),
        ]:
            self.assertFalse(handoff_ready(c, **{**data, key: value}))


class GuidanceTests(unittest.TestCase):
    def control(self, target=(30, 0, 0), speed=15, velocity=(15, 0, 0)):
        return VisionGuidance(GuidanceConfig()).update(target, velocity, speed, 0, 0, 0.05)[0]

    def test_centered_level(self):
        r, p, t = self.control()
        self.assertEqual(r, 0)
        self.assertEqual(p, 0)
        self.assertEqual(t, 0.4)

    def test_left_right_signs(self):
        self.assertLess(self.control((30, 3, 0))[0], 0)
        self.assertGreater(self.control((30, -3, 0))[0], 0)

    def test_vertical_signs(self):
        self.assertGreater(self.control((30, 0, 3))[1], 0)
        self.assertLess(self.control((30, 0, -3))[1], 0)

    def test_throttle_sign_and_slew(self):
        self.assertGreater(self.control(speed=12)[2], 0.4)
        self.assertLess(self.control(speed=18)[2], 0.4)
        self.assertLessEqual(self.control(speed=12)[2] - 0.4, 0.3 * 0.05 + 1e-9)

    def test_rejects_bad_inputs(self):
        for kwargs in (
            dict(speed=0),
            dict(target=(math.nan, 0, 0)),
            dict(target=(1, 0, 0)),
            dict(velocity=(-15, 0, 0)),
        ):
            with self.assertRaises(ValueError):
                self.control(**kwargs)

    def test_airspeed_integral_no_saturation_windup(self):
        g = VisionGuidance(GuidanceConfig())
        g.integral = 1e6
        g.update((30, 0, 0), (15, 0, 0), 10, 0, 0, 0.05)
        self.assertEqual(g.integral, 1e6)


if __name__ == "__main__":
    unittest.main()
