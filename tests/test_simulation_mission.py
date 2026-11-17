import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "src/autoland"))
from autoland.mission_builder import Item
from simulation_mission import with_altitude


class MissionAltitudeTests(unittest.TestCase):
    def test_preserves_home_land_geometry_and_original_input(self):
        original = [Item(frame=0, z_alt=584.09), Item(command=22, z_alt=50),
                    Item(x_lat=-35.36, y_long=149.16, z_alt=100),
                    Item(command=21, z_alt=1.26, param4=1)]
        result, changed = with_altitude(original, 50)
        self.assertEqual(changed, [2])
        self.assertEqual(result[0], original[0])
        self.assertEqual(result[-1], original[-1])
        self.assertEqual(result[2].x_lat, original[2].x_lat)
        self.assertEqual(result[2].y_long, original[2].y_long)
        self.assertEqual(result[2].z_alt, 50)
        self.assertEqual(original[2].z_alt, 100)

    def test_refuses_ambiguous_altitudes_commands_and_home(self):
        for items in ([Item(), Item()], [Item(frame=0), Item(frame=0)],
                      [Item(frame=0), Item(command=20)]):
            with self.assertRaises(ValueError):
                with_altitude(items, 50)
        for altitude in (float("nan"), float("inf"), 0, 1000):
            with self.assertRaises(ValueError):
                with_altitude([Item(frame=0), Item()], altitude)
