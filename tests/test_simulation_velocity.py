"""Known MAVROS ENU metadata fix must not rotate data or renew its timestamp."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from simulation_velocity import corrected_velocity


class VelocityMetadataTests(unittest.TestCase):
    def test_preserves_vectors_timestamp_and_original_message(self):
        original = NS(header=NS(frame_id="base_link", stamp=NS(sec=7, nanosec=23)),
                      twist=NS(linear=NS(x=3., y=-5., z=-2.), angular=NS(x=.1, y=.2, z=.3)))
        corrected = corrected_velocity(original)
        self.assertEqual(corrected.header.frame_id, "map")
        self.assertEqual(original.header.frame_id, "base_link")
        self.assertEqual(corrected.header.stamp, original.header.stamp)
        self.assertEqual(corrected.twist, original.twist)
        self.assertEqual(corrected_velocity(corrected), corrected)
        original.header.frame_id = "unknown"
        with self.assertRaises(ValueError):
            corrected_velocity(original)
