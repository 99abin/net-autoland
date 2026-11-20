import unittest
from simulation_characterization import summarize

class CharacterizationTests(unittest.TestCase):
    def rows(self):
        return [dict(sim_s=i*.5,speed_mps=15.,throttle_pct=40.,airspeed_mps=14.6,
            vz_mps=0.,pitch_deg=3.,alpha_body_deg=3.,height_error_m=0.) for i in range(21)]
    def test_steady_level_point_keeps_reported_and_velocity_speeds_separate(self):
        r=summarize(self.rows())
        self.assertTrue(r['steady']);self.assertEqual(r['speed_mps'],15.)
        self.assertAlmostEqual(r['reported_airspeed_mps'],14.6)
    def test_accelerating_descending_and_short_segments_are_not_steady_mapping(self):
        self.assertFalse(summarize(self.rows()[:3])['steady'])
        for k in ('speed_mps','vz_mps','height_error_m'):
            rows=self.rows()
            for i,r in enumerate(rows):r[k]+=30 if k=='height_error_m' else i*.5
            self.assertFalse(summarize(rows)['steady'])
