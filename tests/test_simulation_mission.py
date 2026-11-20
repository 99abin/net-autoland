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


class AdaptiveBaseTests(unittest.TestCase):
    def test_does_not_add_staging_or_change_original_waypoints(self):
        from simulation_mission import adaptive_base
        from autoland.mission_builder import LandingConfig
        config=LandingConfig(-35.36,149.16,1.26,180.,500.,50.)
        base=[Item(frame=0),Item(command=22),Item(x_lat=-35.36,y_long=149.17,z_alt=50)]
        self.assertEqual(adaptive_base(base,config),base)

    def test_reconciles_every_partial_four_item_journal_without_duplicate_appends(self):
        from dataclasses import asdict
        from simulation_mission import adaptive_base
        from autoland.mission_builder import LandingConfig,fingerprint,landing_pair
        config=LandingConfig(-35.36,149.16,1.26,180.,500.,50.)
        base=[Item(frame=0),Item(command=22),Item(x_lat=-35.36,y_long=149.17,z_alt=50)]
        expected=base+[Item(x_lat=-35.35),Item(x_lat=-35.34)]+landing_pair(config)
        journal=dict(base=[asdict(w) for w in base],expected=[asdict(w) for w in expected],
                     status=dict(base_id=fingerprint(base)))
        for count in range(5):
            self.assertEqual(adaptive_base(expected[:len(base)+count],config,journal),base)
        tampered=expected.copy();tampered[1]=Item(command=22,z_alt=100)
        with self.assertRaises(ValueError):adaptive_base(tampered,config,journal)


class QGCImportTests(unittest.TestCase):
    def test_import_preserves_actual_home_and_positionless_takeoff(self):
        import json,tempfile
        from simulation_mission import qgc_plan
        home=Item(frame=0,x_lat=-35.36,y_long=149.16,z_alt=584.)
        data=dict(fileType='Plan',mission=dict(plannedHomePosition=[1,2,3],items=[
            dict(type='SimpleItem',command=22,frame=3,autoContinue=True,params=[15,0,0,0,0,0,50]),
            dict(type='SimpleItem',command=16,frame=3,autoContinue=True,params=[0,0,0,None,-35.37,149.17,50])]))
        with tempfile.NamedTemporaryFile(mode='w',suffix='.plan') as f:
            json.dump(data,f);f.flush();items=qgc_plan([home],f.name)
            self.assertEqual(items[0],home)
            self.assertEqual(items[1].x_lat,0)
            self.assertTrue(all(type(getattr(items[1],k)) is float for k in ('param1','param2','param3','param4','x_lat','y_long','z_alt')))
            self.assertEqual(items[2].z_alt,50)
            import math
            self.assertEqual(items[2].param4,0.)
            data['mission']['items'][1]['type']='ComplexItem'
            f.seek(0);f.truncate();json.dump(data,f);f.flush()
            with self.assertRaises(ValueError):qgc_plan([home],f.name)
