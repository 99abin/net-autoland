import math
import unittest
from dataclasses import replace
from autoland.adaptive_approach import AdaptiveConfig,remaining_route,cruise_true_airspeed
from autoland.alignment_planner import alignment_items,local,sub,dot,norm,rot
from autoland.mission_builder import LandingConfig,Item
from autoland.geometry import destination


class AlignmentTests(unittest.TestCase):
    config=LandingConfig(-35.36,149.16,1.26,180.,500.,50.)

    def test_circle_tangencies_and_coupled_alignment_equation_all_quadrants(self):
        for heading in (0,45,90,180,270):
            config=replace(self.config,approach_heading_deg=float(heading))
            for bearing in (0,60,120,180,240,300):
                lat,lon=destination(config.land_lat,config.land_lon,bearing,1100)
                end=Item(x_lat=lat,y_long=lon,z_alt=50.)
                items,s=alignment_items(config,AdaptiveConfig(),end,1.,-1.,16.)
                self.assertEqual([w.command for w in items],[16,16,16,21])
                self.assertEqual([w.z_alt for w in items],[50.,50.,50.,1.26])
                self.assertAlmostEqual(s['alignment_distance_m'],60*math.tan(math.radians(s['alignment_turn_deg'])/2)
                    +s['alignment_groundspeed_mps']*8+20,places=2)
                t,q=s['adjustment'],s['alignment']
                app=local(items[2].x_lat,items[2].y_long,(config.land_lat,config.land_lon))
                line=sub(app,q)
                self.assertAlmostEqual(dot(rot(line),app),0.,places=4)
                if s['circle_center'] is not None:
                    center=s['circle_center']
                    e=local(end.x_lat,end.y_long,(config.land_lat,config.land_lon))
                    self.assertAlmostEqual(norm(sub(t,center)),60.,places=5)
                    self.assertAlmostEqual(norm(sub(q,center)),60.,places=5)
                    self.assertAlmostEqual(dot(sub(t,center),sub(t,e)),0.,places=4)
                    self.assertAlmostEqual(dot(sub(q,center),line),0.,places=4)

    def test_already_aligned_entry_has_no_artificial_loop(self):
        lat,lon=destination(self.config.land_lat,self.config.land_lon,0,1200)
        _,s=alignment_items(self.config,AdaptiveConfig(),Item(x_lat=lat,y_long=lon),0,0,16)
        self.assertEqual(s['circle_side'],'straight')
        self.assertAlmostEqual(s['alignment_distance_m'],148.)

    def test_end_need_not_be_behind_approach(self):
        lat,lon=destination(self.config.land_lat,self.config.land_lon,90,100)
        _,s=alignment_items(self.config,AdaptiveConfig(),Item(x_lat=lat,y_long=lon),0,0,16)
        self.assertIn(s['circle_side'],('left','right'))

    def test_impossible_alignment_bounds_are_rejected(self):
        c=replace(AdaptiveConfig(),alignment_distance_min_m=10.,alignment_distance_max_m=20.)
        with self.assertRaisesRegex(ValueError,'bounded'):
            alignment_items(self.config,c,Item(x_lat=-35.35,y_long=149.17),0,0,16)


class RouteTests(unittest.TestCase):
    def test_projects_wind_on_each_leg_and_includes_last_wp(self):
        points=[(0.,0.),(0.,.001),(0.,0.),(.001,0.)]
        r=remaining_route(points,15.,3.,0.)
        legs=r['route_legs']
        self.assertEqual(len(legs),3)
        for actual,expected in zip([x['groundspeed_mps'] for x in legs],[18.,12.,math.sqrt(216)]):
            self.assertAlmostEqual(actual,expected)
        self.assertAlmostEqual(r['remaining_route_time_s'],sum(x['distance_m']/x['groundspeed_mps'] for x in legs))

    def test_duplicate_points_zero_time_and_infeasible_wind_rejected(self):
        self.assertEqual(remaining_route([(0,0),(0,0)],15,0,0)['remaining_route_time_s'],0.)
        with self.assertRaises(ValueError):remaining_route([(0,0),(0,.01)],15,0,16)
        with self.assertRaises(ValueError):remaining_route([(0,0),(0,.01)],15,-12,0)

    def test_eas_tas_density_conversion_uses_amsl(self):
        self.assertEqual(cruise_true_airspeed(16,0),16.)
        self.assertGreater(cruise_true_airspeed(16,584),16.)
        self.assertLess(cruise_true_airspeed(16,584),17.)
        for alt in (float('nan'),12000):
            with self.assertRaises(ValueError):cruise_true_airspeed(16,alt)
