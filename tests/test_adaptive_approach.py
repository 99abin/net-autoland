import math
from dataclasses import replace
import json
from pathlib import Path
import struct
import tempfile
import unittest
from types import SimpleNamespace as NS
import test_nodes as helpers
from autoland.adaptive_approach import AdaptiveConfig,integrate_distance,stable_wind
from autoland.mission_partial import write_one,item_packet,decode_reply
from autoland.mission_builder import Item
from autoland.mission_manager import MissionManager
from autoland.geometry import destination
ros=helpers.ros


class IntegrationTests(unittest.TestCase):
    def test_frames_are_trimmed_before_crc_for_mavconn_serialization(self):
        self.assertEqual(write_one(7,1)['len'],6)
        self.assertEqual(item_packet(Item(x_lat=-35.36,y_long=149.16,z_alt=50),7,2)['len'],37)

    def test_constant_speed_tailwind_example(self):
        c=AdaptiveConfig(airspeed_profile_time_fractions=[0.,1.],airspeed_profile_tas_mps=[15.,15.])
        self.assertAlmostEqual(integrate_distance(c,180,0,-3,48.74)[0],600.,places=6)

    def test_linear_profile_crosswind_integral_matches_analytic_result(self):
        c=AdaptiveConfig(airspeed_profile_time_fractions=[0.,1.],airspeed_profile_tas_mps=[16.,13.])
        def primitive(v):return .5*(v*math.sqrt(v*v-9)-9*math.acosh(v/3))
        expected=c.t_reference_s*(primitive(16)-primitive(13))/3
        self.assertAlmostEqual(integrate_distance(c,0,3,0,48.74)[0],expected,places=6)
        self.assertNotAlmostEqual(expected,c.t_reference_s*math.sqrt(14.5**2-9),places=3)

    def test_invalid_inputs_and_infeasible_duration_are_rejected(self):
        for changes,wind in ((dict(t_reference_s=0),(0,0)),
            (dict(t_reference_s=10),(0,0)),(dict(airspeed_profile_time_fractions=[0.,0.,1.]),(0,0)),
            (dict(max_wind_mps=30),(20,0)),(dict(approach_distance_max_m=400),(0,0))):
            with self.assertRaises(ValueError):integrate_distance(replace(AdaptiveConfig(),**changes),0,*wind,48.74)

    def test_wind_requires_samples_stability_and_freshness_without_direction_gate(self):
        c=AdaptiveConfig()
        samples=[(i*.4,1.,2.) for i in range(23)]
        self.assertEqual(stable_wind(c,samples,8.8),(1.,2.,0.))
        for s,t in ((samples[:3],8.8),(samples,12.),
                    ([(i*.4,float(i%2)*4,0) for i in range(23)],8.8)):
            with self.assertRaises(ValueError):stable_wind(c,s,t)

    def test_new_wind_and_strict_speed_sink_boundaries(self):
        c=AdaptiveConfig(t_reference_s=40.,airspeed_profile_time_fractions=[0.,1.],
            airspeed_profile_tas_mps=[13.,13.],approach_distance_min_m=50.,approach_distance_max_m=1000.)
        self.assertGreater(integrate_distance(c,0,0,-9.9,48.74)[0],120.)
        self.assertAlmostEqual(integrate_distance(c,0,0,10.,48.74)[0],920.)
        with self.assertRaisesRegex(ValueError,'groundspeed'):integrate_distance(c,0,0,-10.,48.74)
        with self.assertRaisesRegex(ValueError,'Wind'):integrate_distance(c,0,0,-10.01,48.74)
        with self.assertRaisesRegex(ValueError,'sink'):integrate_distance(c,0,0,0,120.)
        self.assertGreater(integrate_distance(c,0,0,0,119.9)[0],0.)

    def test_wind_variation_between_previous_and_new_limit_is_accepted(self):
        samples=[(i*.4,(.9 if i%2 else -.9),0.) for i in range(23)]
        self.assertLess(stable_wind(AdaptiveConfig(),samples,8.8)[2],1.)
        with self.assertRaises(ValueError):stable_wind(replace(AdaptiveConfig(),wind_max_std_mps=.7),samples,8.8)


class AppendTests(unittest.TestCase):
    setUp=helpers.NodeTests.setUp
    stamp=helpers.NodeTests.stamp
    state=helpers.NodeTests.state
    interface=helpers.NodeTests.interface
    status=helpers.NodeTests.status
    target=helpers.NodeTests.target

    def manager(self):
        folder=tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        ros.Node.overrides={'mission_manager':dict(configured=True,upload_enabled=True,
            airborne_preparation_enabled=True,adaptive_corridor_reviewed=True,
            airborne_journal_path=str(Path(folder.name)/'plan.json'),land_lat=-35.36,land_lon=149.16,
            land_alt=1.26,approach_alt=50.,approach_heading_deg=180.,minimum_route_time_s=1.)}
        n=MissionManager()
        n.profile_cb(ros.verified_profile(n,dict(LAND_TYPE=0,LAND_FLARE_ALT=3.,LAND_FLARE_SEC=2.,AIRSPEED_CRUISE=16.)))
        n.state_cb(self.state())
        lat,lon=destination(-35.36,149.16,0,1100)
        self.base=[Item(frame=0),Item(command=22)]+[Item(x_lat=lat,y_long=lon,z_alt=50.) for _ in range(5)]
        n.tick()
        self.finish_pull(n,self.base)
        self.assertEqual(n.adaptive.state,'BASE_READY')
        return n

    def finish_pull(self,n,items):
        n.list_cb(ros.WaypointList(waypoints=items,current_seq=n.adaptive.current))
        n.future.finish(success=True,wp_received=len(items))
        n.tick()

    def ready(self,n):
        n.state_cb(self.state('AUTO',True))
        a=n.adaptive
        a.current=2
        a.times['current']=ros.clock.value
        a.save('gps',self.stamp(n,ros.NavSatFix(latitude=-35.355,longitude=149.16)))
        a.save('alt',ros.Float64(data=50.))
        a.save('hud',ros.VfrHud(altitude=584.09))
        a.save('velocity',self.stamp(n,ros.TwistStamped(),'map'))
        a.wind.extend((ros.clock.value-8.8+i*.4,0.,0.) for i in range(23))
        a.times['wind']=ros.clock.value
        n.tick()
        self.finish_pull(n,self.base)
        self.assertEqual(a.state,'ACK_WAIT')

    def reply(self,index,ack=False,result=0):
        payload=bytes([1,192,result,0]) if ack else struct.pack('<HBBB',index,1,192,0)
        return ros.Mavlink(sysid=1,compid=1,framing_status=1,msgid=47 if ack else 51,
            len=len(payload),payload64=[int.from_bytes(payload.ljust(8,b'\0'),'little')])

    def transfer(self,n):
        a=n.adaptive
        a.raw_cb(self.reply(a.raw_index))
        a.raw_cb(self.reply(a.raw_index,True))
        n.tick()

    def test_only_four_new_items_are_sent_and_each_is_read_back(self):
        n=self.manager()
        self.ready(n)
        items=n.adaptive.pair
        self.assertEqual([w.command for w in items],[16,16,16,21])
        for count in range(1,5):
            self.transfer(n)
            self.finish_pull(n,self.base+items[:count])
        self.assertTrue(n.plan.verified)
        self.assertEqual(n.plan.approach_index,len(self.base)+2)
        self.assertEqual(n.plan.land_index,len(self.base)+3)
        self.assertEqual([m.msgid for m in n.adaptive.raw_pub.messages],[38,73]*4)
        self.assertEqual(len(n.push.calls),0)
        journal=json.loads(n.adaptive.journal.read_text())
        self.assertEqual(journal['status']['state'],'VERIFIED')
        self.assertEqual(journal['status']['verified_appends'],4)

    def test_second_append_failure_preserves_journal_and_never_reports_ready(self):
        n=self.manager();self.ready(n)
        self.transfer(n);self.finish_pull(n,self.base+n.adaptive.pair[:1])
        n.adaptive.raw_cb(self.reply(8));n.adaptive.raw_cb(self.reply(8,True,1));n.tick()
        self.assertEqual(n.adaptive.state,'FAILED')
        self.assertFalse(n.plan.verified)
        self.assertTrue(n.adaptive.journal.exists())
        self.assertEqual(json.loads(n.adaptive.journal.read_text())['status']['state'],'FAILED')
        self.assertEqual(len(n.push.calls),0)

    def test_third_and_fourth_failures_do_not_publish_a_verified_plan(self):
        for failing in (2,3):
            n=self.manager();self.ready(n)
            for part in range(failing):
                self.transfer(n)
                self.finish_pull(n,self.base+n.adaptive.pair[:part+1])
            a=n.adaptive
            a.raw_cb(self.reply(a.raw_index));a.raw_cb(self.reply(a.raw_index,True,1));n.tick()
            self.assertEqual(a.state,'FAILED')
            self.assertEqual(a.verified_appends,failing)
            self.assertFalse(n.plan.verified)
            self.assertEqual(len(n.push.calls),0)

    def test_lost_ack_is_resolved_by_readback_without_resending_append(self):
        n=self.manager();self.ready(n)
        n.adaptive.raw_cb(self.reply(7))
        n.adaptive.raw_deadline=ros.clock.value-1
        n.tick()
        self.finish_pull(n,self.base+n.adaptive.pair[:1])
        self.assertEqual(n.adaptive.raw_index,8)
        self.assertEqual([m.msgid for m in n.adaptive.raw_pub.messages],[38,73,38])

    def test_unresolved_journal_blocks_a_new_preflight_attempt(self):
        n=self.manager();self.ready(n)
        n.adaptive.state='INITIAL'
        n.state_cb(self.state())
        n.tick()
        self.assertEqual(n.adaptive.state,'FAILED')
        self.assertIn('journal',n.adaptive.reason)

    def test_stale_progress_prevents_writes(self):
        n=self.manager()
        n.state_cb(self.state('AUTO',True))
        n.tick()
        self.assertEqual(n.adaptive.state,'FAILED')
        self.assertFalse(n.adaptive.raw_pub.messages)

    def test_original_last_index_is_allowed_without_fixed_index_deadline(self):
        n=self.manager();self.ready(n)
        n.adaptive.current=len(self.base)-1
        n.adaptive.raw_cb(self.reply(7))
        self.assertEqual(n.adaptive.state,'ACK_WAIT')
        self.assertEqual(n.adaptive.raw_pub.messages[-1].msgid,73)

    def test_positionless_takeoff_does_not_forecast_a_leg_to_zero_zero(self):
        n=self.manager();self.ready(n)
        a=n.adaptive
        a.current=1
        first=a.route_budget(0.,0.)
        a.current=2
        second=a.route_budget(0.,0.)
        self.assertAlmostEqual(first['remaining_route_distance_m'],second['remaining_route_distance_m'])
        self.assertLess(first['remaining_route_distance_m'],10000.)

    def test_time_budget_expiry_prevents_next_item(self):
        n=self.manager();self.ready(n)
        n.adaptive.c.minimum_route_time_s=10000.
        self.transfer(n);self.finish_pull(n,self.base+n.adaptive.pair[:1])
        self.assertEqual(n.adaptive.state,'FAILED')
        self.assertIn('budget',n.adaptive.reason)
        self.assertEqual([m.msgid for m in n.adaptive.raw_pub.messages],[38,73])

    def test_timestamp_less_ardupilot_wind_uses_receipt_clock_without_rewriting_header(self):
        n=self.manager()
        n.state_cb(self.state('AUTO',True))
        wind=ros.TwistWithCovarianceStamped()
        wind.header.stamp.sec=1700000000
        wind.twist.twist.linear.x=1.
        wind.twist.twist.linear.y=-2.
        n.adaptive.wind_cb(wind)
        self.assertEqual(n.adaptive.wind[-1],(ros.clock.value,1.,-2.))
        self.assertEqual(wind.header.stamp.sec,1700000000)

    def test_failure_recovery_requires_explicit_flag_and_matching_base(self):
        n=self.interface(True)
        msg=ros.String(data=json.dumps(dict(state='FAILED',base_verified=True,base_id='base',reason='deadline')))
        n.preparation_cb(msg)
        self.assertFalse(n.owner_cb(NS(owner='ABORT',plan_id='base'),NS()).accepted)
        n.p['airborne_preparation_enabled']=True
        self.assertFalse(n.owner_cb(NS(owner='ABORT',plan_id='wrong'),NS()).accepted)
        self.assertTrue(n.owner_cb(NS(owner='ABORT',plan_id='base'),NS()).accepted)
        self.assertEqual(n.mode_client.calls[-1][0].custom_mode,'RTL')

class ProfileOrderingTests(unittest.TestCase):
    setUp=AppendTests.setUp
    state=AppendTests.state
    manager=AppendTests.manager
    finish_pull=AppendTests.finish_pull
    def test_identical_future_heartbeat_keeps_previous_strictly_fresh_lease(self):
        import copy
        from autoland.ros_util import reordered_profile_heartbeat
        n=self.manager()
        old=n.profile;receipt=n.profile_time
        incoming=copy.deepcopy(old)
        incoming.header.stamp.nanosec+=1000000
        self.assertTrue(reordered_profile_heartbeat(n,incoming,old,receipt))
        n.profile_cb(incoming)
        self.assertIs(n.profile,old)
        self.assertEqual(n.profile_time,receipt)
        self.assertTrue(n.profile_ok())
        incoming.verified=False
        self.assertFalse(reordered_profile_heartbeat(n,incoming,old,receipt))
        n.profile_cb(incoming)
        self.assertFalse(n.profile_ok())

    def test_changed_payload_and_expired_lease_are_not_hidden(self):
        import copy
        from autoland.ros_util import reordered_profile_heartbeat
        n=self.manager();old=n.profile
        incoming=copy.deepcopy(old);incoming.header.stamp.nanosec+=1000000
        self.assertFalse(reordered_profile_heartbeat(n,incoming,old,ros.clock.value-4))
        incoming.parameters_json='{}'
        self.assertFalse(reordered_profile_heartbeat(n,incoming,old,n.profile_time))
