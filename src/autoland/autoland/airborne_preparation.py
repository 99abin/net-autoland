"""Opt-in airborne planning and four independently verified append transactions."""
from collections import deque
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import time

from .adaptive_approach import AdaptiveConfig, integrate_distance, stable_wind, remaining_route, cruise_true_airspeed
from .geometry import finite
from .alignment_planner import alignment_items
from .mission_builder import LandingConfig, copy_item, landing_pair, same_mission, fingerprint
from .mission_partial import write_one, item_packet, decode_reply
from .ros_util import retained, recent, stamp_age, profile_values


class AirbornePreparation:
    def __init__(self, node):
        from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import String, Float64
        from mavros_msgs.msg import Mavlink, VfrHud
        from autoland_interfaces.srv import SetControlOwnership
        from rclpy.qos import qos_profile_sensor_data
        self.n=node
        self.c=AdaptiveConfig(**{k:node.p[k] for k in AdaptiveConfig.__dataclass_fields__})
        self.c.validate()
        self.state='INITIAL'
        self.reason='Waiting for disarmed base-mission verification'
        self.base=None
        self.base_id=''
        self.current=0
        self.wind=deque(maxlen=1000)
        self.gps=self.alt=self.velocity=self.hud=None
        self.times={}
        self.last_publish=0
        self.raw_sequence=0
        self.raw_index=None
        self.raw_item=None
        self.raw_requested=False
        self.ack=None
        self.details={}
        self.verified_appends=0
        self.error_future=None
        self.journal=Path(node.p['airborne_journal_path']).expanduser()
        self.message_class=Mavlink
        self.string_class=String
        self.publisher=node.create_publisher(String,'/autoland/airborne_preparation',retained())
        ns=node.p['mavros_namespace'].rstrip('/')
        node.create_subscription(TwistWithCovarianceStamped,ns+'/wind_estimation',self.wind_cb,qos_profile_sensor_data)
        node.create_subscription(TwistStamped,ns+'/local_position/velocity_local',self.velocity_cb,qos_profile_sensor_data)
        node.create_subscription(NavSatFix,ns+'/global_position/global',lambda m:self.save('gps',m),qos_profile_sensor_data)
        node.create_subscription(Float64,ns+'/global_position/rel_alt',lambda m:self.save('alt',m),qos_profile_sensor_data)
        node.create_subscription(VfrHud,ns+'/vfr_hud',lambda m:self.save('hud',m),qos_profile_sensor_data)
        self.raw_pub=node.create_publisher(Mavlink,node.p['mavlink_sink_topic'],qos_profile_sensor_data)
        node.create_subscription(Mavlink,node.p['mavlink_source_topic'],self.raw_cb,qos_profile_sensor_data)
        self.recovery=node.create_client(SetControlOwnership,'/autoland/set_control_ownership')
        self.recovery_type=SetControlOwnership

    def now(self):
        return self.n.get_clock().now().nanoseconds/1e9

    def save(self,key,message):
        setattr(self,key,message)
        self.times[key]=time.monotonic()

    def wind_cb(self,m):
        if not self.n.state or not self.n.state.armed:
            self.wind.clear()
            return
        v=m.twist.twist.linear
        # ArduPilot WIND carries no measurement timestamp. MAVROS stamps it
        # with its plugin's receipt clock (wall time on Humble). Use our ROS
        # reception clock for the observation window and monotonic receipt lease.
        # This does not claim a source measurement age or alter the input header.
        if not finite(v.x,v.y,v.z):
            return
        stamp=self.now()
        if self.wind and stamp<=self.wind[-1][0]:
            if stamp<self.wind[-1][0]:self.wind.clear()
            return
        self.wind.append((stamp,v.x,v.y))
        self.times['wind']=time.monotonic()

    def velocity_cb(self,m):
        self.save('velocity',m)

    def raw_cb(self,m):
        if m.sysid==1 and m.compid==1 and m.framing_status==1 and m.msgid==42 and m.len>=2:
            import struct
            self.current=m.payload64[0]&65535
            self.times['current']=time.monotonic()
        if self.state!='ACK_WAIT':return
        try:
            if (not self.n.profile_ok() or not recent(self.n.state_time,3)
                    or not self.n.state.armed or self.n.state.mode!='AUTO'
                    or self.current>=len(self.base) or not self.fresh_inputs()):
                raise ValueError('Append authorization lost')
            reply=decode_reply(m,self.raw_index)
            if reply is None:return
            if reply[0]=='request':
                self.raw_requested=True
                # ArduPilot may request legacy MISSION_ITEM even while accepting
                # ITEM_INT. Keep GPS precision using its supported INT encoding.
                self.send(item_packet(self.raw_item,self.raw_index,self.sequence(),True))
            elif self.raw_requested:
                self.ack=reply[1]
        except ValueError as exc:
            self.fail(str(exc))

    def sequence(self):
        self.raw_sequence=(self.raw_sequence+1)%256
        return self.raw_sequence

    def send(self,fields):
        m=self.message_class()
        for key,value in fields.items():setattr(m,key,value)
        m.framing_status=1
        m.header.stamp=self.n.get_clock().now().to_msg()
        self.raw_pub.publish(m)

    def status(self):
        return dict(state=self.state,reason=self.reason,stamp_s=self.now(),base_verified=self.base is not None,
            base_id=self.base_id,base_count=len(self.base or []),current_seq=self.current,
            end_index=len(self.base or [])-1,profile_id=self.n.transaction_profile_id or '',
            t_reference_s=self.c.t_reference_s,verified_appends=self.verified_appends,**self.details)

    def save_journal(self):
        def item_record(w):
            return {k:('NaN' if isinstance(v,float) and math.isnan(v) else v)
                    for k,v in asdict(copy_item(w)).items()}
        value=dict(status=self.status(),config=asdict(self.c),
            base=[item_record(w) for w in self.base],
            expected=[item_record(w) for w in self.n.expected])
        temp=self.journal.with_suffix('.tmp')
        with temp.open('w') as stream:
            json.dump(value,stream,indent=2,allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        temp.replace(self.journal)

    def fail(self,reason):
        self.state='FAILED'
        self.reason=reason
        if self.base is not None and self.n.expected is not None:
            try:self.save_journal()
            except (OSError,ValueError) as exc:self.reason+='; journal update failed: '+str(exc)
        self.n.fail('Airborne preparation failed: '+reason)

    def fresh_inputs(self):
        return (all(recent(self.times.get(k),1.5) for k in ('gps','alt','velocity','hud'))
            and self.velocity.header.frame_id=='map'
            and 0<=stamp_age(self.n,self.gps.header.stamp)<=1.5
            and 0<=stamp_age(self.n,self.velocity.header.stamp)<=1.5)

    def pull(self,state):
        if not self.n.pull.service_is_ready():return
        self.n.request_pull('ADAPTIVE_BUSY')
        self.state=state
        self.n.deadline=time.monotonic()+self.n.p['transaction_timeout_s']

    def pulled(self):
        if not self.n.future.done():return False
        r=self.n.future.result()
        if r is None or not r.success:raise ValueError('Mission pull failed')
        return self.n.generation>self.n.baseline and len(self.n.items)==r.wp_received

    def tick(self):
        n=self.n
        if time.monotonic()-self.last_publish> .5:
            self.publisher.publish(self.string_class(data=json.dumps(self.status(),allow_nan=False)))
            self.last_publish=time.monotonic()
        try:
            if self.state=='FAILED':
                if (self.base is not None and n.state and n.state.armed and n.state.mode=='AUTO'
                        and self.recovery.service_is_ready() and self.error_future is None):
                    req=self.recovery_type.Request(owner='ABORT',plan_id=self.base_id)
                    self.error_future=self.recovery.call_async(req)
                return
            if self.state=='VERIFIED':
                if n.expected is None:self.fail('Verified appended mission was changed')
                return
            if not n.p['configured'] or not recent(n.state_time,3) or not n.state.connected:return
            if not n.profile_ok():
                if self.base is not None:self.fail('Parameter profile lost during preparation')
                return
            if self.base is not None and n.profile.profile_id!=n.transaction_profile_id:
                raise ValueError('Parameter profile changed during preparation')
            if self.base is not None and n.state.armed:
                if self.state=='BASE_READY' and n.state.mode=='MANUAL':return
                if n.state.mode!='AUTO':raise ValueError('Left AUTO during airborne preparation')
                if not recent(self.times.get('current'),3):raise ValueError('Mission progress stream is stale')
                if self.current>=len(self.base):
                    raise ValueError('Original END waypoint already passed before preparation completed')
                if self.state!='BASE_READY' and not self.fresh_inputs():raise ValueError('Airborne planning telemetry is stale')
            if self.state=='INITIAL':
                if n.state.armed or n.state.mode!='MANUAL':return
                if self.journal.exists():raise ValueError('Existing airborne journal requires ground inspection before restart')
                self.journal.parent.mkdir(parents=True,exist_ok=True)
                self.journal.with_suffix('.tmp').write_text('{}')
                self.journal.with_suffix('.tmp').unlink()
                n.transaction_profile_id=n.profile.profile_id
                self.pull('BASE_WAIT')
            elif self.state=='BASE_WAIT':
                if n.state.armed:raise ValueError('Base verification requires disarmed MANUAL')
                if self.pulled():
                    if (len(n.items)<3 or any(w.command not in (16,22) for w in n.items)
                            or sum(w.command==22 for w in n.items)!=1 or n.items[0].frame!=0
                            or any(w.frame!=3 for w in n.items[1:])
                            or any(not finite(w.x_lat,w.y_long,w.z_alt) for w in n.items)):
                        raise ValueError('Need a base mission of HOME, one TAKEOFF and WAYPOINTs, without LAND')
                    self.base=[copy_item(w) for w in n.items]
                    self.base_id=fingerprint(self.base)
                    self.state='BASE_READY'
                    self.reason='Base mission verified; waiting for airborne wind observations'
            elif self.state=='BASE_READY':
                if not same_mission(n.items,self.base):raise ValueError('Original mission changed')
                if not n.state.armed:return
                if not self.fresh_inputs():
                    self.reason='Waiting for fresh GPS, altitude and local velocity'
                    return
                if not finite(self.alt.data) or self.alt.data<self.c.minimum_planning_alt_m:return
                if not recent(self.times.get('wind'),self.c.wind_timeout_s):
                    self.reason='Waiting for fresh wind stream'
                    return
                try:
                    east,north,std=stable_wind(self.c,list(self.wind),self.now())
                except ValueError as exc:
                    self.reason=str(exc)
                    return
                n.config=LandingConfig(**{k:n.p[k] for k in LandingConfig.__dataclass_fields__})
                distance,details=integrate_distance(self.c,n.config.approach_heading_deg,east,north,
                    n.config.approach_alt-n.config.land_alt)
                n.config=replace(n.config,approach_distance_m=distance)
                n.config.validate()
                budget=self.route_budget(east,north)
                if budget['remaining_route_time_s']<self.c.minimum_route_time_s:
                    raise ValueError('Insufficient original route time remains for verified appends')
                self.pair,alignment=alignment_items(n.config,self.c,self.base[-1],east,north,
                    budget['cruise_tas_mps'])
                self.details=dict(distance_m=distance,wind_east_mps=east,wind_north_mps=north,
                    wind_observed_std_mps=std,**details,**budget,**alignment)
                n.expected=self.base+self.pair
                self.reason='Integrated approach frozen; checking unchanged base before writes'
                self.pull('RECHECK_WAIT')
            elif self.state=='RECHECK_WAIT':
                if self.pulled():
                    if not same_mission(n.items,self.base):raise ValueError('Base mission changed before append')
                    if not n.p['upload_enabled']:raise ValueError('Airborne mission writes are disabled')
                    self.save_journal()
                    self.begin_append(0)
            elif self.state=='ACK_WAIT':
                if self.ack is not None:
                    if self.ack!=0:raise ValueError('FCU rejected partial append: MAV_MISSION_RESULT '+str(self.ack))
                    self.pull('APPEND_VERIFY_WAIT')
                elif time.monotonic()>self.raw_deadline:
                    # Readback resolves an ACK lost after the FCU stored the item.
                    # Never retry an uncertain append blindly.
                    self.pull('APPEND_VERIFY_WAIT')
            elif self.state=='APPEND_VERIFY_WAIT':
                if self.pulled():
                    expected=n.expected[:len(self.base)+self.part+1]
                    if not same_mission(n.items,expected):raise ValueError('Partial append readback mismatch; inspect journal')
                    self.verified_appends=self.part+1
                    if self.part+1<len(self.pair):
                        self.save_journal()
                        self.begin_append(self.part+1)
                    else:
                        self.state='VERIFIED'
                        self.reason='All four airborne landing items and unchanged original mission verified'
                        self.save_journal()
                        n.publish_plan(True,self.reason)
                        n.phase='IDLE'
            if self.state.endswith('_WAIT') and self.state!='ACK_WAIT' and time.monotonic()>n.deadline:
                raise ValueError('Airborne mission readback timed out')
        except Exception as exc:
            self.fail(str(exc))

    def begin_append(self,part):
        budget=self.route_budget(self.details['wind_east_mps'],self.details['wind_north_mps'])
        if budget['remaining_route_time_s']<self.c.minimum_route_time_s:
            raise ValueError('Original route preparation budget expired before append')
        self.details.update(budget)
        self.part=part
        self.raw_item=self.pair[part]
        self.raw_index=len(self.base)+part
        self.raw_requested=False
        self.ack=None
        self.state='ACK_WAIT'
        self.reason='Appending only '+('adjustment waypoint','alignment waypoint','approach waypoint','NAV_LAND')[part]
        self.raw_deadline=time.monotonic()+self.c.append_timeout_s
        self.send(write_one(self.raw_index,self.sequence()))


    def route_budget(self,east,north):
        if not self.fresh_inputs() or not 0<=self.current<len(self.base):
            raise ValueError('Fresh original route telemetry is required')
        values=profile_values(self.n,self.n.profile,self.n.profile_time)
        if values is None or 'AIRSPEED_CRUISE' not in values:
            raise ValueError('Verified FCU AIRSPEED_CRUISE is required for route-time prediction')
        eas=values['AIRSPEED_CRUISE']
        tas=cruise_true_airspeed(eas,self.hud.altitude)
        route=[(self.gps.latitude,self.gps.longitude)]+[(w.x_lat,w.y_long)
            for w in self.base[self.current:] if not (w.command==22 and w.x_lat==0 and w.y_long==0)]
        result=remaining_route(route,tas,east,north,self.c.min_predicted_groundspeed_mps)
        result.update(cruise_eas_mps=eas,cruise_tas_mps=tas,cruise_altitude_amsl_m=self.hud.altitude,
            cruise_conversion='ISA troposphere density estimate',minimum_route_time_s=self.c.minimum_route_time_s)
        return result
