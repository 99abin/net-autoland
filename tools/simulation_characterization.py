"""Calm-air, straight-flight SITL performance sampling; no landing or gain writes."""
import csv,json,math,statistics,time
from autoland.geometry import quaternion_euler,inverse_rotate,clip


def summarize(rows):
    if len(rows)<20:return dict(steady=False,reason='Insufficient samples')
    def mean(k):return statistics.mean(r[k] for r in rows)
    dt=rows[-1]['sim_s']-rows[0]['sim_s']
    accel=(rows[-1]['speed_mps']-rows[0]['speed_mps'])/max(dt,.01)
    return dict(samples=len(rows),throttle_pct=mean('throttle_pct'),speed_mps=mean('speed_mps'),
        reported_airspeed_mps=mean('airspeed_mps'),speed_std_mps=statistics.pstdev(r['speed_mps'] for r in rows),
        acceleration_mps2=accel,vertical_speed_mps=mean('vz_mps'),pitch_deg=mean('pitch_deg'),
        alpha_body_deg=mean('alpha_body_deg'),height_error_m=mean('height_error_m'),
        steady=abs(accel)<.15 and abs(mean('vz_mps'))<.5 and abs(mean('height_error_m'))<10
            and statistics.pstdev(r['speed_mps'] for r in rows)<.5)


def characterize(o,folder,mode,rc,state,baseline,config):
    from simulation_flight import managed_session
    # GPS-derived local velocity is only an airspeed proxy in this calm world.
    mode('FBWA');height=o.messages['altitude'].data;rows=[];points=[];stall=None
    maxpitch=float(baseline['PTCH_LIM_MAX_DEG']);minpitch=float(baseline['PTCH_LIM_MIN_DEG'])
    trim=float(baseline.get('PTCH_TRIM_DEG',0));deadzone=float(baseline['RC2_DZ'])
    start=o.node.get_clock().now().nanoseconds/1e9;wallend=time.monotonic()+900
    bias=3.;previous=start
    stream=(folder/'characterization.csv').open('w',buffering=1);writer=None
    try:
        for pwm in (1740,1700,1660,1620,1600,1580,1560,1540):
            t0=o.node.get_clock().now().nanoseconds/1e9;stage=[]
            print('Throttle stage PWM',pwm,flush=True)
            while o.node.get_clock().now().nanoseconds/1e9-t0<35:
                if time.monotonic()>wallend:raise RuntimeError('Characterization time budget expired')
                managed_session(config);o.spin(.05)
                if state().mode!='FBWA':raise RuntimeError('Left FBWA')
                for key in ('pose','velocity','altitude','airspeed','wind'):
                    if time.monotonic()-o.received.get(key,0)>1.5:raise RuntimeError('Stale '+key)
                wind=o.messages['wind'].twist.twist.linear
                if math.hypot(wind.x,wind.y)>.5:raise RuntimeError('Calm-air test requires wind below 0.5 m/s')
                q=o.messages['pose'].pose.orientation;quat=(q.x,q.y,q.z,q.w)
                roll,pitch,_=quaternion_euler(*quat)
                vel=o.messages['velocity'].twist.linear
                body=inverse_rotate(quat,(vel.x,vel.y,vel.z));speed=math.sqrt(vel.x**2+vel.y**2+vel.z**2)
                alt=o.messages['altitude'].data
                if alt<150 or abs(roll)>math.radians(45) or abs(pitch)>math.radians(35):raise RuntimeError('Characterization envelope exceeded')
                # Slow outer altitude loop, bounded pitch request to native FBWA.
                clock=o.node.get_clock().now().nanoseconds/1e9
                bias=clip(bias+.03*(height-alt)*clip(clock-previous,0,.2),-3.,12.)
                previous=clock
                demand=clip(.18*(height-alt)-1.6*vel.z+bias,-5.,18.)
                normalized=clip((demand-trim)/(maxpitch if demand>=trim else abs(minpitch)),-.9,.9)
                pwm_pitch=round(1500+(deadzone+(400-deadzone)*abs(normalized))*(1 if normalized>=0 else -1))
                rc(pitch=pwm_pitch,throttle=pwm)
                now=o.node.get_clock().now().nanoseconds/1e9
                row=dict(sim_s=now,stage_pwm=pwm,throttle_pct=100*o.messages['airspeed'].throttle,
                    speed_mps=speed,airspeed_mps=o.messages['airspeed'].airspeed,vz_mps=vel.z,
                    pitch_deg=math.degrees(pitch),roll_deg=math.degrees(roll),
                    alpha_body_deg=math.degrees(math.atan2(-body[2],body[0])),height_error_m=height-alt)
                if writer is None:writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader()
                writer.writerow(row);rows.append(row);stage.append(row)
                # A sustained inability to hold height identifies a practical
                # level-flight limit, not proof of aerodynamic stall.
                if now-t0>2 and height-alt>15 and vel.z< -1.5:
                    stall=dict(kind='level-flight capability limit, not confirmed aerodynamic stall',**row)
                    break
            tail=[r for r in stage if r['sim_s']>=stage[-1]['sim_s']-10]
            point=dict(requested_pwm=pwm,**summarize(tail));points.append(point)
            print(json.dumps(point),flush=True)
            (folder/'characterization_result.json').write_text(json.dumps(dict(points=points,limit=stall),indent=2))
            if stall:break
    finally:stream.close()
    return dict(passed=True,reason='Characterization samples collected',points=points,limit=stall,
        caveat='Velocity-based speed assumes calm air; a height-holding limit is not proof of stall')
