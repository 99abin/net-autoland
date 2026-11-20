"""Explicit managed-SITL-only roll/pitch AUTOTUNE experiment; no landing app."""
import json,math,os,struct,sys,time,signal
from pathlib import Path
from simulation import settings,environment
from simulation_flight import FlightObserver,managed_session,plugin_status,gazebo_service
from simulation_contract import require_preflight
from autoland.geometry import quaternion_euler
from autoland.mission_partial import crc


def rc_override_packet(channels,gcs_id,seq):
    """MAVLink2 RC override; source must be the verified configured GCS ID."""
    if len(channels)!=8 or not 1<=gcs_id<=255 or not 0<=seq<=255:
        raise ValueError('Invalid RC packet identity or channel count')
    if any(type(v)is not int or not (v in (0,65535) or 800<=v<=2200) for v in channels):
        raise ValueError('RC command outside experiment PWM limits')
    payload=struct.pack('<8HBB',*channels,1,1)
    header=bytes([len(payload),0,0,seq,gcs_id,193])+int(70).to_bytes(3,'little')
    return dict(magic=253,len=len(payload),seq=seq,sysid=gcs_id,compid=193,msgid=70,
        checksum=crc(header+payload+bytes([124])),payload64=list(struct.unpack('<QQQ',payload.ljust(24,b'\0'))))


def run(characterization=False):
    config=settings();os.environ.update(environment(config))
    managed_session(config);require_preflight()
    if plugin_status().get('used'):raise RuntimeError('Fresh launch-assist session required')
    from rcl_interfaces.srv import GetParameters,SetParameters,ListParameters
    from rcl_interfaces.msg import Parameter,ParameterValue
    from mavros_msgs.srv import SetMode,CommandBool,ParamPull,CommandLong
    from mavros_msgs.msg import Mavlink
    from rclpy.qos import qos_profile_sensor_data
    o=FlightObserver()
    def interrupt(signum,frame):raise KeyboardInterrupt('Operator interrupted tuning')
    signal.signal(signal.SIGINT,interrupt)
    signal.signal(signal.SIGTERM,interrupt)
    folder=o.begin_recording();(folder/'autotune_runner.py').write_text(Path(__file__).read_text());seq=0;baseline={};finished=set();armed=False
    native_mode=[None];rc_seen=[None];gcs_id=[None]
    result=dict(passed=False,reason='Incomplete',directory=str(folder))
    pub=o.node.create_publisher(Mavlink,'/uas1/mavlink_sink',qos_profile_sensor_data)
    def raw(m):
        if m.sysid==1 and m.compid==1 and m.msgid==0 and m.framing_status==1:
            native_mode[0]=m.payload64[0]&0xffffffff
        if m.sysid==1 and m.compid==1 and m.msgid==65 and m.framing_status==1:
            data=b''.join(int(v).to_bytes(8,'little') for v in m.payload64)
            rc_seen[0]=struct.unpack_from('<4H',data,4)
        if m.sysid==1 and m.compid==1 and m.msgid==253 and m.framing_status==1:
            b=b''.join(int(v).to_bytes(8,'little') for v in m.payload64)
            text=b[1:51].split(b'\0')[0].decode(errors='replace')
            if 'Finished' in text:finished.add(text)
            with (folder/'statustext.jsonl').open('a') as f:f.write(json.dumps(dict(time=time.time(),text=text))+'\n')
    o.node.create_subscription(Mavlink,'/uas1/mavlink_source',raw,qos_profile_sensor_data)
    def call(typ,name,req):
        managed_session(config)
        client=o.node.create_client(typ,name)
        try:
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Missing '+name)
            f=client.call_async(req);o.ros.spin_until_future_complete(o.node,f,timeout_sec=65)
            if not f.done() or f.result() is None:raise RuntimeError('Timeout '+name)
            return f.result()
        finally:o.node.destroy_client(client)
    def state():
        if time.monotonic()-o.received.get('state',0)>3:raise RuntimeError('Stale FCU state')
        return o.messages['state']
    def wait(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            o.spin(.05)
            if fn():return
        raise RuntimeError('Telemetry wait expired')
    def read(names):
        r=call(GetParameters,'/mavros/param/get_parameters',GetParameters.Request(names=names))
        if len(r.values)!=len(names) or any(v.type not in (2,3) for v in r.values):raise RuntimeError('Incomplete parameter cache')
        return {k:(v.integer_value if v.type==2 else v.double_value) for k,v in zip(names,r.values)}
    def write(values):
        ps=[]
        for k,v in values.items():
            original=baseline[k];pv=ParameterValue(type=2 if type(original)is int else 3)
            if pv.type==2:pv.integer_value=int(v)
            else:pv.double_value=float(v)
            ps.append(Parameter(name=k,value=pv))
        r=call(SetParameters,'/mavros/param/set_parameters',SetParameters.Request(parameters=ps))
        if len(r.results)!=len(ps) or not all(v.successful for v in r.results):raise RuntimeError('Parameter write refused')
        actual=read(list(values))
        if any(not math.isclose(actual[k],v,abs_tol=1e-4) for k,v in values.items()):raise RuntimeError('Parameter readback mismatch')
    def mode(name):
        if name=='TAKEOFF':
            r=call(CommandLong,'/mavros/cmd/command',CommandLong.Request(command=176,param1=1.,param2=13.))
            if not r.success:raise RuntimeError('Native TAKEOFF request refused')
            wait(lambda:native_mode[0]==13,8);return
        r=call(SetMode,'/mavros/set_mode',SetMode.Request(custom_mode=name))
        if not r.mode_sent:raise RuntimeError('Mode refused '+name)
        wait(lambda:state().mode==name,8)
    def rc(roll=1500,pitch=1500,throttle=1700,release=False):
        nonlocal seq
        channels=[0]*8 if release else [roll,pitch,throttle,1500,65535,65535,65535,65535]
        seq=(seq+1)%256
        m=Mavlink(**rc_override_packet(channels,gcs_id[0],seq))
        m.framing_status=1;pub.publish(m)
    try:
        wait(lambda:'state'in o.messages and o.messages['state'].connected,15)
        if state().armed or state().mode!='MANUAL':raise RuntimeError('Disarmed MANUAL required')
        if not call(ParamPull,'/mavros/param/pull',ParamPull.Request(force_pull=True)).success:raise RuntimeError('FCU parameter pull failed')
        names=call(ListParameters,'/mavros/param/list_parameters',ListParameters.Request()).result.names
        selected=[k for k in names if k.startswith(('RLL_RATE_','PTCH_RATE_')) or k in ('RLL2SRV_TCONST','PTCH2SRV_TCONST','AUTOTUNE_LEVEL','AUTOTUNE_AXES','TKOFF_ALT','TKOFF_DIST','TKOFF_THR_MINACC','TKOFF_THR_MINSPD','TKOFF_THR_DELAY','TKOFF_ACCEL_CNT','SYSID_MYGCS','MAV_GCS_SYSID','RCMAP_ROLL','RCMAP_PITCH','RCMAP_THROTTLE','RCMAP_YAW','RC1_MIN','RC1_MAX','RC1_TRIM','RC2_MIN','RC2_MAX','RC2_TRIM','RC3_MIN','RC3_MAX','RC3_TRIM','RC4_TRIM','STALL_PREVENTION','AIRSPEED_MIN','AIRSPEED_STALL','PTCH_LIM_MAX_DEG','PTCH_LIM_MIN_DEG','PTCH_TRIM_DEG','RC2_DZ')]
        baseline=read(selected);(folder/'autotune_baseline.json').write_text(json.dumps(baseline,indent=2))
        gcs_id[0]=int(baseline.get('MAV_GCS_SYSID',baseline.get('SYSID_MYGCS',0)))
        if not 1<=gcs_id[0]<=255:raise RuntimeError('Missing configured GCS identity')
        if [baseline[k] for k in ('RCMAP_ROLL','RCMAP_PITCH','RCMAP_THROTTLE','RCMAP_YAW')]!=[1,2,3,4]:raise RuntimeError('Unsupported RC map')
        if baseline['RC1_TRIM']!=1500 or baseline['RC2_TRIM']!=1500:raise RuntimeError('Unsupported RC trim')
        setup=dict(TKOFF_ALT=620,TKOFF_DIST=5000,TKOFF_THR_MINACC=15,TKOFF_THR_MINSPD=0,TKOFF_THR_DELAY=2,TKOFF_ACCEL_CNT=1)
        if not characterization:setup.update(AUTOTUNE_AXES=3,AUTOTUNE_LEVEL=3)
        write(setup)
        call(CommandLong,'/mavros/cmd/command',CommandLong.Request(command=511,param1=65.,param2=100000.))
        rc(throttle=1100)
        wait(lambda:rc_seen[0] is not None and rc_seen[0][:3]==(1500,1500,1100),5)
        print('RC override reception confirmed',flush=True)
        if not call(CommandBool,'/mavros/cmd/arming',CommandBool.Request(value=True)).success:raise RuntimeError('Arming refused')
        armed=True;wait(lambda:state().armed,5);mode('TAKEOFF')
        rc(throttle=1800)
        if 'true' not in gazebo_service('/trigger','gz.msgs.Boolean','gz.msgs.Boolean','data: true'):raise RuntimeError('Launch refused')
        end=time.monotonic()+400;peak=0.
        while time.monotonic()<end:
            rc(throttle=1800);o.spin(.1)
            if native_mode[0]!=13:raise RuntimeError('Unexpected takeoff mode')
            if o.messages.get('altitude'):
                alt=o.messages['altitude'].data;peak=max(peak,alt)
                if peak>20 and alt<10:raise RuntimeError('Takeoff lost altitude before AUTOTUNE')
                if alt>600:break
        else:raise RuntimeError('Takeoff altitude not reached')
        if characterization:
            from simulation_characterization import characterize
            result.update(characterize(o,folder,mode,rc,state,baseline,config))
            return 0
        mode('AUTOTUNE');start=o.node.get_clock().now().nanoseconds/1e9;deadline=time.monotonic()+600
        while time.monotonic()<deadline:
            managed_session(config);o.spin(.05)
            if state().mode!='AUTOTUNE':raise RuntimeError('Left AUTOTUNE')
            for k in ('pose','altitude','airspeed'):
                if time.monotonic()-o.received.get(k,0)>1.5:raise RuntimeError('Stale '+k)
            q=o.messages['pose'].pose.orientation;r,p,_=quaternion_euler(q.x,q.y,q.z,q.w)
            if o.messages['altitude'].data<65 or abs(r)>math.radians(65) or abs(p)>math.radians(40) or o.messages['airspeed'].airspeed<14:raise RuntimeError('AUTOTUNE flight envelope exceeded')
            elapsed=o.node.get_clock().now().nanoseconds/1e9-start
            axis=int(elapsed//160)%2;phase=elapsed%8
            command=1900 if phase<2 else 1100 if phase<4 else 1500
            rc(roll=command if axis==0 else 1500,pitch=command if axis==1 else 1500,throttle=1700)
            if any('Roll' in s for s in finished) and any('Pitch' in s for s in finished):
                result['passed']=True;result['reason']='Both axes reported Finished';break
        else:raise RuntimeError('AUTOTUNE did not complete both axes')
    except BaseException as exc:
        result['reason']=str(exc);print('AUTOTUNE stopped:',exc,flush=True)
    finally:
        try:
            rc(throttle=1000 if not armed else 1700);mode('RTL' if armed else 'MANUAL');rc(release=True)
            result['rtl_confirmed']=True
            if baseline:
                if not call(ParamPull,'/mavros/param/pull',ParamPull.Request(force_pull=True)).success:raise RuntimeError('Final FCU parameter pull failed')
                result['after']=read(list(baseline));(folder/'autotune_after.json').write_text(json.dumps(result['after'],indent=2))
        except Exception as exc:result['cleanup_error']=str(exc)
        result['finished_messages']=sorted(finished)
        filename='characterization_result.json' if characterization else 'autotune_result.json'
        (folder/filename).write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True);o.close()
    return 0 if result['passed'] else 1

if __name__=='__main__':
    if sys.argv[1:] not in (['--execute'],['--execute','--characterize']):raise SystemExit('Explicit --execute required; managed local SITL only')
    raise SystemExit(run('--characterize' in sys.argv))
