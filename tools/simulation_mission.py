"""Explicit preflight waypoint-height edit for the managed local simulator."""
from dataclasses import asdict
import math
import os
from pathlib import Path
import time
import json

from autoland.mission_builder import copy_item, same_mission

ROOT = Path(__file__).resolve().parents[1]


def with_altitude(items, altitude):
    if not math.isfinite(altitude) or not 5 <= altitude <= 150:
        raise ValueError("Simulator waypoint height must be 5..150 m above HOME")
    if len(items) < 2 or items[0].command != 16 or items[0].frame != 0:
        raise ValueError("Expected ArduPlane HOME as item zero; refusing ambiguous mission")
    result = [copy_item(item) for item in items]
    changed = []
    for index, item in enumerate(result[1:], 1):
        if item.command == 21:
            continue  # Preserve the configured net interception height.
        if item.command not in (16, 22) or item.frame != 3:
            raise ValueError("Expected only relative-HOME WAYPOINT/TAKEOFF and LAND items")
        if item.z_alt != altitude:
            changed.append(index)
        item.z_alt = float(altitude)
    return result, changed


def adaptive_base(items, config, journal=None):
    """SITL fixture: preserve the base, remove only journaled appends or our known static pair."""
    from autoland.mission_builder import landing_pair, Item, fingerprint
    base=[copy_item(w) for w in items]
    if journal is not None:
        def restore(record):return Item(**{k:(float('nan') if v=='NaN' else v) for k,v in record.items()})
        saved=[restore(w) for w in journal['base']]
        expected=[restore(w) for w in journal['expected']]
        if (fingerprint(saved)!=journal['status']['base_id'] or len(expected)-len(saved) not in (2,4)
                or not same_mission(saved,expected[:len(saved)])
                or not any(same_mission(base,expected[:len(saved)+n]) for n in range(len(expected)-len(saved)+1))):
            raise ValueError('Mission does not match the airborne journal; inspect before resetting')
        base=saved
    if base and base[-1].command==21:
        if not same_mission(base[-2:],landing_pair(config)):
            raise ValueError('Existing LAND pair differs; refusing to remove it')
        base=base[:-2]
    if any(w.command not in (16,22) for w in base):
        raise ValueError('Adaptive test base requires only HOME, TAKEOFF and WAYPOINTs')
    return base


def qgc_plan(items, path):
    """Keep actual FCU HOME; never move spawn/HOME to the planner reference."""
    from autoland.mission_builder import Item
    data=json.loads(Path(path).read_text())
    if data.get('fileType')!='Plan':raise ValueError('Expected QGC Plan file')
    if not items or items[0].frame!=0:raise ValueError('Missing actual FCU HOME')
    if any(data.get('geoFence',{}).get(k) for k in ('circles','polygons')) or data.get('rallyPoints',{}).get('points'):
        raise ValueError('Importing fence/rally data is unsupported')
    result=[copy_item(items[0])]
    for raw in data['mission']['items']:
        if raw.get('type')!='SimpleItem' or raw.get('command') not in (16,22) or raw.get('frame')!=3:
            raise ValueError('Only relative-HOME simple WAYPOINT/TAKEOFF items are supported')
        params=raw['params']
        if len(params)!=7 or not all(type(v) in (int,float) and math.isfinite(v) for v in params[4:]):
            raise ValueError('Invalid mission coordinates/altitude')
        if not -89<=params[4]<=89 or not -180<=params[5]<=180 or not 5<=params[6]<=150:
            raise ValueError('Mission position or test altitude outside supported limits')
        if any(v is not None and (type(v)not in (int,float) or not math.isfinite(v)) for v in params[:4]):
            raise ValueError('Invalid mission command parameters')
        result.append(Item(command=raw['command'],autocontinue=raw['autoContinue'],
            **{f'param{i+1}':(0. if i==3 and raw['command']==16 else float('nan')) if v is None else float(v) for i,v in enumerate(params[:4])},
            x_lat=float(params[4]),y_long=float(params[5]),z_alt=float(params[6])))
    if len(result)<3 or result[1].command!=22 or sum(w.command==22 for w in result)!=1:
        raise ValueError('Need TAKEOFF followed by ordinary waypoints')
    return result


def run(config, args):
    from simulation_contract import require_preflight
    from simulation_flight import FlightObserver, managed_session
    from mavros_msgs.msg import Waypoint
    from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointSetCurrent

    managed_session(config)
    require_preflight()
    observer = FlightObserver()
    directory = ROOT / ".simulation/mission_backups" / (time.strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}")

    def guard():
        managed_session(config)
        state = observer.messages.get("state")
        if (state is None or time.monotonic() - observer.received.get("state", 0) > 3
                or not state.connected or state.armed or state.mode != "MANUAL"):
            raise RuntimeError("Mission editing requires fresh connected DISARMED MANUAL SITL")

    def call(typ, name, request):
        guard()
        client = observer.node.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError("Missing service: " + name)
            future = client.call_async(request)
            observer.ros.spin_until_future_complete(observer.node, future, timeout_sec=30)
            guard()
            if not future.done() or future.result() is None or not future.result().success:
                raise RuntimeError("Mission service failed: " + name)
            return future.result()
        finally:
            observer.node.destroy_client(client)

    def pull():
        since = time.monotonic()
        response = call(WaypointPull, "/mavros/mission/pull", WaypointPull.Request())
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            observer.spin(.1)
            guard()
            mission = observer.messages.get("mission")
            if (mission is not None and observer.received.get("mission", 0) >= since
                    and len(mission.waypoints) == response.wp_received):
                return mission
        raise RuntimeError("No fresh mission readback")

    def save(name, mission):
        (directory / name).write_text(json.dumps(dict(
            current_seq=mission.current_seq,
            waypoints=[asdict(copy_item(item)) for item in mission.waypoints]), indent=2) + "\n")

    try:
        observer.spin(4)
        original = pull()
        if getattr(args,'command','')=='import-plan':
            expected=qgc_plan(original.waypoints,args.file)
            changed=not same_mission(expected,original.waypoints)
        elif getattr(args,'adaptive_base',False):
            import yaml
            from autoland.mission_builder import LandingConfig
            values=yaml.safe_load((ROOT/'src/start/config/mission.yaml').read_text())['mission_manager']['ros__parameters']
            journal=ROOT/'.simulation/adaptive_plan.json'
            expected=adaptive_base(original.waypoints,LandingConfig(**{k:values[k] for k in LandingConfig.__dataclass_fields__}),
                json.loads(journal.read_text()) if journal.exists() else None)
            if original.current_seq>=len(expected):
                raise ValueError('Select an original base item while disarmed before resetting the airborne test')
            changed=not same_mission(expected,original.waypoints)
        else:
            expected, changed = with_altitude(original.waypoints, args.altitude)
        directory.mkdir(parents=True)
        save("before.json", original)
        print(json.dumps(dict(altitude=getattr(args,'altitude',None), changed_indices=changed,
                              writes_enabled=args.apply, backup=str(directory / "before.json"))), flush=True)
        if not args.apply:
            return
        if changed:
            request = WaypointPush.Request()
            request.start_index = 0
            request.waypoints = [Waypoint(**asdict(item)) for item in expected]
            response = call(WaypointPush, "/mavros/mission/push", request)
            if response.wp_transfered != len(expected):
                raise RuntimeError("Incomplete mission upload; inspect mission before retrying")
        desired_index=original.current_seq
        if getattr(args,'command','')=='import-plan':
            desired_index=1
            if not call(WaypointSetCurrent,'/mavros/mission/set_current',WaypointSetCurrent.Request(wp_seq=1)).success:
                raise RuntimeError('Imported mission TAKEOFF selection failed')
        actual = pull()
        save("after.json", actual)
        if not same_mission(expected, actual.waypoints) or desired_index != actual.current_seq:
            raise RuntimeError("Mission readback/index mismatch; inspect saved before/after files")
        print("Verified mission readback and requested active index; backup: "+str(directory), flush=True)
        if getattr(args,'adaptive_base',False) or getattr(args,'command','')=='import-plan':
            journal=ROOT/'.simulation/adaptive_plan.json'
            if journal.exists():journal.replace(directory/'previous_adaptive_plan.json')
    finally:
        observer.close()
