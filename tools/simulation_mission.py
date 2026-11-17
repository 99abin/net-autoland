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


def run(config, args):
    from simulation_contract import require_preflight
    from simulation_flight import FlightObserver, managed_session
    from mavros_msgs.msg import Waypoint
    from mavros_msgs.srv import WaypointPull, WaypointPush

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
        expected, changed = with_altitude(original.waypoints, args.altitude)
        directory.mkdir(parents=True)
        save("before.json", original)
        print(json.dumps(dict(altitude=args.altitude, changed_indices=changed,
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
        actual = pull()
        save("after.json", actual)
        if not same_mission(expected, actual.waypoints) or original.current_seq != actual.current_seq:
            raise RuntimeError("Mission readback/index mismatch; inspect saved before/after files")
        print("Verified: all WAYPOINT/TAKEOFF altitudes set; HOME, LAND and mission index preserved.", flush=True)
    finally:
        observer.close()
