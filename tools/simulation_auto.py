"""Explicit opt-in autonomous experiment, restricted to the managed local SITL.

No normal application launch enables this. Flight algorithms and arming checks
are unchanged. The existing QGC mission is retained and prepared by its manager.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from flight_checks import launch_blockers
from simulation_contract import require_contract, require_preflight
from simulation_flight import FlightObserver, managed_session, plugin_status, gazebo_service

ROOT = Path(__file__).resolve().parents[1]
TAKEOFF = {"TKOFF_THR_MINACC": 15., "TKOFF_THR_MINSPD": 0.,
           "TKOFF_THR_DELAY": 2, "TKOFF_ACCEL_CNT": 1}


def preserve_native_fallback(outcome, mode, phases):
    return (outcome == "FLIGHT_RUNNING" and mode == "AUTO"
            and not {"HANDOFF_PENDING", "VISION_FINAL", "ABORT", "COMPLETE"}.intersection(phases))


def run(config, args):
    if not args.execute:
        print("No actions taken. --execute explicitly permits simulator-only parameter writes, "
              "arming, AUTO and one catapult request. Start a fresh --launch-assist session "
              "and run verify-attitude first. The existing QGC base mission is retained.")
        return
    if not 10 <= args.duration <= 1800:
        raise ValueError("Autonomous experiment duration must be 10..1800 wall seconds")
    managed_session(config)
    require_contract(config)
    require_preflight()
    status = plugin_status()
    if status.get("used") or not status.get("ready"):
        raise RuntimeError("Autonomous test needs a fresh unused launch plugin at original ground pose")
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue
    from mavros_msgs.srv import CommandBool, SetMode, WaypointSetCurrent
    from std_srvs.srv import Trigger
    from rclpy.executors import ExternalShutdownException

    observer = FlightObserver(full_autoland=True)
    directory = observer.begin_recording()
    app = None
    log = (directory / "application.log").open("w")
    launched = False
    outcome = "INCOMPLETE"
    actions = []

    def save_action(name, detail):
        actions.append(dict(action=name, detail=str(detail), wall_time=time.time()))
        (directory / "actions.json").write_text(json.dumps(actions, indent=2) + "\n")
        print(name + ": " + str(detail), flush=True)

    def call(typ, name, request, timeout=8):
        managed_session(config)
        client = observer.node.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError("Missing service " + name)
            future = client.call_async(request)
            observer.ros.spin_until_future_complete(observer.node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError("No response from " + name)
            return future.result()
        finally:
            observer.node.destroy_client(client)

    def wait_for(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            managed_session(config)
            observer.spin(.1)
            if app is not None and app.poll() is not None:
                raise RuntimeError("Application exited; inspect " + str(directory / "application.log"))
            if predicate():
                return
        raise RuntimeError("Timed out: " + description)

    def state():
        message = observer.messages.get("state")
        if message is None or time.monotonic() - observer.received.get("state", 0) > 3 or not message.connected:
            raise RuntimeError("Fresh connected FCU state required")
        return message

    def mode(name):
        request = SetMode.Request()
        request.custom_mode = name
        result = call(SetMode, "/mavros/set_mode", request)
        if not result.mode_sent:
            raise RuntimeError("Mode request rejected: " + name)
        wait_for(lambda: state().mode == name, 8, name + " telemetry")
        save_action("mode_confirmed", name)

    try:
        # DDS discovery can take longer than a fixed three-second sleep under
        # GUI load. Wait for an actually fresh state; do not widen its lease.
        wait_for(lambda: observer.messages.get("state") is not None
                 and observer.messages["state"].connected
                 and time.monotonic() - observer.received["state"] <= 3,
                 15, "initial fresh connected FCU state")
        if state().armed or state().mode != "MANUAL":
            raise RuntimeError("Must start disarmed MANUAL")
        baseline = {}
        def baseline_ready():
            nonlocal baseline
            if state().armed or state().mode != "MANUAL":
                raise RuntimeError("Left disarmed MANUAL while reading takeoff baseline")
            baseline = observer.takeoff_parameters()
            return all(type(baseline.get(name)) in (int, float) and math.isfinite(baseline[name])
                       for name in TAKEOFF)
        wait_for(baseline_ready, 25, "readable native takeoff baseline (no parameter writes yet)")
        (directory / "takeoff_parameter_baseline.json").write_text(json.dumps(baseline, indent=2) + "\n")
        (directory / "native_parameter_baseline.json").write_text(json.dumps(baseline, indent=2) + "\n")
        for name, desired in TAKEOFF.items():
            if state().armed or state().mode != "MANUAL":
                raise RuntimeError("Left disarmed preflight during takeoff parameter setup")
            original = baseline.get(name)
            if type(original) not in (int, float) or not math.isfinite(original):
                raise RuntimeError("Takeoff parameter unavailable: " + name)
            if original == desired:
                continue
            value = ParameterValue()
            if type(original) is int:
                value.type, value.integer_value = 2, int(desired)
            else:
                value.type, value.double_value = 3, float(desired)
            parameter = Parameter()
            parameter.name, parameter.value = name, value
            request = SetParameters.Request()
            request.parameters = [parameter]
            result = call(SetParameters, "/mavros/param/set_parameters", request)
            if len(result.results) != 1 or not result.results[0].successful:
                raise RuntimeError("SITL rejected takeoff parameter " + name)
            save_action("native_parameter_write", f"{name}: {original} -> {desired}")
        actual = observer.takeoff_parameters()
        from flight_checks import native_takeoff_blockers
        if native_takeoff_blockers(actual):
            raise RuntimeError("Takeoff readback mismatch: " + str(actual))
        if any(not math.isclose(actual.get(k, math.inf), v, rel_tol=0, abs_tol=1e-5)
               for k, v in TAKEOFF.items()):
            raise RuntimeError("Native simulator experiment profile readback mismatch")
        (directory / "native_takeoff_parameters.json").write_text(json.dumps(actual, indent=2) + "\n")
        app = subprocess.Popen(["bash", "tools/sim.sh", "autoland", "--upload-mission"],
                               cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        save_action("application_pid", app.pid)
        wait_for(lambda: all(observer.messages.get(k) is not None and observer.messages[k].verified
                             for k in ("profile", "plan")), 100, "verified parameter profile and landing plan")
        mission = observer.messages.get("mission")
        takeoff = [i for i, item in enumerate(mission.waypoints) if item.command == 22] if mission else []
        if len(takeoff) != 1:
            raise RuntimeError("Existing mission must contain exactly one NAV_TAKEOFF; edit it in QGC")
        request = WaypointSetCurrent.Request()
        request.wp_seq = takeoff[0]
        result = call(WaypointSetCurrent, "/mavros/mission/set_current", request)
        if not result.success:
            raise RuntimeError("Could not select TAKEOFF")
        wait_for(lambda: observer.messages["mission"].current_seq == takeoff[0], 8, "TAKEOFF selection")
        result = call(Trigger, "/landing_supervisor/enable", Trigger.Request())
        if not result.success:
            raise RuntimeError("Supervisor enable refused: " + result.message)
        wait_for(lambda: observer.messages.get("supervisor") is not None
                 and observer.messages["supervisor"].monitoring_enabled, 5, "enabled supervisor")
        flags = observer.flags()
        observer.spin(.2)
        plugin = plugin_status()
        def pre_arm_ready():
            snapshot = observer.snapshot(flags, plugin)
            snapshot.update(armed=True, mode="AUTO", native_takeoff_ok=True)
            reasons = launch_blockers(snapshot, time.monotonic(), full_autoland=True)
            from flight_checks import finite_json
            (directory / "pre_arm_checks.json").write_text(json.dumps(finite_json(
                dict(snapshot=snapshot, blockers=reasons)), indent=2) + "\n")
            return not reasons
        wait_for(pre_arm_ready, 10, "all pre-arm readiness guards (see pre_arm_checks.json)")
        require_contract(config)
        # Arm in MANUAL with a zero throttle ground state; AUTO throttle remains
        # inhibited until the reviewed acceleration trigger. Never force-arm.
        request = CommandBool.Request()
        request.value = True
        result = call(CommandBool, "/mavros/cmd/arming", request)
        if not result.success:
            raise RuntimeError("Arming refused by FCU; arming checks remain enabled")
        wait_for(lambda: state().armed, 5, "armed telemetry")
        save_action("armed", True)
        mode("AUTO")
        flags = observer.flags()
        native_ok = not native_takeoff_blockers(observer.takeoff_parameters())
        plugin = plugin_status()
        def pre_launch_ready():
            snapshot = observer.snapshot(flags, plugin)
            snapshot["native_takeoff_ok"] = native_ok
            reasons = launch_blockers(snapshot, time.monotonic(), full_autoland=True)
            from flight_checks import finite_json
            (directory / "pre_launch_checks.json").write_text(json.dumps(finite_json(
                dict(snapshot=snapshot, blockers=reasons)), indent=2) + "\n")
            return not reasons
        wait_for(pre_launch_ready, 5, "all pre-launch guards (see pre_launch_checks.json)")
        output = gazebo_service("/trigger", "gz.msgs.Boolean", "gz.msgs.Boolean", "data: true")
        if "true" not in output:
            raise RuntimeError("Launch request not confirmed; do not retry without inspecting plugin")
        launched = True
        outcome = "FLIGHT_RUNNING"
        save_action("catapult_accepted", True)
        end = time.monotonic() + args.duration
        last_print = 0
        native_fallback_reported = False
        while time.monotonic() < end:
            observer.spin(.2)
            managed_session(config)
            if app.poll() is not None:
                raise RuntimeError("Application exited during flight; requesting RTL")
            if time.monotonic() - last_print > 10:
                last_print = time.monotonic()
                m = observer.messages
                print(json.dumps(dict(mode=getattr(m.get("state"), "mode", None),
                                      altitude=getattr(m.get("altitude"), "data", None),
                                      mission=getattr(m.get("mission"), "current_seq", None),
                                      supervisor=getattr(m.get("supervisor"), "state", None),
                                      reason=getattr(m.get("supervisor"), "reason", None))), flush=True)
            status = observer.messages.get("supervisor")
            if (not state().armed and state().mode == "AUTO" and observer.armed_auto
                    and not observer.guided_observed and not observer.vision_owner_observed
                    and not {"HANDOFF_PENDING", "VISION_FINAL", "ABORT"}.intersection(observer.supervisor_states)
                    and observer.land_index is not None
                    and observer.land_index in observer.auto_sequences):
                outcome = "NATIVE_AUTO_DISARMED; net capture not established"
                save_action("native_fcu_disarmed", "FCU disarmed after native LAND; no automatic capture claim")
                break
            if status and status.state in ("ABORT", "COMPLETE"):
                outcome = status.state + ": " + status.reason
                save_action("terminal_supervisor_state", status.state + ": " + status.reason)
                observer.spin(5)
                break
            plan, gps = observer.messages.get("plan"), observer.messages.get("gps")
            mission = observer.messages.get("mission")
            if plan and gps and mission and mission.current_seq == plan.land_index:
                from autoland.geometry import leg_position
                progress, _, _, _ = leg_position(plan.approach_lat, plan.approach_lon,
                    plan.land_lat, plan.land_lon, gps.latitude, gps.longitude)
                if progress >= 1 and not observer.vision_owner_observed and not native_fallback_reported:
                    native_fallback_reported = True
                    save_action("native_auto_continues", "Passed LAND without vision takeover; leaving native AUTO in control")
            altitude = observer.messages.get("altitude")
            if time.monotonic() > end - args.duration + 20 and altitude and altitude.data < -2:
                raise RuntimeError("Simulated aircraft below ground reference")
        (directory / "launch_result.json").write_text(json.dumps(plugin_status(), indent=2) + "\n")
    except (KeyboardInterrupt, ExternalShutdownException):
        outcome = "INTERRUPTED"
        save_action("experiment_interrupted", "Observer interrupted; initiating local SITL recovery")
    except Exception as exc:
        outcome = "ERROR: " + str(exc)
        save_action("experiment_error", str(exc))
        raise
    finally:
        # Preserve an uninterrupted native AUTO fallback on normal observation
        # expiry. Interruptions, faults and attempted/owned handoffs still use
        # confirmed recovery. No blind force-disarm is permitted in flight.
        recovery_confirmed = False
        native_auto_preserved = False
        try:
            if state().armed and outcome.startswith("COMPLETE"):
                save_action("capture_hold_preserved", "Application remains running; handle simulator disarming explicitly")
            elif (state().armed and not observer.guided_observed and not observer.vision_owner_observed
                  and preserve_native_fallback(outcome, state().mode, observer.supervisor_states)):
                native_auto_preserved = True
                outcome = "NATIVE_AUTO_FALLBACK_OBSERVATION_LIMIT; landing not confirmed"
                save_action("native_auto_preserved", "Observation limit reached without handoff; no RTL requested")
            elif state().armed:
                mode("RTL")
                recovery_confirmed = True
            else:
                recovery_confirmed = True
        except Exception as exc:
            save_action("recovery_unconfirmed", str(exc))
            # Signals can invalidate this process's ROS context. A new process
            # can still independently verify local SITL and confirm recovery.
            try:
                recovery = subprocess.run(["bash", "tools/sim.sh", "recover"], cwd=ROOT,
                                          capture_output=True, text=True, timeout=25)
                (directory / "recovery.log").write_text(recovery.stdout + recovery.stderr)
                recovery_confirmed = recovery.returncode == 0
            except (OSError, subprocess.TimeoutExpired) as recovery_error:
                save_action("independent_recovery_failed", str(recovery_error))
        summary = observer.summary()
        if outcome == "FLIGHT_RUNNING":
            outcome = "RECORDING_TIME_LIMIT"
        summary.update(catapult_requested=launched, recovery_confirmed=recovery_confirmed,
                       native_auto_preserved=native_auto_preserved,
                       experiment_outcome=outcome)
        (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print("Experiment result:", json.dumps(summary, indent=2), "\nLogs:", directory, flush=True)
        if app is not None and app.poll() is None and (recovery_confirmed or native_auto_preserved):
            app.send_signal(signal.SIGINT)
            try:
                app.wait(timeout=15)
            except subprocess.TimeoutExpired:
                save_action("application_still_running", app.pid)
        log.close()
        observer.close()
    if not outcome.startswith("COMPLETE"):
        raise RuntimeError("Flight experiment did not achieve confirmed capture: " + outcome)
