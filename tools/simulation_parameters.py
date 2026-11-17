"""Backed-up, explicitly applied native parameter trials in managed local SITL."""
import json
import math
import os
from pathlib import Path
import time

import yaml

ROOT = Path(__file__).resolve().parents[1]
LIMITS = {"PTCH_RATE_FF": (.05, 3.), "PTCH_RATE_I": (.01, .6),
          "PTCH_RATE_P": (0., .35), "PTCH_RATE_D": (0., .03),
          "PTCH2SRV_TCONST": (.4, 1.), "LAND_OPTIONS": (0, 2147483647),
          "TECS_LAND_ARSPD": (-1, 30), "LAND_PF_ARSPD": (0, 30)}


def load_trial(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get("simulator_only") is not True:
        raise ValueError("Native trials must explicitly declare simulator_only: true")
    values = data.get("parameters")
    if not isinstance(values, dict) or not values:
        raise ValueError("A nonempty parameter mapping is required")
    for name, value in values.items():
        if name not in LIMITS:
            raise ValueError("Parameter outside reviewed native trial scope: " + str(name))
        low, high = LIMITS[name]
        if (type(value) not in (int, float) or not math.isfinite(value)
                or not low - 1e-6 <= value <= high + 1e-6):
            raise ValueError("Invalid native trial value for " + name)
        if name == "LAND_OPTIONS" and int(value) != value:
            raise ValueError("LAND_OPTIONS requires an integer bitmask")
    return values


def validate_landing_target(requested, current):
    if "LAND_OPTIONS" in requested:
        if (int(requested["LAND_OPTIONS"]) ^ int(current["LAND_OPTIONS"])) & ~2:
            raise ValueError("This trial may change only LAND_OPTIONS bit 1; preserve other bits")
    effective = {**current, **requested}
    preflare = effective.get("LAND_PF_ARSPD", 0)
    if preflare != 0 and not effective["AIRSPEED_MIN"] <= preflare <= effective["AIRSPEED_MAX"]:
        raise ValueError("Preflare airspeed must remain inside existing native airspeed limits")
    speed = effective["TECS_LAND_ARSPD"]
    if speed == -1:
        return
    if not effective["AIRSPEED_MIN"] <= speed <= effective["AIRSPEED_MAX"]:
        raise ValueError("Landing airspeed must remain inside existing native airspeed limits")
    if speed > effective["AIRSPEED_CRUISE"] and not int(effective["LAND_OPTIONS"]) & 2:
        raise ValueError("Landing target exceeds cruise; LAND_OPTIONS bit 1 is required to avoid clamping")


def run(config, args):
    values = load_trial(args.file)
    from simulation_flight import FlightObserver, managed_session
    from simulation_contract import require_preflight
    from autoland.parameter_profile import decode_value
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue
    from mavros_msgs.srv import ParamPull
    managed_session(config)
    require_preflight()
    observer = FlightObserver(full_autoland=True)
    directory = ROOT / ".simulation/parameter_trials" / (time.strftime("%Y%m%d-%H%M%S") + f"-{os.getpid()}")
    directory.mkdir(parents=True)
    attempted = []

    def guard():
        managed_session(config)
        state = observer.messages.get("state")
        if (state is None or time.monotonic() - observer.received["state"] > 3
                or not state.connected or state.armed or state.mode != "MANUAL"):
            raise RuntimeError("Native parameter trial requires fresh disarmed MANUAL local SITL")

    def call(typ, name, request, timeout=8):
        guard()
        client = observer.node.create_client(typ, name)
        try:
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError("Missing service: " + name)
            future = client.call_async(request)
            observer.ros.spin_until_future_complete(observer.node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError("No response from " + name)
            guard()
            return future.result()
        finally:
            observer.node.destroy_client(client)

    def read(names=None):
        request = GetParameters.Request()
        request.names = sorted(values) if names is None else list(names)
        response = call(GetParameters, "/mavros/param/get_parameters", request)
        if len(response.values) != len(request.names):
            raise RuntimeError("Incomplete parameter response")
        result = {}
        for name, value in zip(request.names, response.values):
            if value.type not in (2, 3):
                raise RuntimeError("Native parameter is unavailable/nonnumeric: " + name)
            result[name] = decode_value(value)[0]
        return result

    try:
        observer.spin(3)
        pull = ParamPull.Request()
        pull.force_pull = True
        if not call(ParamPull, "/mavros/param/pull", pull, 65).success:
            raise RuntimeError("Native FCU pull failed before trial")
        baseline = read()
        if {"LAND_OPTIONS", "TECS_LAND_ARSPD", "LAND_PF_ARSPD"}.intersection(values):
            context = read(["LAND_OPTIONS", "TECS_LAND_ARSPD", "AIRSPEED_MIN", "AIRSPEED_MAX", "AIRSPEED_CRUISE"])
            validate_landing_target(values, context)
            (directory / "airspeed_limits.json").write_text(json.dumps(context, indent=2) + "\n")
        # This baseline is itself a valid explicit restoration profile.
        (directory / "baseline.yaml").write_text(yaml.safe_dump(dict(
            simulator_only=True, description="Restore baseline before native parameter trial",
            parameters=baseline), sort_keys=False))
        (directory / "requested.yaml").write_text(Path(args.file).read_text())
        print("Native parameter trial:", json.dumps({"before": baseline, "requested": values,
                                         "writes_enabled": args.apply}), flush=True)
        if args.apply:
            for name, target in values.items():
                if math.isclose(baseline[name], target, rel_tol=0, abs_tol=1e-6):
                    continue
                parameter = Parameter()
                parameter.name = name
                value = ParameterValue()
                value.type = 3 if type(baseline[name]) is float else 2
                if value.type == 3:
                    value.double_value = float(target)
                else:
                    if int(target) != target:
                        raise ValueError("Fractional value for integer FCU parameter: " + name)
                    value.integer_value = int(target)
                parameter.value = value
                request = SetParameters.Request()
                request.parameters = [parameter]
                attempted.append(name)
                response = call(SetParameters, "/mavros/param/set_parameters", request)
                if len(response.results) != 1 or not response.results[0].successful:
                    raise RuntimeError("FCU rejected native parameter: " + name)
            if not call(ParamPull, "/mavros/param/pull", pull, 65).success:
                raise RuntimeError("Native FCU readback pull failed")
        actual = read()
        verified = args.apply and all(math.isclose(actual[k], v, rel_tol=0, abs_tol=1e-5)
                                      for k, v in values.items())
        result = dict(verified=verified, requested=values, actual=actual,
                      attempted=attempted, baseline=str(directory / "baseline.yaml"))
        (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
        if args.apply and not verified:
            raise RuntimeError("Native pitch parameter readback mismatch")
        if verified:
            (ROOT / ".simulation/active_native_parameter_trial.json").write_text(json.dumps(result, indent=2) + "\n")
        print("Native parameter result:", json.dumps(result, indent=2), flush=True)
    except Exception as exc:
        (directory / "failure.json").write_text(json.dumps(dict(
            reason=str(exc), attempted=attempted,
            message="Partial writes are not silently rolled back; inspect baseline.yaml"), indent=2) + "\n")
        raise
    finally:
        observer.close()
