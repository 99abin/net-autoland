"""Pure checks for the Gazebo native-AUTO launch experiment."""

import math


def native_takeoff_blockers(values):
    # SITL catapult trigger contract: hold native throttle until the physical
    # acceleration event, then use ArduPlane's existing 0.2 s motor-start delay.
    required = {"TKOFF_THR_MINACC": 15.0, "TKOFF_THR_MINSPD": 0.0,
                "TKOFF_THR_DELAY": 2, "TKOFF_ACCEL_CNT": 1}
    return [f"SITL takeoff setting {name} must be {value} (cached: {values.get(name)})"
            for name, value in required.items()
            if type(values.get(name)) not in (int, float)
            or not math.isclose(values[name], value, rel_tol=0, abs_tol=1e-6)]


def validate_assist_config(config):
    limits = {
        "duration_s": (0, 1), "forward_force_n": (0, 200),
        "upward_force_n": (0, 100), "maximum_initial_speed_mps": (0, 2),
        "maximum_initial_height_m": (0, 0.5),
    }
    for name, (lower, upper) in limits.items():
        value = config.get(name)
        if (type(value) not in (int, float) or not math.isfinite(value)
                or value > upper or value < lower
                or (name != "upward_force_n" and value == lower)):
            raise ValueError(f"Invalid simulator launch setting: {name}")


def launch_blockers(snapshot, now, full_autoland=False, airborne_preparation=False):
    reasons = []
    for name, age_limit in (("state", 3), ("pose", 1), ("velocity", 1),
                            ("altitude", 1), ("profile", 3), ("plan", 3)):
        if airborne_preparation and name=='plan':continue
        age = now - snapshot.get(name + "_received", -1e30)
        if not 0 <= age <= age_limit:
            reasons.append(f"{name} is missing or stale")
    if not snapshot.get("connected"):
        reasons.append("FCU is not connected")
    if not snapshot.get("armed"):
        reasons.append("operator must arm the simulated aircraft in QGC")
    if snapshot.get("mode") != "AUTO":
        reasons.append("operator must select AUTO in QGC")
    if snapshot.get("current_command") != 22:
        reasons.append("NAV_TAKEOFF must be the active mission command")
    if not snapshot.get("profile_verified") or (not airborne_preparation and not snapshot.get("plan_verified")):
        reasons.append("parameter profile and landing plan must be verified")
    for name in ("profile", "plan"):
        if airborne_preparation and name=='plan':continue
        age = snapshot.get(name + "_stamp_age_s", math.inf)
        if not math.isfinite(age) or not 0 <= age <= 3:
            reasons.append(f"{name} simulation-time lease is missing or stale")
    if not snapshot.get("profile_id") or (not airborne_preparation and snapshot.get("profile_id") != snapshot.get("plan_profile_id")):
        reasons.append("landing plan must reference the current verified profile")
    if not airborne_preparation and not snapshot.get("mission_matches"):
        reasons.append("current FCU mission must match the verified landing plan")
    if full_autoland:
        if not all(snapshot.get(k) is True for k in (
                "live_output", "handoff_enabled", "partial_attitude_verified")):
            reasons.append("full autoland requires all three verified handoff/output flags")
        if airborne_preparation:
            if (snapshot.get('airborne_preparation_enabled') is not True
                    or snapshot.get('fcu_airborne_preparation_enabled') is not True
                    or snapshot.get('preparation_state')!='BASE_READY'
                    or not snapshot.get('base_matches')
                    or snapshot.get('base_profile_id')!=snapshot.get('profile_id')
                    or not 0<=now-snapshot.get('preparation_received',-1e30)<=3
                    or not 0<=snapshot.get('preparation_stamp_age_s',math.inf)<=3):
                reasons.append('Need fresh verified unchanged airborne base mission and preparation/recovery flags')
        elif snapshot.get("supervisor_state") != "MISSION_READY":
            reasons.append("enable the supervisor before takeoff; MISSION_READY required")
        if not airborne_preparation and snapshot.get("supervisor_enabled") is not True:
            reasons.append("call /landing_supervisor/enable before takeoff")
        if not airborne_preparation and not snapshot.get("supervisor_fresh"):
            reasons.append("supervisor readiness is missing or stale")
    elif snapshot.get("live_output") is not False or snapshot.get("handoff_enabled") is not False:
        reasons.append("native AUTO test requires live_output=false and handoff_enabled=false")
    for name, maximum in (("speed_mps", 2), ("relative_alt_m", 0.8),
                          ("roll_deg", 15), ("pitch_deg", 20)):
        value = snapshot.get(name, math.nan)
        if not math.isfinite(value) or abs(value) > maximum:
            reasons.append(f"{name} outside initial ground envelope")
    yaw = snapshot.get("yaw_deg", math.nan)
    if not math.isfinite(yaw) or abs((yaw + 180) % 360 - 180) > 10:
        reasons.append("Merlin must retain its original east-facing spawn heading")
    if not snapshot.get("plugin_ready"):
        reasons.append("Gazebo launch plugin is not ready (ground/pose/one-shot guard)")
    if not snapshot.get("native_takeoff_ok"):
        reasons.append("reviewed SITL takeoff-trigger parameters are required")
    return reasons


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return value
