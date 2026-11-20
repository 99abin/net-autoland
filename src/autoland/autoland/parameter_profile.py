"""Load a landing profile and compare typed FCU values without ROS imports."""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import yaml

REQUIRED = {"LAND_TYPE", "LAND_FLARE_ALT", "LAND_FLARE_SEC"}
RELATED = {
    "TECS_FLARE_HGT",
    "TECS_APPR_SMAX",
    "AIRSPEED_CRUISE",
    "AIRSPEED_MIN",
    "AIRSPEED_MAX",
    "ARSPD_FBW_MIN",
    "ARSPD_FBW_MAX",
}


@dataclass
class Profile:
    configured: bool
    parameters: dict


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f"Duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def load_profile(path):
    with Path(path).expanduser().open() as stream:
        raw = yaml.load(stream, Loader=UniqueKeyLoader)
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError("Profile must have schema_version: 1")
    if type(raw.get("configured")) is not bool:
        raise ValueError("Profile needs configured: true/false")
    values = raw.get("parameters")
    if not isinstance(values, dict) or not values or len(values) > 256:
        raise ValueError("parameters must be a nonempty mapping, at most 256 entries")
    for name, value in values.items():
        if not isinstance(name, str) or not re.fullmatch(r"[A-Z][A-Z0-9_]{0,15}", name):
            raise ValueError(f"Invalid FCU parameter name: {name}")
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value)):
            raise ValueError(f"{name} must be a finite number or null (preserve FCU value)")
    if not REQUIRED.issubset(values):
        raise ValueError("Profile must include LAND_TYPE, LAND_FLARE_ALT and LAND_FLARE_SEC")
    return Profile(raw["configured"], values)


def selected_names(profile, available, discover=True):
    available = set(available)
    missing = set(profile.parameters) - available
    if missing:
        raise ValueError("FCU does not expose profile parameters: " + ", ".join(sorted(missing)))
    names = set(profile.parameters)
    if discover:
        names |= {n for n in available if n.startswith(("LAND_", "TECS_LAND_")) or n in RELATED}
    return sorted(names)


def decode_value(value):
    # rcl_interfaces/ParameterType: BOOL=1, INTEGER=2, DOUBLE=3.
    if value.type == 2:
        return int(value.integer_value), 2
    if value.type == 3 and math.isfinite(value.double_value):
        return float(value.double_value), 3
    raise ValueError("FCU parameter is missing, non-finite, or not numeric")


def typed_target(desired, current, kind):
    if desired is None:
        return current
    if kind == 2:
        if float(desired) != int(desired):
            raise ValueError("Cannot assign a fractional value to an integer FCU parameter")
        if not -(2**31) <= int(desired) < 2**31:
            raise ValueError("Integer outside MAVLink parameter range")
        return int(desired)
    if kind == 3:
        return float(desired)
    raise ValueError("Unsupported parameter type")


def equal_value(a, b):
    if type(a) is int and type(b) is int:
        return a == b
    return math.isfinite(a) and math.isfinite(b) and math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-5)


def validate_effective(values):
    if not isinstance(values, dict) or not REQUIRED.issubset(values):
        raise ValueError("Verified profile is missing required landing parameters")
    if not all(type(v) in (int, float) and math.isfinite(v) for v in values.values()):
        raise ValueError("Verified profile contains invalid numeric values")
    if values.get("LAND_TYPE") != 0:
        raise ValueError("This landing stack requires LAND_TYPE=0 (standard glide slope)")
    for name in (
        "LAND_FLARE_ALT",
        "LAND_FLARE_SEC",
        "LAND_PF_ALT",
        "LAND_PF_SEC",
        "LAND_PF_ARSPD",
        "TECS_LAND_SINK",
    ):
        if name in values and values[name] < 0:
            raise ValueError(f"{name} must be nonnegative")
    if "LAND_FLARE_AIM" in values and not 0 <= values["LAND_FLARE_AIM"] <= 100:
        raise ValueError("LAND_FLARE_AIM must be within [0, 100]")
    low = values.get("AIRSPEED_MIN", values.get("ARSPD_FBW_MIN"))
    high = values.get("AIRSPEED_MAX", values.get("ARSPD_FBW_MAX"))
    if low is not None and high is not None and not 0 < low < high:
        raise ValueError("Invalid FCU minimum/maximum airspeed")
    for name in ("LAND_PF_ARSPD", "TECS_LAND_ARSPD"):
        speed = values.get(name, 0)
        # Zero disables preflare; negative TECS_LAND_ARSPD means its native default.
        if speed > 0 and ((low is not None and speed < low) or (high is not None and speed > high)):
            raise ValueError(f"{name} is outside the FCU airspeed envelope")


def profile_id(values):
    return hashlib.sha256(
        json.dumps(values, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()[:20]
