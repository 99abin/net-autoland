"""Build and compare mission items without importing ROS or MAVROS."""

from dataclasses import dataclass
import hashlib
import json
import math
from .geometry import destination, finite


@dataclass(frozen=True)
class LandingConfig:
    land_lat: float
    land_lon: float
    land_alt: float
    approach_heading_deg: float
    approach_distance_m: float
    approach_alt: float

    def validate(self):
        if not finite(*self.__dict__.values()):
            raise ValueError("Landing configuration must be finite")
        if not -89 <= self.land_lat <= 89 or not -180 <= self.land_lon <= 180:
            raise ValueError("Invalid LAND coordinates (polar regions unsupported)")
        if self.approach_distance_m < 10:
            raise ValueError("Approach distance must be at least 10 m")
        if self.approach_alt <= self.land_alt:
            raise ValueError("Approach altitude must exceed LAND altitude")
        if not 0 <= self.approach_heading_deg < 360:
            raise ValueError("Heading must be within [0,360)")
        if self.land_alt < 0:
            raise ValueError("Home-relative LAND altitude must be nonnegative")


@dataclass
class Item:
    frame: int = 3
    command: int = 16
    is_current: bool = False
    autocontinue: bool = True
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    x_lat: float = 0.0
    y_long: float = 0.0
    z_alt: float = 0.0


FIELDS = tuple(Item.__dataclass_fields__)


def copy_item(item):
    return Item(**{k: getattr(item, k) for k in FIELDS})


def same_item(a, b):
    for key in FIELDS:
        if key == "is_current":
            continue  # execution status can change
        x, y = getattr(a, key), getattr(b, key)
        if key in ("frame", "command", "autocontinue"):
            if x != y:
                return False
        else:
            # NaN is permitted by some existing mission commands.
            if math.isnan(x) and math.isnan(y):
                continue
            tolerance = 2e-7 if key in ("x_lat", "y_long") else 0.02
            if not math.isfinite(x) or not math.isfinite(y) or abs(x - y) > tolerance:
                return False
    return True


def same_mission(a, b):
    return len(a) == len(b) and all(same_item(x, y) for x, y in zip(a, b))


def landing_pair(config):
    config.validate()
    lat, lon = destination(
        config.land_lat,
        config.land_lon,
        (config.approach_heading_deg + 180) % 360,
        config.approach_distance_m,
    )
    return [
        Item(x_lat=lat, y_long=lon, z_alt=config.approach_alt),
        Item(command=21, x_lat=config.land_lat, y_long=config.land_lon, z_alt=config.land_alt),
    ]


def extend_mission(original, config):
    """Return (extended_items, already_present); never mutate the caller's list."""
    pair = landing_pair(config)
    mission = [copy_item(w) for w in original]
    if len(mission) >= 2 and same_mission(mission[-2:], pair):
        return mission, True
    if not mission:
        raise ValueError("Upload a base mission before preparing landing")
    if any(w.command == 21 for w in mission):
        raise ValueError(
            "Existing NAV_LAND differs; edit it explicitly instead of appending another"
        )
    if mission[-1].command in (17, 20, 177):
        raise ValueError(
            "Base mission ends with unlimited loiter, RTL or DO_JUMP; landing may be unreachable"
        )
    return mission + pair, False


def fingerprint(items):
    rows = []
    for w in items:
        row = []
        for k in FIELDS:
            if k == "is_current":
                continue
            v = getattr(w, k)
            if isinstance(v, float):
                v = round(v, 7) if math.isfinite(v) else str(v)
            row.append(v)
        rows.append(row)
    return hashlib.sha256(json.dumps(rows, separators=(",", ":")).encode()).hexdigest()[:20]
