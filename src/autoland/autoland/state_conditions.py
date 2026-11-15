"""Pure supervisor decisions. These are not ArduPilot's internal phase flags."""

from dataclasses import dataclass
import math


@dataclass
class HandoffConfig:
    flare_alt_m: float = 3.0
    flare_time_s: float = 2.0
    handoff_margin_m: float = 1.5
    handoff_margin_s: float = 0.5
    max_cross_track_m: float = 5.0
    max_track_error_deg: float = 15.0
    min_net_range_m: float = 8.0
    max_net_range_m: float = 80.0

    def validate(self):
        if not all(math.isfinite(v) and v >= 0 for v in self.__dict__.values()):
            raise ValueError("Handoff thresholds must be finite and nonnegative")
        if self.min_net_range_m >= self.max_net_range_m:
            raise ValueError("Invalid net range limits")


def in_handoff_window(c, height, sink, progress):
    if not all(math.isfinite(v) for v in (height, sink, progress)):
        return False
    trigger = height <= c.flare_alt_m + c.handoff_margin_m
    trigger |= (
        c.flare_time_s > 0 and sink > 0 and height <= sink * (c.flare_time_s + c.handoff_margin_s)
    )
    return bool(trigger and sink >= 0 and progress > 0.5)


def handoff_ready(c, *, height, sink, progress, cross, track_error, net_range, vision_ready):
    if not all(math.isfinite(v) for v in (height, sink, progress, cross, track_error, net_range)):
        return False
    return bool(
        in_handoff_window(c, height, sink, progress)
        and sink >= 0
        and 0.5 < progress < 1
        and abs(cross) <= c.max_cross_track_m
        and abs(math.degrees(track_error)) <= c.max_track_error_deg
        and c.min_net_range_m <= net_range <= c.max_net_range_m
        and vision_ready
    )


class Supervisor:
    def __init__(self):
        self.state = "WAITING"
        self.enabled = False
        self.reason = "Prepare and verify mission"

    def enable(self, verified):
        if not verified or self.state in ("ABORT", "COMPLETE", "VISION_FINAL", "HANDOFF_PENDING"):
            return False
        self.enabled = True
        self.state = "MISSION_READY"
        self.reason = "Monitoring enabled"
        return True

    def abort(self, reason):
        self.state = "ABORT"
        self.enabled = False
        self.reason = reason

    def step(
        self,
        *,
        verified,
        telemetry_ok,
        mode,
        armed,
        landing_active,
        ready,
        allow_handoff,
        owned,
        missed_window=False,
    ):
        if self.state in ("ABORT", "COMPLETE"):
            return None
        if not self.enabled:
            self.state = "MISSION_READY" if verified else "WAITING"
            return None
        if not verified or not telemetry_ok:
            self.abort("Mission or telemetry unavailable")
            return "ABORT"
        if not armed:
            if self.state in ("HANDOFF_PENDING", "VISION_FINAL"):
                self.abort("Aircraft disarmed during active control")
                return "RELEASE"
            self.state = "MISSION_READY"
            return None
        if self.state == "HANDOFF_PENDING":
            if mode == "GUIDED" and owned:
                self.state = "VISION_FINAL"
                self.reason = "GUIDED and ownership confirmed"
            elif mode not in ("AUTO", "GUIDED"):
                self.abort("Pilot/FCU changed mode during handoff")
                return "RELEASE"
            return None
        if self.state == "VISION_FINAL":
            if mode != "GUIDED" or not owned:
                self.abort("Vision control ownership lost")
                return "RELEASE"
            return None
        if mode == "AUTO" and landing_active:
            self.state = "AUTO_APPROACH"
            self.reason = "Native ArduPlane approach"
            if allow_handoff and missed_window and not ready:
                self.abort("Handoff boundary reached without valid aligned vision control")
                return "ABORT"
            if ready:
                self.reason = (
                    "Handoff ready (preview)"
                    if not allow_handoff
                    else "Requesting vision ownership"
                )
                if allow_handoff:
                    self.state = "HANDOFF_PENDING"
                    return "VISION"
        elif self.state == "AUTO_APPROACH":
            self.abort("Left intended AUTO landing leg")
            return "RELEASE"
        return None
