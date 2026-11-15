"""Experimental finite-horizon net interception law, NOT TECS or a native flare port."""

from dataclasses import dataclass
import math
from .geometry import clip, finite


@dataclass
class GuidanceConfig:
    target_airspeed_mps: float = 15.0
    minimum_airspeed_mps: float = 10.0
    minimum_forward_speed_mps: float = 5.0
    minimum_range_m: float = 2.5
    maximum_range_m: float = 100.0
    minimum_ttc_s: float = 0.2
    lateral_gain: float = 2.0
    vertical_gain: float = 0.7
    roll_limit_deg: float = 20.0
    pitch_min_deg: float = -8.0
    pitch_max_deg: float = 12.0
    roll_rate_deg_s: float = 20.0
    pitch_rate_deg_s: float = 12.0
    throttle_trim: float = 0.4
    throttle_min: float = 0.15
    throttle_max: float = 0.7
    throttle_slew_s: float = 0.3
    speed_kp: float = 0.04
    speed_ki: float = 0.01

    def validate(self):
        if not finite(*self.__dict__.values()):
            raise ValueError("Non-finite guidance parameter")
        if not 0 < self.minimum_airspeed_mps < self.target_airspeed_mps:
            raise ValueError("Invalid airspeed limits")
        if (
            min(
                self.minimum_forward_speed_mps,
                self.minimum_range_m,
                self.minimum_ttc_s,
                self.roll_rate_deg_s,
                self.pitch_rate_deg_s,
                self.throttle_slew_s,
            )
            <= 0
        ):
            raise ValueError("Guidance limits must be positive")
        if self.maximum_range_m <= self.minimum_range_m:
            raise ValueError("Invalid range limits")
        if (
            not 0 < self.roll_limit_deg < 45
            or not -30 < self.pitch_min_deg < self.pitch_max_deg < 30
        ):
            raise ValueError("Invalid attitude envelope")
        if not 0 <= self.throttle_min <= self.throttle_trim <= self.throttle_max <= 1:
            raise ValueError("Invalid throttle envelope")
        if min(self.lateral_gain, self.vertical_gain, self.speed_kp, self.speed_ki) < 0:
            raise ValueError("Negative gain")


class VisionGuidance:
    def __init__(self, config):
        self.c = config
        config.validate()
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous = None

    def update(self, target, velocity_body, airspeed, current_roll, current_pitch, dt):
        c = self.c
        if not finite(*target, *velocity_body, airspeed, current_roll, current_pitch, dt):
            raise ValueError("Non-finite guidance input")
        if not 0 < dt <= 0.2:
            raise ValueError("Invalid controller time step")
        x, y, z = target
        vx, vy, vz = velocity_body
        if not c.minimum_range_m <= x <= c.maximum_range_m:
            raise ValueError("Net outside control range")
        if vx < c.minimum_forward_speed_mps:
            raise ValueError("Insufficient forward velocity")
        if airspeed < c.minimum_airspeed_mps:
            raise ValueError("Airspeed outside operating envelope")
        ttc = x / vx
        if ttc < c.minimum_ttc_s:
            raise ValueError("Insufficient interception time")
        # Position of net target relative to constant-velocity crossing point,
        # expressed in current body FLU coordinates (y left, z up).
        ey = y - vy * ttc
        ez = z - vz * ttc
        a_left = c.lateral_gain * ey / (ttc * ttc)
        roll = clip(
            -math.atan2(a_left, 9.80665),
            -math.radians(c.roll_limit_deg),
            math.radians(c.roll_limit_deg),
        )
        pitch = clip(
            current_pitch + math.atan2(c.vertical_gain * ez, x),
            math.radians(c.pitch_min_deg),
            math.radians(c.pitch_max_deg),
        )
        error = c.target_airspeed_mps - airspeed
        raw = c.throttle_trim + c.speed_kp * error + c.speed_ki * self.integral
        if (
            c.throttle_min < raw < c.throttle_max
            or raw >= c.throttle_max
            and error < 0
            or raw <= c.throttle_min
            and error > 0
        ):
            self.integral += error * dt
        thrust = clip(
            c.throttle_trim + c.speed_kp * error + c.speed_ki * self.integral,
            c.throttle_min,
            c.throttle_max,
        )
        if self.previous is None:
            self.previous = (current_roll, current_pitch, c.throttle_trim)
        limits = (
            math.radians(c.roll_rate_deg_s) * dt,
            math.radians(c.pitch_rate_deg_s) * dt,
            c.throttle_slew_s * dt,
        )
        output = tuple(
            clip(v, p - limit, p + limit)
            for v, p, limit in zip((roll, pitch, thrust), self.previous, limits)
        )
        # A previous measured attitude can initially be outside the final envelope.
        # Reject rather than publish an unbounded or discontinuously clamped command.
        if abs(output[0]) > math.radians(c.roll_limit_deg) or not math.radians(
            c.pitch_min_deg
        ) <= output[1] <= math.radians(c.pitch_max_deg):
            raise ValueError("Initial attitude outside final-phase envelope")
        self.previous = output
        return output, ttc, (ey, ez)
