"""Prototype terminal commitment gates; configure and validate in SITL first."""
from dataclasses import dataclass
import math
from .geometry import finite

TERMINAL_STATES = ("TERMINAL_COMMITTED", "NET_CONTACT", "MISSED_NET", "TERMINAL_FAULT")
ZERO_THRUST_STATES = ("NET_CONTACT", "MISSED_NET", "TERMINAL_FAULT", "COMPLETE")


@dataclass
class TerminalConfig:
    terminal_commit_ttc_s: float = 2.0
    terminal_decision_ttc_s: float = 1.0
    terminal_commit_stable_s: float = .15
    terminal_target_age_s: float = .25
    terminal_max_position_std_m: float = 1.0
    terminal_net_half_width_m: float = 2.75
    terminal_net_half_height_m: float = 1.0
    terminal_opening_margin_m: float = .1

    def validate(self):
        if not finite(*self.__dict__.values()) or min(self.__dict__.values()) <= 0:
            raise ValueError("Terminal limits must be finite and positive")
        if not self.terminal_commit_ttc_s > self.terminal_decision_ttc_s > .2:
            raise ValueError("Terminal decision must precede the close-range guidance boundary")
        if self.terminal_opening_margin_m >= min(self.terminal_net_half_width_m, self.terminal_net_half_height_m):
            raise ValueError("Terminal opening margin exceeds net opening")


def crossing(target, velocity):
    if not finite(*target, *velocity) or velocity[0] <= 0:
        raise ValueError("Invalid terminal crossing inputs")
    time = target[0] / velocity[0]
    return time, target[1]-velocity[1]*max(time, 0.), target[2]-velocity[2]*max(time, 0.)


def commit_ready(config, ttc, error_y, error_z, std_y, std_z, target_age):
    if not finite(ttc, error_y, error_z, std_y, std_z, target_age):
        return False
    margin = config.terminal_opening_margin_m
    return (config.terminal_decision_ttc_s < ttc <= config.terminal_commit_ttc_s
            and 0 <= target_age <= config.terminal_target_age_s
            and 0 <= max(std_y, std_z) <= config.terminal_max_position_std_m
            and min(std_y, std_z) >= 0
            and abs(error_y)+std_y <= config.terminal_net_half_width_m-margin
            and abs(error_z)+std_z <= config.terminal_net_half_height_m-margin)


def net_contact_pair(collision1, collision2):
    """Only aircraft / blue net panel contact; not ground or decorative figures."""
    def panel(name):
        return name.endswith("among_us_landing_panel::assembly::blue_panel_collision")
    def aircraft(name):
        return "merlinv2" in name.split("::")
    return (panel(collision1) and aircraft(collision2)) or (panel(collision2) and aircraft(collision1))
