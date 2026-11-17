"""Parameter trials cannot alter arming settings or accept invalid native gains."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from simulation_parameters import load_trial, validate_landing_target


class PitchTrialTests(unittest.TestCase):
    def test_requested_preflare_speed_and_restore_respect_native_limits(self):
        current = dict(LAND_OPTIONS=2, TECS_LAND_ARSPD=23,
                       AIRSPEED_MIN=13, AIRSPEED_MAX=23, AIRSPEED_CRUISE=16)
        validate_landing_target(dict(LAND_PF_ARSPD=13.3), current)
        validate_landing_target(dict(LAND_PF_ARSPD=0), current)
        for value in (12., 24.):
            with self.assertRaisesRegex(ValueError, "Preflare.*limits"):
                validate_landing_target(dict(LAND_PF_ARSPD=value), current)
        # Do not lower a stricter minimum merely to apply the requested target.
        with self.assertRaisesRegex(ValueError, "Preflare.*limits"):
            validate_landing_target(dict(LAND_PF_ARSPD=13.3), {**current, "AIRSPEED_MIN": 13.5})

    def test_landing_speed_cannot_be_silently_capped_or_exceed_limits(self):
        current = dict(LAND_OPTIONS=0, TECS_LAND_ARSPD=-1,
                       AIRSPEED_MIN=13, AIRSPEED_MAX=23, AIRSPEED_CRUISE=16)
        with self.assertRaisesRegex(ValueError, "bit 1"):
            validate_landing_target(dict(TECS_LAND_ARSPD=23), current)
        validate_landing_target(dict(TECS_LAND_ARSPD=23, LAND_OPTIONS=2), current)
        with self.assertRaisesRegex(ValueError, "limits"):
            validate_landing_target(dict(TECS_LAND_ARSPD=24, LAND_OPTIONS=2), current)
        with self.assertRaisesRegex(ValueError, "other bits"):
            validate_landing_target(dict(LAND_OPTIONS=3), current)

    def read(self, text):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.yaml"
            path.write_text(text)
            return load_trial(path)

    def test_explicit_pitch_profile_and_native_float32_baseline_are_supported(self):
        self.assertEqual(self.read("simulator_only: true\nparameters: {PTCH_RATE_FF: 0.6}"),
                         {"PTCH_RATE_FF": .6})
        self.assertEqual(self.read("simulator_only: true\nparameters: {PTCH_RATE_I: 0.6000000238418579}")["PTCH_RATE_I"],
                         .6000000238418579)

    def test_unreviewed_arming_or_invalid_gain_profiles_are_rejected(self):
        for text in (
            "parameters: {PTCH_RATE_FF: 0.6}",
            "simulator_only: true\nparameters: {ARMING_CHECK: 0}",
            "simulator_only: true\nparameters: {PTCH_RATE_FF: .nan}",
            "simulator_only: true\nparameters: {PTCH_RATE_I: 0.61}",
            "simulator_only: true\nparameters: {PTCH_RATE_FF: true}",
            "simulator_only: true\nparameters: {}",
        ):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.read(text)
