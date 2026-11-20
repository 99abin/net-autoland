# Experimental terminal guidance

The original early AUTO → GUIDED handoff remains. It still requires the height/
sink trigger, NAV_LAND, approach geometry, fresh telemetry, verified parameters,
stable vision and a bounded preview. Missing vision before handoff leaves AUTO
in control. These ROS states are not ArduPilot's internal flare telemetry.

The new terminal phase addresses the old inevitable abort at 2.5 m range or
0.2 s time to target. Enable it only for reviewed simulator testing. Normal
launch defaults remain disabled; `sim.sh autoland` and `auto-test --execute`
explicitly enable it for the managed local simulator.

| Phase | Behavior |
| --- | --- |
| VISION_FINAL | Existing interception law and recovery checks. Evaluate terminal commitment. |
| TERMINAL_COMMITTED | Regularized guidance, small bank envelope, thrust ramp toward zero. |
| NET_CONTACT | Simulator blue-panel/aircraft contact; zero-thrust attitude hold. Capture unconfirmed. |
| MISSED_NET | Target plane passed without observed contact. Hold direction with zero thrust; never chase behind. |
| TERMINAL_FAULT | Invalid commands, telemetry, ownership or readiness after commitment. Zero-thrust hold; no automatic late RTL. |
| COMPLETE | Operator-confirmed stationary capture; zero-thrust hold until pilot action. |

Commitment requires time to target at most 2 s, target age at most 0.25 s,
position standard deviations at most 1 m and predicted lateral/vertical errors
plus their respective standard deviations fitting the 2.75 m / 1 m half-opening,
with 0.1 m margin. Measured and commanded bank must be within 5 degrees. These
checks must remain acceptable for 0.15 wall seconds. Failure to commit by 1 s
time to target requests ordinary recovery before commitment. Configuration is
in `src/start/config/control.yaml`; the controller and supervisor both check it.

This is a centre-crossing gate using a one-standard-deviation allowance. It does
not account for the complete aircraft swept volume or establish a statistical
capture guarantee. It must not be treated as a validated go-around boundary.

After commitment the guidance denominator uses at least a 0.7 s horizon rather
than shrinking toward zero. Actual time to target is still reported. Pitch
stays within −8 to +12 degrees; roll within 5 degrees. Existing attitude slew
limits remain. Thrust ramps toward zero at 0.3 normalized units/s. Contact,
terminal faults and a passed target demand zero immediately. The FCU interface
permits the lower throttle range only with explicit terminal enablement and
matching supervisor authorization after existing vision ownership.

The target cache continues to store a stationary position in local coordinates,
updated by each accepted visual observation. Detector gaps alone do not change
that target. Fresh aircraft pose/velocity is still needed to use it. The
terminal hold uses the last sent bounded attitude if guidance becomes invalid.
Pilot-selected modes outside AUTO/GUIDED release control. A failed process or
lost command transport cannot be made safe by this Python policy alone.

Gazebo's blue panel now has a contact sensor in the generated world. The original
aircraft/world source files and spawn pose remain unchanged. Its ROS topic is
`/simulation/net_contacts`. Ground and decorative-figure collisions do not count
as net contact. The panel remains rigid; it does not simulate net restraint,
deformation or a successful capture. Contact never automatically sets COMPLETE.
Operator confirmation also requires fresh horizontal speed below 1 m/s and
height within the configured net half-height of the landing target. Nothing
automatically arms/disarms the aircraft in the application.

For the complete test commands, see [simulation_command.md](simulation_command.md).
The recorder reports terminal commitment, close-range previews, zero-thrust
messages and net contact separately. A run ending in NET_CONTACT still returns
a nonzero test result because capture was not confirmed. The application stays
running while armed after terminal outcomes to preserve the hold. The runner
does not add a late RTL during cleanup.
