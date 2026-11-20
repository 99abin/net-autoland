# Wind-aware airborne landing preparation

The opt-in airborne planner preserves your original mission and appends four
items: **Adjustment → Alignment → Approach → LAND**. Normal launches remain
preflight-only, with mission writes, handoff and live actuation disabled by
default. The existing early handoff, vision readiness and terminal guidance
checks are unchanged. No measured-alignment gate has been added.

## Configuration

Edit `src/start/config/mission.yaml` under `mission_manager.ros__parameters`:

| Setting | Default | Meaning |
|---|---:|---|
| `t_reference_s` | 33.3333333333 s | Desired nominal APP-to-net flight duration, tune from flight measurements |
| `minimum_route_time_s` | 30 s | Required predicted time to the last **original** waypoint before each append |
| `turn_radius_m` | 60 m | Planning-circle radius; must be reviewed against aircraft turning performance |
| `alignment_straight_time_s` | 8 s | Planned straight flight allowance before APP |
| `alignment_margin_m` | 20 m | Additional alignment distance |
| `alignment_distance_min_m` / `alignment_distance_max_m` | 50 / 1000 m | Bounds for the alignment setback solution |
| `approach_alt` | 50 m | Adjustment, Alignment and Approach altitude above HOME |
| `land_alt` | 1.26 m | Net centre relative to this SITL HOME |
| `approach_heading_deg` | 180° | Travel direction toward the net; here south |

The example settings are provisional, not an aircraft-specific tune. The
original aircraft spawn and original waypoint coordinates are preserved.
`approach_distance_m: 500` still controls ordinary static preflight preparation;
airborne mode replaces it with the integrated distance described below.

## Remaining original route time

END is the last original waypoint captured before takeoff. There is no fixed
start waypoint index or append-deadline index, and END no longer has to be
behind APP or within a fixed lateral corridor. After takeoff, when fresh armed
AUTO telemetry, a verified profile and stable wind are available, prediction
follows:

```
current GPS → active original waypoint → next original waypoints → END
remaining time = sum(each leg distance / predicted groundspeed on that leg)
```

For example, while flying from WP5 to WP6 in a mission ending at WP9, include
current→WP6, WP6→WP7, WP7→WP8 and WP8→WP9. Do not include the new landing items.
Each leg projects wind onto its own direction; it does not divide total route
distance by one observed GPS speed. Cruise speed comes from the verified FCU
`AIRSPEED_CRUISE`, read without changing it. This equivalent airspeed is converted
to true airspeed using ISA troposphere density at the fresh VFR HUD AMSL altitude.
That is an atmospheric approximation, not measured density; GPS ellipsoid
height and HOME-relative height are not used as AMSL substitutes.

The 30 s preparation budget is configurable and is checked again before each
new item is transmitted. It is separate from `t_reference_s`. Waypoint acceptance
radii, corner cutting, speed errors and atmospheric errors affect actual time;
this straight-leg forecast is not a guaranteed time bound. Preparation refuses
writes after the original mission has already advanced into appended indices.
A failed/expired airborne preparation requests RTL through the opted-in FCU
interface when still armed in AUTO; it does not silently retry partial writes.
Pilot-selected modes are respected.

## Approach distance and alignment geometry

For a track direction h and wind pointing toward east/north:

```
W_along = W_east*sin(h) + W_north*cos(h)
W_cross = W_east*cos(h) - W_north*sin(h)
V_ground = sqrt(V_TAS² - W_cross²) + W_along
APP distance = integral(V_ground(t), 0 .. t_reference_s)
```

The YAML arrays `airspeed_profile_time_fractions` and
`airspeed_profile_tas_mps` define a piecewise-linear **true** airspeed profile.
Each segment uses Simpson integration with 128 intervals. The illustrative
profile `[15.45, 15.45, 13.70]` at fractions `[0, 0.8, 1]` predicts about
509.17 m in calm wind. A constant 15 m/s TAS with a 3 m/s tailwind predicts
600 m for 33.333 s. This plans a duration; it does not force native TECS to
achieve that duration or prove net capture.

Two circles, left and right, are tangent to the landing line at **Alignment**.
For each side the planner finds a tangent from END; its contact point is
**Adjustment**. It selects the shortest feasible reference path. Alignment,
Approach and LAND are collinear. Alignment and its circle are solved together:

```
D_alignment = R*tan(abs(delta)/2) + Vg_alignment*T_straight + margin
```

`delta` is the direction change from Adjustment→Alignment to Alignment→Approach.
`Vg_alignment` uses FCU cruise TAS and wind along the landing direction. A
bounded root search rejects near-reversals and solutions outside the YAML
bounds. If END already lies suitably behind Alignment on the landing line,
the planner uses a straight entry instead of making an unnecessary loop.

These are ordinary NAV_WAYPOINTs: ArduPlane L1 controls the actual turns and can
cut the reference circle. Four waypoints do not guarantee exact tangency or
measured alignment. There is no new measured-alignment condition. Terrain,
fences and obstacle clearance still require review (`adaptive_corridor_reviewed`).

## Wind and transaction checks

Wind uses sensor-data QoS and an 8 s ROS reception window, at least 20 samples,
vector standard deviation ≤1.0 m/s, receipt freshness ≤2 s and magnitude ≤10 m/s.
No direction-diversity condition remains. Native WIND has no measurement
timestamp; the incoming MAVROS header is retained in recordings, while the
window uses local ROS receipt time and a monotonic receipt lease. Stable wind
does not prove an accurate wind estimate. Image timestamps remain unchanged.
Predicted groundspeeds must exceed 3 m/s and planned average APP-to-LAND sink
must be below 3 m/s. Infeasible wind, distance or geometry is rejected.

Before takeoff the manager pulls and fingerprints HOME, exactly one TAKEOFF
and ordinary WAYPOINTs, with no existing LAND/RTL/loiter. It then waits for
airborne observations at least 20 m above HOME. Once the new plan is frozen:

1. Pull again and check the original mission is unchanged.
2. Save an atomic journal with the base, expected mission, wind and calculation.
3. Send `MISSION_WRITE_PARTIAL_LIST`, start=end=N, then **only Adjustment**.
4. Pull and verify the entire original prefix plus this new item.
5. Repeat independently for Alignment at N+1, Approach at N+2 and LAND at N+3.
6. Publish a verified LandingPlan only after all four readbacks succeed.

The existing MAVROS router carries companion 1.192 ↔ FCU 1.1 messages. INT item
encoding preserves GPS precision. There is no airborne full mission upload,
mission clear or set-current call. A lost ACK is resolved by readback, not a
blind retransmission. Four appends are not atomic. Partial failures preserve
the journal and invalidate readiness. Restarting with a journal requires ground
inspection. `adaptive-base --apply`, while disarmed MANUAL, can reconcile a
matching full/partial journal (including older two-item journals) and archive it.
It removes a recognized static landing pair if necessary; it **does not add a
staging waypoint** or change your original route. Unexpected missions are refused.

## Headless full test

See [simulation_command.md](simulation_command.md) for commands. The automated
test explicitly enables managed local SITL arming, AUTO, finite catapult launch,
mission append, handoff and live output. It enables the supervisor only after
all four appended items are verified. It leaves the existing TECS tuning and
vision controller unchanged. A successful append is distinct from a successful
net interception; this feature does not establish flight readiness.

Status: `/autoland/airborne_preparation`. It reports wind, route legs/times,
cruise EAS/TAS and ISA assumption, alignment geometry, `end_index`, appended
coordinates and `verified_appends` from 0 through 4. The simulator journal is
`.simulation/adaptive_plan.json`; recordings are `.simulation/flight_runs/`.

## Observed test result

The 19 November headless repeat verified all four airborne appends with the
original six-item prefix unchanged. It subsequently lost stability in native
AUTO while turning toward Adjustment, before reaching the landing approach.
No ROS live commands or GUIDED handoff occurred. Full net landing remains
unsuccessful; details and artifacts are in [VALIDATION.md](VALIDATION.md).
