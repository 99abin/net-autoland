# Centre-ray estimator and stationary-target filter

The runtime pipeline is now:

```text
calibrated image -> colour-region centre pixel
  -> undistorted camera ray + image-time aircraft/GPS history
  -> intersection with surveyed VERTICAL net plane
  -> uncertainty checks -> stationary-position Kalman filter in local ENU
  -> body-frame NetEstimate at the ORIGINAL image timestamp
  -> existing preview / conditional GUIDED handoff / retained-target guidance
```

## Plane geometry

The verified landing plan supplies net latitude/longitude and travel heading.
The vertical plane passes through that surveyed location and is perpendicular
to `approach_heading_deg`. The actual net must face that approach. A northward
approach uses a north-facing plane normal; an eastward approach an east-facing
normal. The sign of the normal does not change the plane.

For camera position C, calibrated local viewing direction d, plane point P and
unit normal n, the intersection is C + t d, with:

```text
t = dot(n, P-C) / dot(n, d)
```

The normal is horizontal: `[sin(heading), cos(heading), 0]` in east/north/up
coordinates. There is no division by the ray's vertical component. A level ray
can therefore intersect this vertical plane even at the net's centre altitude.
Parallel/grazing rays, intersections behind the camera, and excessive ranges
are rejected. Native `land_alt` still configures the mission; the observed centre
height is obtained from the ray, not clamped to LAND altitude.

The estimator uses local position/attitude and GPS at image time. GPS-to-plane
offsets are transported between GPS and image timestamps using measured local
motion, avoiding a range error from holding an old GPS vector. This assumes
local ENU and the GPS map are aligned and the camera mounting transform is
correct. It uses real MAVROS telemetry, not simulator ground-truth target pose.

## What “centre detection” currently means

`net_det` uses the centroid of the largest acceptable blue/purple colour region.
It no longer requires a quadrilateral or four corner correspondences. Area,
colour, fill and aspect checks remain; image-border-clipped regions are rejected.
It is not a trained centre-keypoint model or a marker detector. Interior
occlusion, perspective distortion, asymmetric colouring and another similarly
coloured object can still bias or misidentify this centre. Filtering cannot
prove object identity or correct a consistently biased observation. A visible
physical centre feature or trained centre detector can later publish the same
NetDetection centre-pixel interface.

## Filtering and uncertainty

The position filter assumes a stationary net, with a 3D position state in the
local map frame. Measurements with higher covariance receive less weight.
Defaults in `src/start/config/perception.yaml`:

- Pixel noise: 2 pixels; attitude error budget: 1 degree.
- Local-position and survey-position error budgets: 0.5 m each.
- GPS covariance must be supplied; an additional 0.5 m floor is applied.
- Surveyed plane-heading error budget: 1 degree.
- Minimum absolute ray/normal cosine: 0.2; maximum intersection range: 200 m.
- Maximum position standard deviation: 2 m.
- Filter process noise: 0.02 m per square-root second.
- Minimum filtered standard deviation: 0.75 m, to limit overconfidence.
- Innovation Mahalanobis threshold: 11.345; maximum measurement jump: 3 m.

These are assumed prototype error budgets, not validated sensor accuracy.
Calibration bias, false association and unannounced local-frame origin jumps
are not fully modeled. GPS is mandatory for this estimator; it is no longer
an independent monocular size-based range estimate.

Rejected observations publish `valid: false`. They do not renew the accepted
target's image timestamp. Missing images do not generate fresh valid predictions.
The original 0.25 s freshness and 0.5 s pre-handoff stability rules are unchanged.
After handoff authorization, the existing controller can retain the last
accepted stationary target through gaps. Plan/calibration/clock/invalid-frame
resets increment tracking_epoch so incompatible held targets are cleared too.
Existing telemetry, command, range and time-to-contact guards still apply.

## Message and workflow changes

NetDetection now carries `center_u`/`center_v`, image
width/height, area and score. It no longer carries four corners. NetEstimate
carries plan_id, tracking_epoch, estimation_method and ray_plane_incidence.
Its position is in body FLU at image time; net orientation is not measured.
The old PnP reprojection-error field is removed. The old pure PnP implementation
remains only as an offline comparison/regression helper, not the runtime path.

Rebuild every interface consumer before starting the application:

```bash
cd /home/abin/td/autoland1
bash tools/build.sh
```

Use [simulation_command.md](simulation_command.md) for the flight workflow. The
same verified mission and native speed configuration are used. All ordinary
launch defaults remain disabled for writes, uploads, handoff and live output.
This change does not resolve camera/processing/message-delivery delays by itself.
It adds no terminal commitment, contact detection or net-impact motor cutoff.

## Recorded verification

All 142 Python tests passed, all four ROS packages built, and an isolated real
ROS publish/subscribe smoke test recovered the expected 30 m target. The
full Gazebo/SITL test `20261117-105653-25990` generated 20 valid filtered
estimates, confirmed GUIDED/vision ownership, published 29 live attitude
messages and used the held target in 15 valid previews. It then aborted at the
existing 0.2 s minimum interception-time guard; RTL telemetry was confirmed.
No controlled capture or safe physical go-around was established. This verifies
the estimator/handoff/holding path in a simulator run, not flight readiness.
