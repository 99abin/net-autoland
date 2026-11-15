# Validation report

Base: `origin/handoff` at `0b02a9b8da038cecaf5a02a592c27a7a17fb0e6f`.

Implemented: four ROS packages, seven executable nodes, six message types, one
ownership service, launch/configuration, and non-ROS tests.

Parameter revision retains the existing early handoff. It adds typed YAML profile
loading, native landing-parameter discovery, baseline backup, sequential writes,
forced FCU readback, profile export, readonly monitoring, and parameter readiness
interlocks. No post-flare/settled-sink-rate transition is introduced.

Checks completed in the creation environment:

- **84 unittest tests passed**, no skipped tests with OpenCV available.
- The full handoff/recovery pipeline test also passed when run independently.
- Parameter profile tests cover null preservation, invalid/unknown/duplicate keys,
  integer/float handling, native airspeed constraints, preview, rejected writes,
  clamped readback, baseline backups/export, arming mid-transaction, late RPCs,
  readonly monitoring in flight, drift/change events and readiness interlocks.
- Ruff undefined-name/import checks passed.
- Black formatting check passed.
- Python syntax and XML manifests parsed; YAML parameters matched node declarations.
- The blue/purple panel was detected in the supplied reference image. Approximate
  corners were (160,134), (653,131), (647,302), (167,317) pixels.
- Synthetic calibrated image → detection → metric body-frame pose test passed.
- Mission list 16→18, duplicate prevention, upload/readback comparison, and response/
  topic delivery ordering were exercised with transport doubles.
- Service-before-status message ordering, actual GUIDED confirmation, watchdog recovery,
  pilot override, and stationary capture event were exercised with doubles.

Test runtime: Python 3.12, NumPy, OpenCV 5.0.0. Target build recipe is ROS 2 Humble /
Ubuntu 22.04 / Python 3.10; code uses Python 3.10-compatible syntax. OpenCV APIs are
selected from the 4.x-compatible API, but the Ubuntu packaged version was not run.

Not verified here:

- colcon build, generated ROSIDL code, ROS discovery/QoS or installed entry points;
- Docker build (Docker is unavailable);
- live MAVROS/FCU mode and mission transactions;
- firmware-specific attitude mask on the user's actual ArduPlane version;
- camera calibration, real lighting/occlusions, airspeed health;
- closed-loop SITL dynamics, terminal capture/impact, real aircraft flight.

The ROS node tests use transport/message doubles. They are useful regression tests,
not a substitute for the outstanding integration and flight-dynamics checks.
