"""Metric planar pose from calibrated camera and known physical rectangle size."""

import cv2
import numpy as np


def rotation_quaternion(r):
    # Stable matrix -> (x,y,z,w), including rotations near 180 degrees.
    a = np.asarray(r, dtype=float)
    trace = np.trace(a)
    if trace > 0:
        s = np.sqrt(trace + 1) * 2
        q = [(a[2, 1] - a[1, 2]) / s, (a[0, 2] - a[2, 0]) / s, (a[1, 0] - a[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(a)))
        j = (i + 1) % 3
        k = (i + 2) % 3
        s = np.sqrt(max(1 + a[i, i] - a[j, j] - a[k, k], 0)) * 2
        if s < 1e-10:
            raise ValueError("Degenerate rotation")
        q = [0.0, 0.0, 0.0, 0.0]
        q[i] = s / 4
        q[j] = (a[j, i] + a[i, j]) / s
        q[k] = (a[k, i] + a[i, k]) / s
        q[3] = (a[k, j] - a[j, k]) / s
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def estimate_pose(
    corners,
    k,
    distortion,
    width_m,
    height_m,
    camera_to_body,
    translation_body,
    target_offset=(0, 0, 0),
    pixel_noise=2.0,
    max_reprojection=4.0,
):
    k = np.asarray(k, dtype=np.float64).reshape(3, 3)
    uv = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    d = np.asarray(distortion, dtype=np.float64)
    transform = np.asarray(camera_to_body, dtype=float).reshape(3, 3)
    if not all(np.isfinite(v).all() for v in (k, uv, d, transform)):
        raise ValueError("Non-finite calibration/detection")
    if k[0, 0] <= 0 or k[1, 1] <= 0 or width_m <= 0 or height_m <= 0:
        raise ValueError("Invalid calibration or net dimensions")
    if not np.allclose(transform.T @ transform, np.eye(3), atol=1e-6) or not np.isclose(
        np.linalg.det(transform), 1
    ):
        raise ValueError("camera_to_body must be a proper rotation matrix")
    obj = np.array(
        [
            [-width_m / 2, -height_m / 2, 0],
            [width_m / 2, -height_m / 2, 0],
            [width_m / 2, height_m / 2, 0],
            [-width_m / 2, height_m / 2, 0],
        ],
        dtype=np.float64,
    )
    result = cv2.solvePnPGeneric(obj, uv, k, d, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if result[0]:
        for rv, tv in zip(result[1], result[2]):
            if not np.isfinite(rv).all() or not np.isfinite(tv).all():
                continue
            rotation, _ = cv2.Rodrigues(rv)
            if np.min((rotation @ obj.T + tv.reshape(3, 1))[2]) <= 0:
                continue
            reproj, jac = cv2.projectPoints(obj, rv, tv, k, d)
            error = float(np.sqrt(np.mean(np.sum((reproj.reshape(4, 2) - uv) ** 2, axis=1))))
            candidates.append((error, rotation, tv.reshape(3), jac[:, :6]))
    if not candidates:
        raise ValueError("No positive-depth planar pose")
    error, rotation, t, jac = min(candidates, key=lambda x: x[0])
    if error > max_reprojection:
        raise ValueError("Reprojection error too high")
    offset = np.asarray(target_offset, dtype=float)
    if not np.isfinite(offset).all() or not np.isfinite(translation_body).all():
        raise ValueError("Invalid target/camera offset")
    point_camera = t + rotation @ offset
    body_position = transform @ point_camera + np.asarray(translation_body)
    body_rotation = transform @ rotation
    # Local image-noise covariance; excludes calibration/dimension/association errors.
    information = jac.T @ jac
    if np.linalg.matrix_rank(information) < 6:
        raise ValueError("Unobservable planar pose")
    covariance6 = np.linalg.pinv(information) * max(pixel_noise, error) ** 2
    # Offset target point sensitivity to rvec, computed numerically.
    rv, _ = cv2.Rodrigues(rotation)
    sensitivity = np.zeros((3, 6))
    for axis in range(3):
        perturbed = rv.copy()
        perturbed[axis, 0] += 1e-6
        r2, _ = cv2.Rodrigues(perturbed)
        sensitivity[:, axis] = (r2 @ offset - rotation @ offset) / 1e-6
    sensitivity[:, 3:] = np.eye(3)
    covariance = transform @ sensitivity @ covariance6 @ sensitivity.T @ transform.T
    if body_position[0] <= 0:
        raise ValueError("Target not ahead of aircraft")
    return body_position, rotation_quaternion(body_rotation), covariance, error
