"""Calibrated centre ray / surveyed vertical plane, independent of ROS."""
import math
import cv2
import numpy as np

EARTH_RADIUS_M = 6371000.


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError("Invalid aircraft attitude quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def net_offset_enu(latitude, longitude, net_latitude, net_longitude):
    values = (latitude, longitude, net_latitude, net_longitude)
    if (not np.isfinite(values).all() or abs(latitude) >= 89 or abs(net_latitude) >= 89
            or abs(longitude) > 180 or abs(net_longitude) > 180):
        raise ValueError("Invalid geographic net/aircraft position")
    delta_lon = (math.radians(net_longitude-longitude) + math.pi) % (2*math.pi) - math.pi
    east = EARTH_RADIUS_M * math.cos(math.radians((latitude+net_latitude)/2)) * delta_lon
    north = EARTH_RADIUS_M * math.radians(net_latitude-latitude)
    return np.array([east, north])


def sample_bracket(history, stamp, max_gap=.25, max_nearest=.08):
    history = list(history)
    if not history or not math.isfinite(stamp):
        raise ValueError("No timestamped aircraft history")
    for a, b in zip(history, history[1:]):
        if a[0] <= stamp <= b[0]:
            if b[0] - a[0] > max_gap:
                raise ValueError("Aircraft history gap around image timestamp")
            return a, b, (stamp-a[0]) / max(b[0]-a[0], 1e-9)
    nearest = min(history, key=lambda entry: abs(entry[0]-stamp))
    if abs(nearest[0]-stamp) > max_nearest:
        raise ValueError("Image timestamp outside aircraft history")
    return nearest, nearest, 0.


def image_pose(history, stamp):
    a, b, weight = sample_bracket(history, stamp)
    position = (1-weight)*np.asarray(a[1]) + weight*np.asarray(b[1])
    qa, qb = np.asarray(a[2]), np.asarray(b[2])
    if qa @ qb < 0:
        qb = -qb
    q = (1-weight)*qa + weight*qb
    if not np.isfinite(position).all():
        raise ValueError("Invalid aircraft position")
    return position, quaternion_matrix(q)


def plane_measurement(center, camera_matrix, distortion, camera_to_body,
                      camera_translation_body, body_to_local, offset_enu,
                      heading_deg, gps_covariance, pixel_noise=2.,
                      attitude_std_deg=1., plane_position_std=.5,
                      plane_heading_std_deg=1., min_incidence=.2, max_range=200.):
    """Return body position, LOCAL covariance and ray/normal incidence.

    The plane passes through surveyed net GPS and has a horizontal normal in
    approach-heading direction. It is infinite vertically: no altitude-plane
    intersection is used. Covariance is a local first-order approximation.
    """
    uv = np.asarray(center, float).reshape(2)
    k = np.asarray(camera_matrix, float).reshape(3, 3)
    distortion = np.asarray(distortion, float)
    mount = np.asarray(camera_to_body, float).reshape(3, 3)
    translation = np.asarray(camera_translation_body, float).reshape(3)
    rotation = np.asarray(body_to_local, float).reshape(3, 3)
    offset = np.asarray(offset_enu, float).reshape(2)
    gps_covariance = np.asarray(gps_covariance, float).reshape(2, 2)
    if not all(np.isfinite(x).all() for x in (uv, k, distortion, mount, translation, rotation, offset, gps_covariance)):
        raise ValueError("Non-finite ray/plane input")
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("CameraInfo is uncalibrated")
    for transform in (mount, rotation):
        if not np.allclose(transform.T @ transform, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(transform), 1):
            raise ValueError("Camera/aircraft transform must be a proper rotation")
    if (not np.isfinite([heading_deg, pixel_noise, attitude_std_deg, plane_position_std,
                         plane_heading_std_deg, min_incidence, max_range]).all()
            or not 0 <= heading_deg < 360 or min(pixel_noise, attitude_std_deg, plane_position_std, plane_heading_std_deg) < 0
            or not 0 < min_incidence < 1 or max_range <= 0):
        raise ValueError("Invalid ray/plane uncertainty or geometry limits")
    if not np.allclose(gps_covariance, gps_covariance.T) or np.min(np.linalg.eigvalsh(gps_covariance)) < -1e-9:
        raise ValueError("Invalid GPS covariance")

    def point(pixel, attitude, displacement, heading):
        xy = cv2.undistortPoints(np.asarray(pixel).reshape(1, 1, 2), k, distortion).reshape(2)
        ray = attitude @ mount @ np.array([xy[0], xy[1], 1.])
        ray /= np.linalg.norm(ray)
        normal = np.array([math.sin(heading), math.cos(heading), 0.])
        incidence = float(normal @ ray)
        if abs(incidence) < min_incidence:
            raise ValueError("Viewing ray nearly parallel to vertical net plane")
        camera = attitude @ translation
        distance = (normal @ (np.r_[displacement, 0.] - camera)) / incidence
        if not 0 < distance <= max_range:
            raise ValueError("Vertical-plane intersection behind camera or beyond range")
        local = camera + distance*ray
        body = attitude.T @ local
        if body[0] <= 0:
            raise ValueError("Target not ahead of aircraft")
        return body, local, abs(incidence)

    heading = math.radians(heading_deg)
    body, local, incidence = point(uv, rotation, offset, heading)
    jacobian = np.zeros((3, 8))
    for axis in range(8):
        pixel, attitude, displacement, angle = uv.copy(), rotation, offset.copy(), heading
        step = 1e-3 if axis < 2 or axis in (5, 6) else 1e-5
        if axis < 2:
            pixel[axis] += step
        elif axis < 5:
            perturbation = np.zeros(3)
            perturbation[axis-2] = step
            attitude = cv2.Rodrigues(perturbation)[0] @ rotation
        elif axis < 7:
            displacement[axis-5] += step
        else:
            angle += step
        jacobian[:, axis] = (point(pixel, attitude, displacement, angle)[1]-local)/step
    inputs = np.zeros((8, 8))
    inputs[:2, :2] = np.eye(2)*pixel_noise**2
    inputs[2:5, 2:5] = np.eye(3)*math.radians(attitude_std_deg)**2
    inputs[5:7, 5:7] = gps_covariance + np.eye(2)*plane_position_std**2
    inputs[7, 7] = math.radians(plane_heading_std_deg)**2
    covariance = jacobian @ inputs @ jacobian.T
    return body, (covariance+covariance.T)/2, incidence


class StationaryTargetFilter:
    """Static 3D Kalman filter; rejected observations never renew its timestamp."""
    def __init__(self, process_std=.02, min_std=.75, gate=11.345, max_jump=3.):
        if not np.isfinite([process_std, min_std, gate, max_jump]).all() or min(process_std, min_std) < 0 or min(gate, max_jump) <= 0:
            raise ValueError("Invalid stationary-target filter limits")
        self.process_std, self.min_std, self.gate, self.max_jump = process_std, min_std, gate, max_jump
        self.reset()

    def reset(self):
        self.position = self.covariance = self.stamp = None

    def floor(self, covariance):
        values, vectors = np.linalg.eigh((covariance+covariance.T)/2)
        if values.min() < -1e-8:
            raise ValueError("Invalid position covariance")
        return (vectors*np.maximum(values, self.min_std**2)) @ vectors.T

    def update(self, measurement, covariance, stamp):
        measurement, covariance = np.asarray(measurement, float), np.asarray(covariance, float)
        if measurement.shape != (3,) or covariance.shape != (3, 3) or not all(np.isfinite(x).all() for x in (measurement, covariance, stamp)):
            raise ValueError("Invalid target filter observation")
        if not np.allclose(covariance, covariance.T):
            raise ValueError("Invalid position covariance")
        covariance = self.floor(covariance)
        if self.position is None:
            self.position, self.covariance, self.stamp = measurement.copy(), covariance, stamp
            return self.position.copy(), self.covariance.copy()
        if stamp <= self.stamp:
            raise ValueError("Duplicate or out-of-order target observation")
        prediction = self.covariance + np.eye(3)*self.process_std**2*(stamp-self.stamp)
        innovation = measurement-self.position
        residual = prediction+covariance
        if (np.linalg.norm(innovation) > self.max_jump
                or innovation @ np.linalg.solve(residual, innovation) > self.gate):
            raise ValueError("Net centre rejected by stationary-target innovation gate")
        gain = np.linalg.solve(residual, prediction.T).T
        updated = self.position + gain @ innovation
        identity = np.eye(3)-gain
        updated_cov = self.floor(identity @ prediction @ identity.T + gain @ covariance @ gain.T)
        self.position, self.covariance, self.stamp = updated, updated_cov, stamp
        return updated.copy(), updated_cov.copy()
