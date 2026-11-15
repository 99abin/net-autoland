"""Pure geometry. North/east bearings; body frame is FLU."""

import math

R = 6371000.0


def finite(*values):
    return all(math.isfinite(float(v)) for v in values)


def clip(x, low, high):
    return max(low, min(high, x))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def destination(lat, lon, bearing_deg, distance):
    if not finite(lat, lon, bearing_deg, distance) or distance < 0:
        raise ValueError("Invalid destination inputs")
    if not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError("Invalid latitude/longitude")
    a, b, t = map(math.radians, (lat, lon, bearing_deg))
    d = distance / R
    a2 = math.asin(clip(math.sin(a) * math.cos(d) + math.cos(a) * math.sin(d) * math.cos(t), -1, 1))
    b2 = b + math.atan2(
        math.sin(t) * math.sin(d) * math.cos(a), math.cos(d) - math.sin(a) * math.sin(a2)
    )
    return math.degrees(a2), (math.degrees(b2) + 180) % 360 - 180


def leg_position(a_lat, a_lon, b_lat, b_lon, lat, lon):
    c = math.cos(math.radians(a_lat))
    bn, be = R * math.radians(b_lat - a_lat), R * c * wrap(math.radians(b_lon - a_lon))
    pn, pe = R * math.radians(lat - a_lat), R * c * wrap(math.radians(lon - a_lon))
    length = math.hypot(bn, be)
    if length < 1:
        raise ValueError("Landing leg too short")
    un, ue = bn / length, be / length
    return (pn * un + pe * ue) / length, pn * ue - pe * un, math.atan2(ue, un), length


def quaternion_euler(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not finite(x, y, z, w) or norm < 1e-8:
        raise ValueError("Invalid quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(clip(2 * (w * y - z * x), -1, 1)),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def euler_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def inverse_rotate(q, vector):
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-8 or not finite(*q, *vector):
        raise ValueError("Invalid rotation input")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    r = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return tuple(sum(r[j][i] * vector[j] for j in range(3)) for i in range(3))


def rotate(q, vector):
    x, y, z, w = q
    return inverse_rotate((-x, -y, -z, w), vector)


def interpolate_pose(history, stamp, max_nearest_s=0.04):
    """History entries: (seconds, position_ENU, quaternion_FLU_to_ENU)."""
    if not history:
        raise ValueError("No pose history")
    for a, b in zip(history, history[1:]):
        if a[0] <= stamp <= b[0]:
            f = (stamp - a[0]) / max(b[0] - a[0], 1e-9)
            position = tuple(x + (y - x) * f for x, y in zip(a[1], b[1]))
            qb = b[2]
            if sum(x * y for x, y in zip(a[2], qb)) < 0:
                qb = tuple(-v for v in qb)
            # Normalized interpolation is adequate over adjacent high-rate poses.
            q = tuple(x + (y - x) * f for x, y in zip(a[2], qb))
            norm = math.sqrt(sum(v * v for v in q))
            if norm < 1e-9:
                raise ValueError("Invalid pose interpolation")
            return position, tuple(v / norm for v in q)
    nearest = min(history, key=lambda p: abs(p[0] - stamp))
    if abs(nearest[0] - stamp) > max_nearest_s:
        raise ValueError("Image timestamp outside pose history")
    return nearest[1], nearest[2]


def compensate_target(target_at_image, image_pose, current_pose):
    old_position, old_q = image_pose
    new_position, new_q = current_pose
    world_offset = rotate(old_q, target_at_image)
    relative = tuple(a + b - c for a, b, c in zip(old_position, world_offset, new_position))
    return inverse_rotate(new_q, relative)
