"""Small ROS plumbing helpers; the decision/calculation modules have no ROS imports."""

import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def retained():
    return QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
    )


def params(node, defaults):
    for k, v in defaults.items():
        node.declare_parameter(k, v)
    return {k: node.get_parameter(k).value for k in defaults}


def stamp_age(node, stamp):
    if stamp.sec == 0 and stamp.nanosec == 0:
        return float("inf")
    return (node.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) / 1e9


def recent(received, timeout):
    return received is not None and 0 <= time.monotonic() - received <= timeout


def stamped_recent(node, msg, received, timeout):
    return (
        msg is not None
        and recent(received, timeout)
        and 0 <= stamp_age(node, msg.header.stamp) <= timeout
    )


def run(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            timing = getattr(node, "timing", None)
            if timing is not None:
                timing.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def profile_values(node, status, received):
    """Return a verified profile lease, or None. No bypass for preview consumers."""
    import json
    from .parameter_profile import profile_id, validate_effective

    if not stamped_recent(node, status, received, 3.0) or not status.verified:
        return None
    try:
        values = json.loads(status.parameters_json)
        validate_effective(values)
        if profile_id(values) != status.profile_id:
            return None
        return values
    except (ValueError, TypeError):
        return None


def reordered_profile_heartbeat(node, incoming, current, received):
    """Drop only an identical heartbeat arriving just before its /clock tick.

    Independent DDS topics have no cross-topic ordering. Keep the preceding
    strictly fresh lease; never accept a future-dated status, extend the old
    receipt time, or suppress changed/invalid profile content.
    """
    age=stamp_age(node,incoming.header.stamp)
    return (current is not None and incoming.verified and current.verified
        and incoming.profile_id==current.profile_id
        and incoming.parameters_json==current.parameters_json
        and -.05<=age<0 and profile_values(node,current,received) is not None)
