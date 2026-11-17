"""Correct MAVROS 2.16 velocity_local's ENU data labelled as base_link.

Upstream 2.16.0 local_position.cpp lines 182-191 assigns tf_child_frame_id
to twist_local although both linear and angular vectors are ENU. No rotation
or timestamp replacement is appropriate. Only the simulation application uses
the corrected topic; raw MAVROS topics and velocity_body remain untouched.
https://github.com/mavlink/mavros/blob/2.16.0/mavros/src/plugins/local_position.cpp
"""
import copy
import time


def corrected_velocity(message):
    if message.header.frame_id not in ("map", "base_link"):
        raise ValueError("Unexpected MAVROS velocity_local frame: " + message.header.frame_id)
    corrected = copy.deepcopy(message)
    corrected.header.frame_id = "map"
    return corrected


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandLong
    rclpy.init()
    node = rclpy.create_node("simulation_velocity_frame_adapter")
    publisher = node.create_publisher(TwistStamped, "/simulation/velocity_local_enu",
                                      qos_profile_sensor_data)

    def receive(message):
        try:
            publisher.publish(corrected_velocity(message))
        except ValueError as exc:
            node.get_logger().error(str(exc), throttle_duration_sec=5.)

    node.create_subscription(TwistStamped, "/mavros/local_position/velocity_local",
                             receive, qos_profile_sensor_data)
    # The native 1 Hz streams cannot meet the controller's 0.5 s freshness
    # contract. Request rates on this MAVLink connection only, without changing
    # FCU parameters or the separate QGC connection.
    state = {"connected": False, "future": None, "index": 0, "started": time.monotonic()}
    node.create_subscription(State, "/mavros/state",
                             lambda m: state.update(connected=m.connected), 10)
    client = node.create_client(CommandLong, "/mavros/cmd/command")
    # HEARTBEAT is scheduled in FCU simulation time. At a low real-time factor
    # its default 1 Hz can exceed our 3 s wall watchdog. Request real FCU
    # heartbeats more frequently; never synthesize or renew stale state.
    intervals = [(0, 100000.), (30, 50000.), (32, 50000.), (33, 100000.), (74, 100000.)]

    def configure_rates():
        future = state["future"]
        if future is not None and future.done():
            response = future.result()
            if response and response.success:
                state["index"] += 1
            state["future"] = None
        if state["index"] == len(intervals):
            timer.cancel()
            node.get_logger().info("SITL telemetry requested: heartbeat/global/VFR 10 Hz, attitude/local 20 Hz")
            return
        if time.monotonic() - state["started"] > 60:
            raise RuntimeError("SITL telemetry-rate setup failed; inspect MAVROS command service")
        if state["future"] is None and state["connected"] and client.service_is_ready():
            message_id, period = intervals[state["index"]]
            request = CommandLong.Request()
            request.command, request.param1, request.param2 = 511, float(message_id), period
            state["future"] = client.call_async(request)

    timer = node.create_timer(.5, configure_rates)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
