"""Sample Gazebo's clock for MAVROS without altering simulation timestamps.

Gazebo publishes at the 1 kHz physics rate. Each MAVROS plugin owns a ROS clock;
feeding all of them every physics tick overloads the Humble executor. Application
and camera nodes use the original /simulation/clock. MAVROS plugins ignore global
topic remaps, so this relay publishes their sampled clock on the standard /clock.
"""

import rclpy
from rclpy.clock import Clock as ROSClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class SimulatorClock(Node):
    def __init__(self):
        super().__init__("mavros_simulation_clock")
        self.latest = None
        self.published = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Clock, "/clock", qos)
        self.subscription = self.create_subscription(Clock, "/simulation/clock", self.receive, qos)
        self.timer = self.create_timer(
            0.02, self.publish_latest, clock=ROSClock(clock_type=ClockType.STEADY_TIME)
        )

    def receive(self, message):
        self.latest = message

    def publish_latest(self):
        if self.latest is None:
            return
        stamp = (self.latest.clock.sec, self.latest.clock.nanosec)
        if stamp != self.published:
            self.publisher.publish(self.latest)
            self.published = stamp


def main():
    rclpy.init()
    node = SimulatorClock()
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
