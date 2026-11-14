import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class detector(Node):
    def __init__(self):
        super().__init__('detector')

        self.get_logger().info(
            'detector initialized'
        )

        self.publisher = self.create_publisher(String, '/camera/status', 10)

        self.timer = self.create_timer(1, self.timer_callback
        )

    def timer_callback(self):

        msg = String()
        msg.data = 'image sent'
        self.publisher.publish(msg)

        self.get_logger().info('message sent')



def main(args=None):
    rclpy.init(args=args)

    node = detector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()