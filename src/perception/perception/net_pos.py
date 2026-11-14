import rclpy
from std_msgs.msg import String
from rclpy.node import Node

class processor(Node):
    def __init__(self):
        super().__init__('processor')

        self.get_logger().info('processor initialized')

        self.subscriber = self.create_subscription(String, '/camera/status', self.processor_callback, 10)

    def processor_callback(self, msg):
        self.get_logger().info(f'message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)

    node = processor()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


