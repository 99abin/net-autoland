import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from autoland_interfaces.msg import NetDetection
from .detector import DetectionConfig, detect_rectangle


class NetDetector(Node):
    def __init__(self):
        super().__init__("net_det")
        defaults = dict(
            image_topic="/camera/image_raw",
            hsv_lower=[100, 90, 45],
            hsv_upper=[160, 255, 255],
            min_area_px=600.0,
            min_fill=0.70,
            min_aspect=0.8,
            max_aspect=6.0,
            max_image_fraction=0.90,
        )
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        p = {k: self.get_parameter(k).value for k in defaults}
        self.c = DetectionConfig(
            **{
                k: tuple(v) if k.startswith("hsv_") else v
                for k, v in p.items()
                if k != "image_topic"
            }
        )
        self.bridge = CvBridge()
        self.pub = self.create_publisher(NetDetection, "/autoland/net_detection", 10)
        self.mask_pub = self.create_publisher(Image, "~/mask", qos_profile_sensor_data)
        self.create_subscription(Image, p["image_topic"], self.cb, qos_profile_sensor_data)

    def cb(self, image):
        msg = NetDetection()
        msg.header = image.header
        try:
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            detection, mask = detect_rectangle(bgr, self.c)
            if detection is not None:
                corners, msg.score = detection
                msg.corners = corners.reshape(8).astype(float).tolist()
                msg.valid = True
                msg.reason = "Color quadrilateral detected"
            else:
                msg.reason = "No matching blue/purple rectangle"
            if self.mask_pub.get_subscription_count():
                output = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
                output.header = image.header
                self.mask_pub.publish(output)
        except Exception as exc:
            msg.reason = str(exc)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
