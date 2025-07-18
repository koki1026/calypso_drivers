import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class ImageRepubNode(Node):
    def __init__(self):
        super().__init__('image_repub_node')
        self.bridge = CvBridge()

        self.topics = {
            '/flir_camera/image_color': '/image_color/compressed',
            '/image_raw': '/image_raw/compressed',
            '/thermal_image': '/thermal_image/compressed'
        }

        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscribers_dict = {}
        self.publishers_dict = {}
        self.last_pub_times = {}

        for in_topic, out_topic in self.topics.items():
            self.subscribers_dict[in_topic] = self.create_subscription(
                Image, in_topic, self.make_callback(in_topic), 10)
            self.publishers_dict[in_topic] = self.create_publisher(
                CompressedImage, out_topic, 10)
            self.last_pub_times[in_topic] = 0.0

    def make_callback(self, topic_name):
        def callback(msg):
            now = time.time()
            if now - self.last_pub_times[topic_name] < 1.0:
                return
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                resized = cv2.resize(cv_image, (320, 240))
                ret, buffer = cv2.imencode('.jpg', resized)
                if not ret:
                    self.get_logger().warn(f"JPEG encoding failed for {topic_name}")
                    return
                comp_msg = CompressedImage()
                comp_msg.header = msg.header
                comp_msg.format = "jpeg"
                comp_msg.data = buffer.tobytes()
                self.publishers_dict[topic_name].publish(comp_msg)
                self.last_pub_times[topic_name] = now
            except Exception as e:
                self.get_logger().error(f"Error in {topic_name}: {e}")
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = ImageRepubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

