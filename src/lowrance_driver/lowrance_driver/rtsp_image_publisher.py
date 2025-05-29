# lowrance_driver/lowrance_driver/rtsp_image_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RTSPImagePublisher(Node):
    def __init__(self):
        super().__init__('rtsp_image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()

        rtsp_url = 'rtsp://192.168.76.1:554/screenmirror'
        self.cap = cv2.VideoCapture(rtsp_url)

        self.timer = self.create_timer(1.0 / 30.0, self.publish_image)

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("RTSPストリームから画像を取得できませんでした")

def main(args=None):
    rclpy.init(args=args)
    node = RTSPImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()