#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LowrancePublisher(Node):
    def __init__(self):
        super().__init__('lowrance_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

        # device_idの取得
        self.declare_parameter('device_id', 0)
        device_id = self.get_parameter('device_id').get_parameter_value().integer_value

        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'カメラデバイス {device_id} を開けませんでした。')
            return
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            height, width, _ = frame.shape
            if width >= 480:
                # 横方向中央をトリミング（1280x480 → 480x480）
                center_x = width // 2
                half_crop = 240
                cropped = frame[:, center_x - half_crop:center_x + half_crop]
            else:
                self.get_logger().warn(f'フレーム幅が480未満のためトリミングできません（現在: {width}px）')
                cropped = frame
            
            msg = self.bridge.cv2_to_imgmsg(cropped, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('カメラフレームの取得に失敗しました')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LowrancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()