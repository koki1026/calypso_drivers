#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import Jetson.GPIO as GPIO
import time
import threading

class FlowSensorNode(Node):
    def __init__(self):
        super().__init__('flow_sensor_node')

        # GPIO設定
        self.flow_pin = 31  # BOARDモードでのピン番号
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.flow_pin, GPIO.IN)

        # パルスカウンタ
        self.pulse_count = 0
        GPIO.add_event_detect(self.flow_pin, GPIO.FALLING, callback=self.pulse_callback)

        # ROSパブリッシャ
        self.pub = self.create_publisher(Float32, 'flow_rate_lpm', 10)

        # 1秒ごとにパルスを処理するタイマー
        self.timer = self.create_timer(1.0, self.publish_flow_rate)

        self.get_logger().info("FlowSensorNode started")

    def pulse_callback(self, channel):
        self.pulse_count += 1

    def publish_flow_rate(self):
        # 計算式: Q = pulse_count / 7.5 [L/min]
        flow_rate = self.pulse_count / 7.5
        msg = Float32()
        msg.data = flow_rate
        self.pub.publish(msg)
        self.get_logger().info(f"Flow rate: {flow_rate:.2f} L/min")

        # カウントをリセット
        self.pulse_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = FlowSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
