#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import psutil

# 作成したカスタムメッセージをインポート
# <your_package_name> は実際のパッケージ名に置き換えてください
from cpu_monitor.msg import Memory
from std_msgs.msg import Float32

class CpuMemoryPublisher(Node):

    def __init__(self):
        super().__init__('cpu_memory_publisher')

        self.cpu_publisher_ = self.create_publisher(Float32, 'cpu_usage', 10)
        self.memory_publisher_ = self.create_publisher(Float32, 'memory_usage', 10)

        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('CPU and Memory Usage Publisher Node has been started.')
        self.get_logger().info('Publishing to /cpu_memory_usage topic every 1 second.')

        psutil.cpu_percent(interval=None)

    def timer_callback(self):

        cpu_percent = psutil.cpu_percent(interval=None)
        memory_percent = psutil.virtual_memory().percent

        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_publisher_.publish(cpu_msg)

        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_publisher_.publish(memory_msg)

        self.get_logger().info(f'Publishing: [CPU: {cpu_msg.data:.2f}%] [Memory: {memory_msg.data:.2f}%]')

def main(args=None):
    rclpy.init(args=args)
    try:
        cpu_memory_publisher = CpuMemoryPublisher()
        rclpy.spin(cpu_memory_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'cpu_memory_publisher' in locals() and rclpy.ok():
            cpu_memory_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
