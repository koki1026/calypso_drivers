#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import psutil
import time

# 標準メッセージの型をインポート
from std_msgs.msg import Float32, Float64

class SystemMonitorPublisher(Node):
    def __init__(self):
        super().__init__('system_monitor_publisher')

        # --- 5つのパブリッシャーを作成 ---
        self.cpu_pub = self.create_publisher(Float32, 'system/cpu_percent', 10)
        self.mem_pub = self.create_publisher(Float32, 'system/memory_percent', 10)
        self.disk_pub = self.create_publisher(Float32, 'system/disk_percent', 10)
        self.net_sent_pub = self.create_publisher(Float64, 'system/net_sent_rate', 10)
        self.net_recv_pub = self.create_publisher(Float64, 'system/net_recv_rate', 10)

        timer_period = 2.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.last_net_io = psutil.net_io_counters()
        self.last_time = time.time()
        self.get_logger().info('System Monitor Publisher Node has been started.')

    def timer_callback(self):
        current_time = time.time()
        delta_time = current_time - self.last_time

        # --- 各メトリクスを計算して変数に格納 ---
        cpu_val = psutil.cpu_percent(interval=None)
        mem_val = psutil.virtual_memory().percent
        disk_val = psutil.disk_usage('/').percent
        
        # --- 変数を使ってメッセージを配信 ---
        self.cpu_pub.publish(Float32(data=cpu_val))
        self.mem_pub.publish(Float32(data=mem_val))
        self.disk_pub.publish(Float32(data=disk_val))

        # --- ネットワーク通信量を計算 & 配信 ---
        current_net_io = psutil.net_io_counters()
        sent_rate = 0.0
        recv_rate = 0.0
        if delta_time > 0:
            sent_rate = (current_net_io.bytes_sent - self.last_net_io.bytes_sent) / delta_time
            recv_rate = (current_net_io.bytes_recv - self.last_net_io.bytes_recv) / delta_time
            self.net_sent_pub.publish(Float64(data=sent_rate))
            self.net_recv_pub.publish(Float64(data=recv_rate))

        # --- 変数を使ってログ出力 ---
        self.get_logger().info(
            f'CPU: {cpu_val:.1f}% | '
            f'Mem: {mem_val:.1f}% | '
            f'Disk: {disk_val:.1f}% | '
            f'Net Sent: {sent_rate/1024:.2f} KB/s | '
            f'Net Recv: {recv_rate/1024:.2f} KB/s'
        )

        # 次回計算のために現在の値を保存
        self.last_net_io = current_net_io
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    monitor_publisher = SystemMonitorPublisher()
    rclpy.spin(monitor_publisher)
    monitor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
