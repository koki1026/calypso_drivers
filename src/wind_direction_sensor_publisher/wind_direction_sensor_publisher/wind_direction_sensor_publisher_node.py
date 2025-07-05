#!/usr/bin/env python3

from pymodbus.client import ModbusSerialClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import time

class WindDirectionPublisherNode(Node):
    def __init__(self):
        super().__init__('wind_direction_sensor_publisher_node')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'wind_direction_data', 10)

        # Modbus client設定
        self.client = ModbusSerialClient(
            port="/dev/ttyUSB0",
            baudrate=9600,
            parity="N",
            stopbits=1,
            bytesize=8
        )

        if self.client.connect():
            self.get_logger().info("Modbus 接続成功")
        else:
            self.get_logger().error("Modbus 接続失敗")
            return

        # 0.5秒ごとにタイマー実行
        self.timer = self.create_timer(0.5, self.read_and_publish)

    def read_and_publish(self):
        result = self.client.read_holding_registers(address=0, count=12, slave=1)
        if result.isError():
            self.get_logger().error(f"読み取りエラー: {result}")
        else:
            msg = Int32MultiArray()
            msg.data = result.registers
            self.publisher_.publish(msg)
            self.get_logger().info(f"パブリッシュ: {msg.data}")

    def destroy_node(self):
        self.client.close()
        self.get_logger().info("Modbus 切断完了")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WindDirectionPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()