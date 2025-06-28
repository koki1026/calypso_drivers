import time
import sys
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import board
import busio
import adafruit_ina219


class INA219PublisherNode(Node):
    def __init__(self, address):
        super().__init__('ina219_publisher_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ina219_data', 10)

        # I2C通信とINA219初期化
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ina219 = adafruit_ina219.INA219(i2c, addr=address)
        self.get_logger().info(f'INA219 initialized at address 0x{address:02X}')

        # 500msごとにコールバック実行
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            msg = Float32MultiArray()
            bus_voltage = self.ina219.bus_voltage
            shunt_voltage = self.ina219.shunt_voltage
            current = self.ina219.current
            power = self.ina219.power

            msg.data = [bus_voltage, shunt_voltage, current, power]
            self.publisher_.publish(msg)

            self.get_logger().info(f'Published: V={bus_voltage:.2f}V, I={current:.2f}mA, P={power:.2f}mW')

        except Exception as e:
            self.get_logger().error(f"INA219 read error: {e}")
            traceback.print_exc()


def scan_i2c_for_ina219():
    """INA219のI2Cアドレスを自動検出"""
    i2c = busio.I2C(board.SCL, board.SDA)

    timeout = 5
    start_time = time.time()
    while not i2c.try_lock():
        if time.time() - start_time > timeout:
            print("I2C lock timeout")
            return None
        time.sleep(0.1)

    try:
        devices = i2c.scan()
        ina219_addresses = [0x40, 0x41, 0x44, 0x45]
        found = [addr for addr in ina219_addresses if addr in devices]
        return found[0] if found else None
    finally:
        i2c.unlock()


def main():
    rclpy.init()

    address = scan_i2c_for_ina219()
    if address is None:
        print("INA219 not found on I2C bus")
        return

    node = INA219PublisherNode(address)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

