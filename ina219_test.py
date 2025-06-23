#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import board
import busio
import adafruit_ina219

class VoltageMonitor:
    def __init__(self, i2c_address=0x40):
        try:
            # I2Cバスの初期化
            self.i2c = busio.I2C(board.SCL, board.SDA)
            # INA219センサーの初期化
            self.ina219 = adafruit_ina219.INA219(self.i2c, addr=i2c_address)
            print(f"INA219センサーが正常に初期化されました (アドレス: 0x{i2c_address:02x})")
        except Exception as e:
            print(f"センサーの初期化に失敗しました: {e}")
            raise                                                                        
        
    def read_voltage(self):
        try:
            bus_voltage = self.ina219.bus_voltagde
            return bus_voltage

        except Exception as e:
            print(f"電圧読み取りエラー: {e}")
            return None

    def read_shunt_voltage(self):
        try:
            shunt_voltage = self.ina219.shunt_voltage                  
            return shunt_voltage
        except Exception as e:
            print(f"シャント電圧読み取りエラー: {e}")                      
            return None

    def read_current(self):
        try:
            current = self.ina219.current
            return current
        except Exception as e:
            print(f"電流読み取りエラー: {e}")
            return None
            
    def read_power(self):
        try:
            power = self.ina219.power
            return power
        except Exception as e:
            print(f"電力読み取りエラー: {e}")
            return None
            
    def read_all_values(self):
        return {
            'bus_voltage': self.read_voltage(),
            'shunt_voltage': self.read_shunt_voltage(),
            'current': self.read_current(),
            'power': self.read_power()
            }

    def main():
        try:
            # 電圧モニターを初期化
            monitor = VoltageMonitor()
                    
            print("電圧測定を開始します...")
            print("Ctrl+Cで終了\n")
        
            while True:
                # すべての値を読み取り
                values = monitor.read_all_values()
                
                if all(v is not None for v in values.values()):
                    print(f"バス電圧: {values['bus_voltage']:.3f} V")
                    print(f"シャント電圧: {values['shunt_voltage']:.3f} mV")
                    print(f"電流: {values['current']:.3f} mA")
                    print(f"電力: {values['power']:.3f} mW")
                    print("-" * 40)
                else:
                    print("測定エラーが発生しました")
                
                time.sleep(1)  # 1秒間隔で測定
        except KeyboardInterrupt:
            print("\n測定を終了します")                     
        except Exception as e:
            print(f"エラーが発生しました: {e}")
