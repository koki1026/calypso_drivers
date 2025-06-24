#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
詳細なI2C通信テスト
"""

import time
import sys

def detailed_i2c_test():
    """詳細なI2C通信テスト"""
    print("=== 詳細I2C通信テスト ===")
    
    try:
        import board
        import busio
        
        print("1. ライブラリインポート成功")
        
        # I2C初期化
        i2c = busio.I2C(board.SCL, board.SDA)
        print("2. I2Cバス初期化成功")
        
        # I2Cロック取得を試行
        print("3. I2Cロック取得を試行中...")
        timeout = 5  # 5秒でタイムアウト
        start_time = time.time()
        
        while not i2c.try_lock():
            if time.time() - start_time > timeout:
                print("✗ I2Cロック取得タイムアウト")
                return False
            time.sleep(0.1)
        
        print("4. I2Cロック取得成功")
        
        try:
            # デバイススキャン実行
            print("5. I2Cデバイススキャン開始...")
            devices = i2c.scan()
            
            print(f"6. スキャン完了")
            print(f"   検出されたデバイス数: {len(devices)}")
            
            if devices:
                print("   検出されたアドレス:")
                for device in devices:
                    print(f"     0x{device:02X} ({device})")
                
                # INA219の一般的なアドレスをチェック
                ina219_addresses = [0x40, 0x41, 0x44, 0x45]
                found_ina219 = [addr for addr in ina219_addresses if addr in devices]
                
                if found_ina219:
                    print(f"   ✓ INA219候補: {[hex(addr) for addr in found_ina219]}")
                    return found_ina219
                else:
                    print("   ⚠ INA219の一般的なアドレスが見つかりません")
                    print("   ⚠ 他のI2Cデバイスが検出されています")
                    return devices
            else:
                print("   ⚠ I2Cデバイスが検出されませんでした")
                return None
                
        except Exception as scan_error:
            print(f"✗ デバイススキャンエラー: {scan_error}")
            return None
            
        finally:
            # ロック解除
            i2c.unlock()
            print("7. I2Cロック解除完了")
            
    except ImportError as e:
        print(f"✗ ライブラリインポートエラー: {e}")
        print("   必要なライブラリ:")
        print("   sudo pip3 install adafruit-blinka")
        return None
        
    except Exception as e:
        print(f"✗ I2C通信エラー: {e}")
        return None

def test_ina219_if_found(address):
    """INA219が見つかった場合のテスト"""
    print(f"\n=== INA219テスト (0x{address:02X}) ===")
    
    try:
        import board
        import busio
        import adafruit_ina219
        
        # I2C初期化
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # INA219初期化
        ina219 = adafruit_ina219.INA219(i2c, addr=address)
        print("1. INA219初期化成功")
        
        # 測定テスト
        bus_voltage = ina219.bus_voltage
        print(f"2. バス電圧: {bus_voltage:.3f} V")
        
        shunt_voltage = ina219.shunt_voltage
        print(f"3. シャント電圧: {shunt_voltage:.3f} mV")
        
        current = ina219.current
        print(f"4. 電流: {current:.3f} mA")
        
        power = ina219.power
        print(f"5. 電力: {power:.3f} mW")
        
        print("✓ INA219動作確認成功！")
        return True
        
    except ImportError:
        print("✗ adafruit_ina219ライブラリが見つかりません")
        print("   インストール: sudo pip3 install adafruit-circuitpython-ina219")
        return False
        
    except Exception as e:
        print(f"✗ INA219テストエラー: {e}")
        return False

def main():
    print("詳細I2C通信テスト開始\n")
    
    # I2C通信テスト
    result = detailed_i2c_test()
    
    if result is None:
        print("\n結果: I2C通信に問題があります")
        print("確認事項:")
        print("- I2Cが有効になっているか")
        print("- 配線が正しいか")
        print("- デバイスに電源が供給されているか")
        
    elif isinstance(result, list) and len(result) > 0:
        print(f"\n結果: I2C通信は正常です")
        
        # INA219らしきデバイスが見つかった場合
        if any(addr in [0x40, 0x41, 0x44, 0x45] for addr in result):
            ina219_candidates = [addr for addr in result if addr in [0x40, 0x41, 0x44, 0x45]]
            print(f"INA219候補が見つかりました: {[hex(addr) for addr in ina219_candidates]}")
            while True:
                # 最初の候補でテスト
                test_ina219_if_found(ina219_candidates[0])
                time.sleep(0.5)
        else:
            print("INA219ではない他のI2Cデバイスが検出されました")
            print("INA219の配線とアドレス設定を確認してください")
    else:
        print("\n結果: I2C通信は正常ですが、デバイスが検出されませんでした")
        print("確認事項:")
        print("- INA219の配線（特にSDA、SCL）")
        print("- INA219への電源供給")
        print("- INA219のI2Cアドレス設定")

if __name__ == "__main__":
    try:
        main()
        print("\nテスト完了")
        
    except KeyboardInterrupt:
        print("\nテスト中断")
        
    except Exception as e:
        print(f"\n予期しないエラー: {e}")
        import traceback
        traceback.print_exc()
