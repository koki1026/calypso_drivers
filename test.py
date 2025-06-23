#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys

def test_i2c_connection():
    print("===I2C接続テスト===")
    try:
        import board
        import busio

        #I2C初期化
        i2c = busio.I2C(board.SCL, board.SDA)
        print ("✓ I2Cバスの初期化成功")

    except Exception as e:
        print(f"✗ I2C接続エラー: {e}")
        return None

def main():
    """メインテスト関数"""
    print("INA219 トラブルシューティング開始\n")
    detected_address = test_i2c_connection()

if __name__ == "__main__":
    try:
        main()
        
    except KeyboardInterrupt:
       print("\nテストを中断しました")
    
    except Exception as e:
        print(f"\n予期しないエラー: {e}")
        import traceback
        traceback.print_exc()
