import serial
from pymodbus.client import ModbusSerialClient as ModbusClient
import time

# シリアルポートとModbusクライアントの設定
# ポート名、ボーレートは環境と風速計の仕様に合わせてください
# parity, stopbits, bytesize も風速計のModbus設定に合わせる
client = ModbusClient(
    port='/dev/ttyUSB0',  # Jetsonで認識されたシリアルポート
    baudrate=9600,        # 風速計のModbusボーレート
    parity='N',           # パリティ (N:None, E:Even, O:Odd)
    stopbits=1,           # ストップビット (1 or 2)
    bytesize=8,           # データビット (通常8)
    timeout=1             # 通信タイムアウト
)

# 風速計のModbusアドレス (通常は1)
# 風速値が格納されているModbusレジスタアドレスとデータ型を確認
# 例: ホールディングレジスタ 0x0001 (10進数で1) に風速がFloatで入っている場合
SLAVE_ID = 1
REGISTER_ADDRESS = 1 # 風速が格納されているModbusレジスタのアドレス (10進数)
REGISTER_COUNT = 2 # Float値の場合、2ワード (4バイト) を読み取る必要がある

print(f"Modbusクライアントを {client.port} で開きます...")

if not client.connect():
    print("シリアルポートに接続できませんでした。")
    exit()

print("Modbus接続成功。風速データを読み取ります...")

try:
    while True:
        # ホールディングレジスタを読み取る
        # response = client.read_holding_registers(REGISTER_ADDRESS, REGISTER_COUNT, unit=SLAVE_ID)
        # 例えば、Input Registersから読む場合:
        response = client.read_input_registers(REGISTER_ADDRESS, REGISTER_COUNT, unit=SLAVE_ID)

        if response.isError():
            print(f"Modbusエラー: {response}")
        else:
            # Modbusから読み取った生のレジスタ値
            # print(f"Raw registers: {response.registers}")

            # レジスタ値を風速に変換（風速計の仕様による）
            # 例: 整数値の場合
            # wind_speed_raw = response.registers[0]
            # wind_speed = wind_speed_raw / 100.0 # 100倍されて送られてくる場合など

            # 例: Modbus Float (32-bit float) の場合
            from pymodbus.payload import BinaryPayloadDecoder
            from pymodbus.constants import Endian

            # レジスタからデコーダーを作成
            decoder = BinaryPayloadDecoder.fromRegisters(
                response.registers,
                byteorder=Endian.Big,    # バイトオーダーを確認 (Big or Little)
                wordorder=Endian.Big     # ワードオーダーを確認 (Big or Little)
            )
            wind_speed = decoder.decode_32bit_float()
            print(f"風速: {wind_speed:.2f} m/s")

        time.sleep(1) # 1秒ごとに読み取り
except KeyboardInterrupt:
    print("\nプログラムを終了します。")
finally:
    if client.is_connected:
        client.close()
        print("Modbus接続を閉じました。")
