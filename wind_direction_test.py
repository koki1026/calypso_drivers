from pymodbus.client import ModbusSerialClient
import time


client = ModbusSerialClient(
    port="/dev/ttyUSB1",
    baudrate=9600,
    parity="N",
    stopbits=1,
    bytesize=8
)

if client.connect():
    print("接続成功")

    while True:
        # アドレス0番から2レジスタ読み込み、スレーブアドレスは1
        result = client.read_holding_registers(address=0, count=12, slave=1)
        if result.isError():
            print("エラー:", result)
            break
        else:
            print("読み取り成功:", result.registers)

        # 1秒待機して再試行
        time.sleep(0.5)

    client.close()
else:
    print("接続失敗")
