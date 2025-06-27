import serial

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

print("受信中（1バイトずつ表示）:")

try:
    while True:
        byte = ser.read()
        if byte:
            print(byte.hex(), end=' ')
except KeyboardInterrupt:
    print("\n終了")
finally:
    ser.close()
