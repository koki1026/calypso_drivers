import smbus2

def scan_i2c_devices(bus_num=1):
    bus = smbus2.SMBus(bus_num)
    print("Scanning I2C bus...")

    devices = []
    for addr in range(0x03, 0x78):
        try:
            bus.write_quick(addr)
            devices.append(addr)
        except:
            continue

    if devices:
        print(f"Found devices at: {[hex(d) for d in devices]}")
    else:
        print("No I2C devices found.")

    bus.close()

scan_i2c_devices()
