# check_adxl345_i2c.py — Raspberry Pi Pico (MicroPython)
from machine import Pin, SoftI2C
import time

# Create I2C instance (your pins)
i2c = SoftI2C(scl=Pin(1), sda=Pin(0))

# ADXL345 I2C possible addresses (depends on ALT ADDRESS pin)
ADXL345_ADDRS = [0x53, 0x1D]
REG_DEVID = 0x00  # Device ID register, should return 0xE5

def check_adxl345_i2c():
    print("🔍 Scanning I2C bus...")
    devices = i2c.scan()
    if not devices:
        print("⚠️  No I2C devices found!")
        return

    print("Found I2C device addresses:", [hex(d) for d in devices])

    for addr in ADXL345_ADDRS:
        if addr in devices:
            try:
                devid = i2c.readfrom_mem(addr, REG_DEVID, 1)[0]
                print("Device at", hex(addr), "returned DEVID =", hex(devid))
                if devid == 0xE5:
                    print("✅ ADXL345 detected at I2C address", hex(addr))
                    return True
                else:
                    print("⚠️  Unexpected DEVID at", hex(addr))
            except OSError:
                print("❌  Error communicating with device at", hex(addr))
    print("❌ No ADXL345 detected.")
    return False

check_adxl345_i2c()
