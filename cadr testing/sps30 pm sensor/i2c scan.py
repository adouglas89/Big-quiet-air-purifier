from machine import Pin, I2C

# Create I2C object on I2C bus 0 (change to 1 if needed)
i2c = I2C(1, scl=Pin(15), sda=Pin(18), freq=10000)

print("Scanning I2C bus...")

devices = i2c.scan()

if devices:
    print("I2C devices found:", len(devices))
    for d in devices:
        print(" - Address: 0x{:02X}".format(d))
else:
    print("No I2C devices found")
