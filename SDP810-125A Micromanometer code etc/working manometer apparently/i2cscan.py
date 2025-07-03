import machine

# Create I2C object
i2c = machine.I2C(1, scl=machine.Pin(27), sda=machine.Pin(26))
#i2c = machine.SoftI2C(scl=machine.Pin(2), sda = machine.Pin(3))
# Print out any addresses found
devices = i2c.scan()

if devices:
    for d in devices:
        print(hex(d))