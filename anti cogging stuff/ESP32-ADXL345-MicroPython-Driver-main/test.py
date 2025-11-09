from machine import Pin, SoftI2C
from adxl345 import ADXL345
from time import sleep
# Define I2C pins
i2c = SoftI2C(scl=Pin(1), sda=Pin(0))

# Initialize ADXL345
accel = ADXL345(i2c)

# Read accelerometer data
while True:
    sleep(0.1)
    x, y, z = accel.read()
    z=float(z)
    if z > 3200:
        new_z = (65536 - z)
    if z < 3200:
        new_z = -z
    print("z = {}".format(new_z))
