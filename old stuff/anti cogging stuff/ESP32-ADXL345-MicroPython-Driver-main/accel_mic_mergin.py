from machine import Pin, I2C
from adxl345 import ADXL345
from time import sleep, sleep_us
import gc
# Define I2C pins
i2c = I2C(scl=Pin(1), sda=Pin(0))

# Initialize ADXL345
accel = ADXL345(i2c)

num_samples = 200
big_num_samples = 30
sleep(1)
# Read accelerometer data
def accel_read():
    x, y, z = accel.read()
    z=float(z)
    if z > 3200:
        new_z = (65536 - z)
    if z < 3200:
        new_z = -z
    return new_z
def measure_accel():
    global num_samples
    peak_measure = 0
    for i in range(num_samples):
        measure = accel_read()
        if measure > peak_measure:
            peak_measure = measure
    return peak_measure
while True:
    accum = 0
    for i in range(big_num_samples):
        measure = measure_accel()
        accum = accum + measure
    average = accum/big_num_samples
    print(average)
    gc.collect()
