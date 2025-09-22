# main.py
from machine import I2C, Pin
import time
from sps30_mp import SPS30

# I2C0 on Pico: SDA=GP14, SCL=GP15 (adjust as needed)
i2c = I2C(1, scl=Pin(15), sda=Pin(18), freq=10000)
sensor = SPS30(i2c)

# Optional: scan to confirm address 0x69 is present
print("I2C scan:", [hex(x) for x in i2c.scan()])
time.sleep(1)
sensor.start_measurement()

# Wait until data are ready (first measurement can take a moment)
for _ in range(50):
    if sensor.data_ready():
        break
    time.sleep_ms(200)

if sensor.data_ready():
    data = sensor.read_measurement()
    print(data)
else:
    print("Sensor data not ready yet.")

# When done
# sensor.stop_measurement()
