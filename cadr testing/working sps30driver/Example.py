'''
Test and example of the SPS30 micropython driver using the I2C protocol.
'''

from machine import Pin, I2C
import time
from u_sps30 import SPS30_I2C

# Iniaitialize the I2C port
i2c = I2C(1, scl=Pin(15), sda=Pin(18), freq=10000)

# Create an instance of the SDS driver.
time.sleep(1)
test = SPS30_I2C(i2c, datatype="FP32") # Standard datatype is UInt
test.wakeup()

# Give information of the SPS30 device
print("*** DEVICE INFORMATION ***")
print(test.read_status_register())
print("SPS30 firmware verison:", test.read_firmware_version())
print("Intervall self cleaning:", test.read_auto_cleaning_interval())
print("Product type:", test.read_product_type())
print("Serial Number: ", test.read_serial_number())

# Doing a measurement
print("*** MEASURMENT ***")
test.start_measurement()
time.sleep(0.3)
print(f"Data ready flag:{test.read_data_ready_flag()}")
data = test.read_data()
print("Recorded Data:", data)
test.stop_measurement()
time.sleep(0.5)
print(f"Data ready flag:{test.read_data_ready_flag()}")

# Testing  Sleep mode (reducing energy consumption)
print("*** TESTING SLEEP MODE ***")
test.sleep()
time.sleep(5)
test.wakeup()
time.sleep(2)
test.start_measurement()
print(test.read_data())

# Testing fan vleaning and put device to sleep to conserve energy
print("*** TESTING FAN/FINISHING UP (SLEEP) ***")
test.start_fan_cleaning() # Runs for 10 s
time.sleep(10)
# # Put the SPS30 into sleep to save energy
test.sleep()