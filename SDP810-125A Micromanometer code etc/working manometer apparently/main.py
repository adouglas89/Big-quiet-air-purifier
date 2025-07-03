import time
from time import ticks_ms, sleep, sleep_ms
from machine import I2C, Pin
from machine_i2c_lcd import I2cLcd
import sdp810_125

# Initialize I2C bus
i2c2 = machine.I2C(1, scl=machine.Pin(27), sda=machine.Pin(26), freq = 100_000)
#for lcd
DEFAULT_I2C_ADDR = 0x27
#i2c2 = I2C(1,  sda=Pin(2), scl=Pin(3), freq=100000)
lcd = I2cLcd(i2c2, DEFAULT_I2C_ADDR, 4, 20)
lcd.backlight_on()
lcd.putstr("Testing LCD\n, if you see this it seems to work.  ")
sleep_ms(1000)
lcd.clear()
# Create library object on our I2C port
devices = i2c2.scan()
if devices:
    for d in devices:
        print(hex(d))
print(sdp810_125.get_reading())
def sample():
    accum = 0
    samples = 30
    for i in range(samples):
        print(i)
        accum = accum + sdp810_125.get_reading()
        print(sdp810_125.get_reading())
        sleep(0.25)
    average = accum/samples
    print("average pressure:",average)
    return average
    
while True:
    average = sample()
    lcd.clear()
    lcd.putstr(str(average))
    





