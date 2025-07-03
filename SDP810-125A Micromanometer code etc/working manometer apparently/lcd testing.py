from machine import Pin, PWM, I2C
from time import ticks_ms, ticks_diff, sleep, sleep_ms
from PWMCounter import PWMCounter
from PID import PID
from machine_i2c_lcd import I2cLcd
# The PCF8574 has a jumper selectable address: 0x20 - 0x27
DEFAULT_I2C_ADDR = 0x27
i2c = I2C(1,  sda=machine.Pin(2), scl=machine.Pin(3), freq=100000)
lcd = I2cLcd(i2c, DEFAULT_I2C_ADDR, 4, 20)
lcd.backlight_on()

lcd.putstr("Testing LCD\n\n, if you see this it seems to work, what happens\n when  ")
sleep_ms(3000)
lcd.clear()
