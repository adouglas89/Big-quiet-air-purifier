import machine
import time
from sps30 import SPS30
from machine import Pin
import sys
from time import sleep, ticks_ms, ticks_us, ticks_diff
import sdcard
import os
import json
import gc
led=Pin('LED', Pin.OUT)
led.on()
spi = machine.SPI(1,
                  sck=machine.Pin(10),
                  mosi=machine.Pin(11),
                  miso=machine.Pin(12))
# Initialize CS pin
cs = machine.Pin(13, machine.Pin.OUT)

# Create SD card object
sd = sdcard.SDCard(spi, cs)
# Mount the filesystem
vfs = os.VfsFat(sd)
os.mount(vfs, "/sd")
print("sd card ok apparently")