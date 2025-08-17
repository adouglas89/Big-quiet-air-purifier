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
filename_counter = 0
folder_name = "test folder3"
samples = []
time_start = ticks_ms()
def save_data(samples):
    global filename_counter
    try:
        os.mkdir("/sd/"+folder_name)
    except:
        print("folder apparently already created, could be bad, should be a new folder for each test")
    outfile = "/sd/"+folder_name+"/sps30samples"+str(filename_counter)+".json"  
    with open(outfile, "w") as f:
        json.dump(samples, f)
        print("samples saved, "+outfile)
    return
sda = machine.Pin(18)
scl = machine.Pin(15)
i2c = machine.I2C(1, sda=sda, scl=scl, freq=100000)
sps30 = SPS30(i2c, print_output=True)
time.sleep(5)
sps30.start_measurement()
errors = 0
if __name__ == "__main__":
    while True:
        sps30.read_data()
        data = sps30.last_measurement
        data.append(time.ticks_ms()-time_start)
        print(data)
        samples.append(data)
        save_data(samples)
        if gc.mem_free()<20_000:
            filename_counter += 1
            samples = []
            gc.collect()
        pm1count = sps30.last_measurement[5]
        pm4count = sps30.last_measurement[7]
        relevant_particle_count = pm4count-pm1count
        print(relevant_particle_count)
        print(time.ticks_ms())
        #print(sps30.last_measurement)
        time.sleep(5)
