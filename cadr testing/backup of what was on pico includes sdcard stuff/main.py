from machine import Pin, ADC, PWM
from math import log, e
import sys
from time import sleep, ticks_ms, ticks_us, ticks_diff
import machine
import sdcard
import os
import json
import gc
therm1_pin = ADC(Pin(27))
therm2_pin = ADC(Pin(26))


led=Pin('LED', Pin.OUT)
led.on()
therm_beta =3950
therm_R0 = 5000
T0 = 298.15
sense_r = 10_000 #resistance of sense resistor in ohms

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
def save_data_if_mem_full(samples):
    global filename_counter
    if gc.mem_free()<30_000:
        outfile = "/sd/thermistor_samples"+str(filename_counter)+".json"  #"/sd/fansamples.txt" to put on sd card if above stuff is done to mount it etc.
        filename_counter += 1
        with open(outfile, "w") as f:
            json.dump(samples, f)
            print("samples saved")
        samples = [] # is this going to erase the original object or a copy of it?? test it.
        gc.collect()
    return
def save_last_bit_data(samples):
    global filename_counter
    print("collection completed apparently,saving last bit")
    outfile = "/sd/thermistor_samples"+str(filename_counter)+".json"  #"/sd/fansamples.txt" to put on sd card if above stuff is done to mount it etc.
    filename_counter += 1
    with open(outfile, "w") as f:
        json.dump(samples, f)
    return

def read_therm(therm_pin):
    accum = 0
    tests = 20
    for i in range(tests):
        accum = accum + therm_pin.read_u16()
        sleep (0.001)
    average = accum/tests
    return average
def check_temperature(therm_pin):
    global therm_beta
    global therm_R0
    global T0
    global sense_r
    reading = read_therm(therm_pin)
    voltage = (reading/65356)*3.3
    current = voltage/(sense_r)

    total_resistance = 3.3/current

    therm_r = total_resistance-sense_r
    
    T = therm_beta/log(therm_r/(therm_R0*(e**(-1*therm_beta/T0)))) #remember this is in kelvin not celcius
    return T
timer = ticks_ms()
samples = []
while ticks_diff(ticks_ms(), timer)<360_000:
    t1 = check_temperature(therm1_pin)
    t2 = check_temperature(therm2_pin)
    time = ticks_ms()
    data_point = [t1,t2,time]
    samples.append(data_point)
    print("data point taken")
    save_data_if_mem_full(samples)
    sleep(0.2)
print("data collection complete")
save_last_bit_data(samples)
print("last bit of data saved")