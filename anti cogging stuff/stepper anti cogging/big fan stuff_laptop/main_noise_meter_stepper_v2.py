from machine import Pin, ADC
from time import sleep, sleep_us
import gc
mic = ADC(Pin(27))


num_samples = 2000
big_num_samples = 20
sample_period = 0.4
sleep(1)
def measure_mic():
    global num_samples
    peak_measure = 0
    for i in range(num_samples):
        sleep_us(int((sample_period*1_000_000/num_samples)))
        measure = mic.read_u16()
        if measure > peak_measure:
            peak_measure = measure
    return peak_measure
while True:
    accum = 0
    for i in range(big_num_samples):
        measure = measure_mic()
        accum = accum + measure
    average = accum/big_num_samples
    print(average)
    gc.collect()
