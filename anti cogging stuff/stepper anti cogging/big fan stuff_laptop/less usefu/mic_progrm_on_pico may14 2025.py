from machine import Pin, ADC
from time import sleep, sleep_us
import gc
mic = ADC(Pin(27))

num_samples = 10000
sample_period = 1

def read():
    accum = 0
    accum_above_average = 0
    samples =[]
    gc.collect()
    for i in range(num_samples):
        measure = mic.read_u16()
        accum = accum + measure
        samples.append(measure)
        sleep_us(int((sample_period*1_000_000/num_samples)))
    average = accum/num_samples
    counter = 0
    for i in samples:
        if i > average:
            accum_above_average += i
            counter += 1
    average_above_average = accum_above_average/counter
    rise = average_above_average-average
    return rise
    #print("average: ", average)
    #print("above average sample average: ", average_above_average-average)
rounds = 3
def big_read():
    big_samples = []
    big_accum = 0
    for i in range(rounds):
        reading = read()
        big_samples.append(reading)
    for i in big_samples:
        big_accum = big_accum + i
    big_average = big_accum/len(big_samples)
    gc.collect()
    return big_average
while True:
    print(big_read())