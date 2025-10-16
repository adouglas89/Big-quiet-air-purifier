from machine import Pin, ADC
from time import sleep, sleep_us

mic = ADC(Pin(27))

num_samples = 10000
sample_period = 2
sleep(10)
while True:
    accum = 0
    accum_above_average = 0
    samples =[]
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
    #print("average: ", average)
    #print("above average sample average: ", average_above_average-average)
    print(average_above_average-average)
