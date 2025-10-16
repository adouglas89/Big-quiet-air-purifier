from machine import ADC, Pin
from time import sleep

micadc = ADC(Pin(27))
oldval = 0
cycle_time = 0.01
while True:
    for i in range(int(0.2/cycle_time)):
        val=micadc.read_u16()
        if val > oldval:
            oldval = val
        if oldval > 32500:
            oldval = (oldval - cycle_time*(1/10)*32500) #skootch it down slowly would be x seconds to go from full to zero if the term is 1/x
        sleep(cycle_time)
    print(oldval)