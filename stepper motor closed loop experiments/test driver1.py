from machine import Pin, PWM, ADC
from time import sleep
dir_pin_green = Pin(27)
pul_purple_pwm = PWM(Pin(4))
enable_blue = Pin(5)
potpinadc = ADC(Pin(28))
oldpot_val =0
dir_pin_green.on()
enable_blue.off()
duty_cycle = 0.5
microsteps = 51200 #per revolution, not per step. 200 full steps per rev so this is 256 microsteps by usual logic
max_rps = 7.5
def check_pot_perc():
    global oldpot_val
    read = potpinadc.read_u16()
    #print("raw pot val:", read)
    if abs(read-oldpot_val)>1600:#some hysteresis
        oldpot_val = read
        return read/655.53
    return oldpot_val/655.53
pul_purple_pwm.duty_u16(32_000)


while True:
    pot_val = check_pot_perc()
    frequency_needed = int(microsteps*max_rps*pot_val/100)+40
    #print(frequency_needed)
    pul_purple_pwm.freq(frequency_needed)
    print("rps: ",max_rps*pot_val/100)
    sleep(1)
