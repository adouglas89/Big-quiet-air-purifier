from math import log, e
from machine import Pin, ADC, PWM
from time import sleep
import sys
unpowered_therm_adc = ADC(Pin(27))
powered_therm_adc =ADC(Pin(26))
led=Pin(25, Pin.OUT)
led.on()
unpowered_pin1 = Pin(0,Pin.OUT)
unpowered_pin1.off()
unpowered_pin2 = Pin(1,Pin.OUT)
unpowered_pin2.off()
beta_unpowered = 2750
beta_powered = beta_unpowered
R0_unpowered = 68
R0_powered = 68
T0_unpowered = 298.15 #this has to be in kelvin not celcius
T0_powered = 298.15
unpowered_supply_v = 3.18
powered_supply_v = 3.3

sense_r_unpowered = 570  # had to add 520 more
sense_r_powered = 15 #had to add 10 extra.

resistance_current_limit_unpowered = sense_r_unpowered #(includes sense resistance)
resistance_current_limit_powered = sense_r_powered #includes sense resistance

def check_powered_temp(powered_therm_adc):
    global sense_r_powered
    global powered_supply_v
    global resistance_current_limit_powered
    global beta_powered
    global R0_powered 
    global T0_powered
    voltage_powered_sense = (powered_therm_adc.read_u16()/65356)*3.3
    current_powered = voltage_powered_sense/(sense_r_powered)
    total_resistance_powered = powered_supply_v/current_powered
    therm_r_powered = total_resistance_powered-resistance_current_limit_powered
    voltage_therm_powered = powered_supply_v*(therm_r_powered/total_resistance_powered)
    therm_powered_power = current_powered*voltage_therm_powered
    T_powered = beta_powered/log(therm_r_powered/(R0_powered*(e**(-1*beta_powered/T0_powered))))
    return T_powered, therm_powered_power
def check_unpowered_temp(unpowered_therm_adc):
    global sense_r_unpowered
    global unpowered_supply_v
    global resistance_current_limit_unpowered
    global beta_unpowered 
    global R0_unpowered 
    global T0_unpowered 
    global current_unpowered
    unpowered_pin1.on()
    unpowered_pin2.on()
    sleep(0.05)
    voltage_unpowered_sense = (unpowered_therm_adc.read_u16()/65356)*3.3
    unpowered_pin1.off()
    unpowered_pin2.off()
    current_unpowered = voltage_unpowered_sense/(sense_r_unpowered)
    total_resistance_unpowered = unpowered_supply_v/current_unpowered
    therm_r_unpowered = total_resistance_unpowered-resistance_current_limit_unpowered
    voltage_therm_unpowered = unpowered_supply_v*(therm_r_unpowered/total_resistance_unpowered)
    T_unpowered = beta_unpowered/log(therm_r_unpowered/(R0_unpowered*(e**(-1*beta_unpowered/T0_unpowered)))) #remember this is in kelvin not celcius
    return T_unpowered

last_t_above_ambient = 0
def mw_per_degree_diff(T_unpowered, T_powered, therm_powered_power):
    T_diff = T_powered-T_unpowered
    mw = therm_powered_power
    ratio = mw/T_diff
    return ratio
measures = 50
measures_unpowered = 50
while True:
    mwperdegsum=0
    t_accum = 0
    T_unpowered = check_unpowered_temp(unpowered_therm_adc)
    for i in range(0,measures_unpowered):
        sleep(0.005)
        T_unpowered = check_unpowered_temp(unpowered_therm_adc)
        #airspeed_proxy = check_airspeed_proxy(T_unpowered, T_powered)
        t_accum = t_accum + T_unpowered
    T_unpowered = t_accum/measures_unpowered
    for i in range(0,measures):
        sleep(0.01)
        T_powered, therm_powered_power = check_powered_temp(powered_therm_adc)
        mwperdeg = mw_per_degree_diff(T_unpowered, T_powered, therm_powered_power)
        mwperdegsum += mwperdeg
    
    #print(" t_u: ", T_unpowered-273.15, " t_p: ", T_powered-273.15)
    print(10000*mwperdegsum/measures)
    sleep(0.5)