from machine import PWM, Pin
from time import sleep, ticks_ms, ticks_us, ticks_diff
import sys
ingress_fan = PWM(Pin(4))
ingress_fan.freq(20_000)
def I_fan_spd(spd_prcnt):
        if spd_prcnt > 100:
            spd_prcnt = 100
        if spd_prcnt < 0:
            spd_prcnt = 0
        duty = spd_prcnt*65530/100
        duty = int(duty)-1
        if duty <0:
            duty = 0
        ingress_fan.duty_u16(duty)
        return spd_prcnt
egress_fan = PWM(Pin(5))
egress_fan.freq(20_000)
def E_fan_spd(spd_prcnt):
        if spd_prcnt > 100:
            spd_prcnt = 100
        if spd_prcnt < 0:
            spd_prcnt = 0
        duty = spd_prcnt*65530/100
        duty = int(duty)-1
        if duty <0:
            duty = 0
        egress_fan.duty_u16(duty)
        return spd_prcnt
tach_ingress = Pin(2, Pin.IN, Pin.PULL_UP) 
tach_egress = Pin(3, Pin.IN, Pin.PULL_UP)
def det_rpm_interval(pin, interval):
    blip_count = 0
    meas_spacing_timer = ticks_us()
    was_1 = 0
    first_edge = 1
    between_edges_time_us = 0
    while ticks_diff(ticks_us(), meas_spacing_timer)/1_000_000 <= interval:
        if pin.value() == 1:
            was_1 = 1
        if pin.value() == 0 and was_1 == 1 and first_edge == 1:
            between_edges_timer = ticks_us()
            was_1 = 0
            first_edge = 0
        if pin.value() == 0 and was_1 == 1 and first_edge == 0:
            between_edges_time_us += ticks_diff(ticks_us(),between_edges_timer)
            between_edges_timer = ticks_us()
            blip_count += 1
            was_1 = 0
    if blip_count != 0:
        between_edges_average = between_edges_time_us/blip_count
        rps = 1000000/(between_edges_average*2)
        rpm = rps*60
    if blip_count == 0:
        #print("no blips")
        rpm = 0
    if pin == tach_egress:
        rpm = -rpm
    return rpm
def calc_total_boost(boost_adjust,rel_boost, spd):
        total_boost = (boost_adjust * rel_boost) # give it a hundred percent power for a while as a boost
        if  abs(spd) > 52 and abs(spd) < 88:
            total_boost = ((boost_adjust * 1.4 * rel_boost)) # just boost the shit out of it to collect data, run again with more practical run to see what stall looks like just extra boost for that region to get it started in high winds
        if  abs(spd) >= 12 and abs(spd) < 30:
            total_boost = ((boost_adjust * 2 * rel_boost))
        if  abs(spd) >= 7.5 and abs(spd) < 12:
            total_boost = ((boost_adjust * 3 * rel_boost))
        return total_boost
was_off_I = 1
was_off_E = 1
combi_ingress_spd = 1
combi_egress_spd = 1
last_dir_throttle = 0
last_spd = 0
spd_internal = 0
def combi_fan_spd(spd):
    global combi_ingress_spd
    global combi_egress_spd
    global was_off_I
    global was_off_E
    global last_dir_throttle 
    global last_spd
    global spd_internal
    spd_internal = spd
    accel_mea_points = 6
    boost_adjust = 0.02 #seconds so for 100 percent diff 100x this many seconds of boost
    accel_samples = []
    if -7.5 <= spd <= 7.5:
        combi_ingress_spd = 0 
        combi_egress_spd = 0
        E_fan_spd(combi_egress_spd)
        I_fan_spd(combi_ingress_spd)
    # if ingress
    if  spd >= 7.5 and (spd - combi_ingress_spd) >= 0: # if it's not as fast as the latest command
        combi_egress_spd = 0
        E_fan_spd(combi_egress_spd)
        rel_boost = spd - combi_ingress_spd
        total_boost = calc_total_boost(boost_adjust,rel_boost, spd)
        time_between_meas_points = total_boost/accel_mea_points
        I_fan_spd(100)
        if combi_ingress_spd == 0: # if it was off, measure and save rpm and timestamp on the way up
            last_dir_throttle = last_spd
            collection_timer = ticks_us()
            while ticks_diff(ticks_us(),collection_timer)/1_000_000 <= total_boost:
                rpm = det_rpm_interval(tach_ingress, time_between_meas_points)
                accel_samples.append(rpm)
        if combi_ingress_spd != 0:
            sleep(total_boost)
        sleep(total_boost)
        combi_ingress_spd = spd
        I_fan_spd(combi_ingress_spd)
        sleep(total_boost/3)
    if  spd > 7.5 and (spd - combi_ingress_spd) < 0: # if speed is being reduced just set reduced speed immediately
        combi_ingress_spd = spd
        I_fan_spd(combi_ingress_spd)
# if it's egress that's commanded       
    if  spd < -7.5 and (abs(spd) - combi_egress_spd) >= 0: 
        combi_ingress_spd = 0
        I_fan_spd(combi_ingress_spd)
        rel_boost = abs(spd) - combi_egress_spd
        total_boost = calc_total_boost(boost_adjust,rel_boost, spd)
        time_between_meas_points = total_boost/accel_mea_points
        E_fan_spd(100)
        if combi_egress_spd == 0: # if it was off, measure and save rpm and timestamp on the way up
            last_dir_throttle = last_spd
            collection_timer = ticks_us()
            while ticks_diff(ticks_us(),collection_timer)/1_000_000 <= total_boost:
                rpm = det_rpm_interval(tach_egress, time_between_meas_points)
                accel_samples.append(rpm)
        if combi_egress_spd != 0:
            sleep(total_boost)
        sleep(total_boost)
        combi_egress_spd = abs(spd)
        E_fan_spd(combi_egress_spd)
        sleep(total_boost/3)
    if  spd < -7.5 and (abs(spd) - combi_egress_spd) < 0: # if speed is being reduced just set reduced speed immediately
        combi_egress_spd = abs(spd)
        E_fan_spd(combi_egress_spd)
    last_spd = spd
     # just to let the fan stabilize to avoid kicking the pid every time it changes 
    return accel_samples # the last three we use for wind pressure regressor, probably chop the timestamp out yeah but not yet, want it for visualizing etc.
def rpm(interval):
    global spd_internal
    if spd_internal == 0:
        rpm = 0
    if spd_internal > 0:
        rpm = det_rpm_interval(tach_ingress, interval)
    if spd_internal < 0:
        rpm = det_rpm_interval(tach_egress, interval)
    return rpm
combi_fan_spd(0)
#sys.exit()
# test program

