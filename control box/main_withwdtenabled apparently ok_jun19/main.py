from time import ticks_ms, ticks_diff, ticks_us, sleep, sleep_us
from machine import PWM, Pin, ADC, reset, UART, WDT
import json
import os
import sys
import select
import network
from umqtt.robust import MQTTClient
import uping
import anemometer
calibration_file_filename = "anem_calib.json"
uart1=UART(1, 115200)
uart1.init(115200, tx=Pin(4),rx=Pin(5),timeout_char=1)# the timeout should be reduced after things appear to be working, to increase responsiveness of the e read
perc_filter_health = 100
potpinadc = ADC(Pin(28))
oldpot_val =0
mqtt_perc = 3
main_perc = 50
pwm_out_pin = PWM(Pin(15))
pwm_out_pin.freq(20_000) #different fans expect different frequencies but 20khz is typical for pc fans,  If driving a mosfet directly, this should be reduced probably
persistent_vars_filename = "persistent_vars.json"
persistent_vars_dict = {"ssid_main_wifi": "enterssidhere",
                        "password_main_wifi": "none",
                        "ADAFRUIT_IO_URL" : "io.adafruit.com",
                        "ADAFRUIT_USERNAME" : b'',
                        "ADAFRUIT_IO_KEY": b'',
                        "ADAFRUIT_IO_FEEDNAME" : b'bqapX_control',
                        "ADAFRUIT_IO_FEEDNAME_publish" : b'bqapX_status',
                        }
def set_v(v):
    num_string = f'{v:.3f}'
    padded_num_s = f'{num_string:0<7}'
    command = "V"+padded_num_s+"\n"
    print("sending to driver:", command)
    uart1.write(command)
def perc_to_voltage(perc):
    max_voltage = 9
    min_voltage = 3
    voltage_out = perc*max_voltage/100
    if voltage_out > max_voltage:
        voltage_out = max_voltage
    if voltage_out < min_voltage:
        voltage_out = 0
    return voltage_out   
def read_rps():
    start_time = ticks_us()
    stuff_ready = 0
    message=uart1.read() #flush out data siting in the uart buffer
    uart1.write("s")
    sleep_us(60_000)
    message=uart1.read()
    string = str(message)
    #print(string)
    string = string[3:-5] # there is extra stuff that gets added during transmission, plus the e, chop it off. 
    print("apparent rps:", string)
    val=None
    try:
        val = float(string)
    except BaseException as error:
        print(error,"maybe the driver is not connected")
    return val # this should be a floating point number with 3 digits after the decimal point, varying from zero to 2pi
def set_pwm_perc(perc):
    duty = 655.53*perc
    if duty > 65535:
        duty = 65535
    if duty < 0:
        duty = 0
    pwm_out_pin.duty_u16(int(duty))
    return
potpinadc = ADC(Pin(28))
oldpot_val=0
def check_pot():
    global oldpot_val
    read = potpinadc.read_u16()
    print("raw pot val:", read)
    if abs(read-oldpot_val)>1600:#some hysteresis
        oldpot_val = read
        return read/655.53
    return oldpot_val/655.53
def connect_wifi(ssid, password):
    wdt.feed()
    wlan = network.WLAN(network.STA_IF)
    wdt.feed()
    if wlan.active()==True:
        wlan.active(False)
    wlan.active(True)
    wdt.feed()
    wlan.connect(ssid, password)
    wdt.feed()
    x=0
    while wlan.isconnected()==False:
        print("Waiting for connection to network with ssid ",ssid," and password:", password, "...",x)
        sleep(1)
        x=x+1
        led.toggle()
        wdt.feed()
        if x>20:
            print("couldn't connect in ", x , " seconds")
            return None, None 
    ip=wlan.ifconfig()
    print("connected to wifi, my address is:",ip[0])
    led.off()
    return ip[0], wlan
def try_connect_forever():
    global ip
    ip = None
    while ip == None:#keep trying forever
        wdt.feed()
        ip, wlan = connect_wifi(ssid_main_wifi,password_main_wifi)
        wdt.feed()
    return ip, wlan
def mqtt_publish(message):
    global mqtt_feedname_publish
    wdt.feed()
    client.publish(mqtt_feedname_publish,    
                   bytes(str(message), 'utf-8'), 
                   qos=0)
    wdt.feed()
    return
def cb(topic, msg):# this only gets executed if there is an mqtt message recieved, it's the callback
    global mqtt_perc
    global main_power_perc
    try:
        perc = int(str(msg)[2:-1]) #should be a bytes object, so if we convert it to a string chop off extras then int that should work.
        if perc >100:
            perc = 100
        if perc <0:
            perc = 0
    except BaseException as error:
        print("there was no message or it was not a valid integer, message was:",str(msg)[2:-1],"error was: ",)
    else:
        mqtt_perc = perc
        main_power_perc = mqtt_perc
        print("mqtt perc command recieved mqtt_perc variable updated:", mqtt_perc)
mqtt_feedname_publish = 0
def connect_mqtt():
    global ADAFRUIT_USERNAME
    global ADAFRUIT_IO_FEEDNAME
    global ADAFRUIT_IO_FEEDNAME_publish
    global ADAFRUIT_IO_URL
    global ADAFRUIT_IO_KEY
    global mqtt_feedname_publish
    mqtt_feedname = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, ADAFRUIT_IO_FEEDNAME), 'utf-8')
    mqtt_feedname_publish = bytes('{:s}/feeds/{:s}'.format(ADAFRUIT_USERNAME, ADAFRUIT_IO_FEEDNAME_publish), 'utf-8')
    random_num = int.from_bytes(os.urandom(3), 'little')
    mqtt_client_id = bytes('client_'+str(random_num), 'utf-8')
    client = MQTTClient(client_id=mqtt_client_id, 
                        server=ADAFRUIT_IO_URL, 
                        user=ADAFRUIT_USERNAME, 
                        password=ADAFRUIT_IO_KEY,
                        ssl=False)
    client.set_callback(cb)
    try:
        print("about to start mqtt connection")
        wdt.feed()
        client.connect()
        wdt.feed()
        mqtt_connected = 1
        print("mqtt connected ok! Recieve feed name:", ADAFRUIT_IO_FEEDNAME, "Status updates feedname:",ADAFRUIT_IO_FEEDNAME_publish)
    except Exception as e:
        mqtt_connected = 0
        print('could not connect to MQTT server, oserror -1 means the adafruit server is not working right or key or username not right -2 usually means no internet connection: {}{}'.format(type(e).__name__, e))
        print("username: ",ADAFRUIT_USERNAME, " key: ",ADAFRUIT_IO_KEY)
    if mqtt_connected == 1:
        wdt.feed()
        client.subscribe(mqtt_feedname)
        wdt.feed()
    return mqtt_connected, client
def comm_mqtt(mqtt_connected):
    global perc_filter_health
    if mqtt_connected == 1:
        try:
            sleep(0.1)
            wdt.feed()
            mqtt_publish("clock time: "+str(ticks_ms()) + " perc filter health: "+str(perc_filter_health))
            wdt.feed()
            print("stuff published")
            sleep(0.1)
            wdt.feed()
            client.check_msg()
            wdt.feed()
            print("mqtt messages checked ok")
        except BaseException as error:
            print("apparently connected to mqtt but got an error during effort to communicate over mqtt", error)
    return
def check_pot():
    global oldpot_val
    read = potpinadc.read_u16()
    print("raw pot val:", read)
    if abs(read-oldpot_val)>1600:
        oldpot_val = read
        return read/655.53
    return oldpot_val/655.53
def perc_to_voltage(perc):
    max_voltage = 9
    min_voltage = 3
    voltage_out = perc*max_voltage/100
    if voltage_out > max_voltage:
        voltage_out = max_voltage
    if voltage_out < min_voltage:
        voltage_out = 0
    return voltage_out   
def check_wifi_reconnect(ip, wlan):
    try:    
        if wlan.isconnected() == False:
            ip, wlan = try_connect_forever() 
            return ip, wlan
    except BaseException as error:
            print("error trying to reconnect wifi, probbly it means we are in ap mode if wlan doesn't exist error is:", error)
            return None, None
    return ip, wlan
def big_connect_mqtt():
    global ADAFRUIT_IO_URL
    global mqtt_connected
    try:
        wdt.feed()
        uping.ping(ADAFRUIT_IO_URL)
        wdt.feed()
        mqtt_connected, client = connect_mqtt()
        wdt.feed()
    except BaseException as error:
        print("error trying to ping stuff to test wifi and then connect mqtt etc.  -2 seems to be no internet connection: ",error)
    return client
def calibrate_anemometer():
    global calibration_file_filename
    data_points = 20
    data = []
    for i in range(data_points):
        perc = 100*i/data_points
        v = perc_to_voltage(perc)
        print("calibrating anemometer for new filters, this takes about 20 minutes, voltage to fan:",v,"percent power (lower powers are off): ", perc)
        set_v(v)
        for i in range(60):#have to do things like this so can not trip wdt
            wdt.feed()
            sleep(1)
        wdt.feed()
        rps = read_rps()
        wdt.feed()
        mwpd = anemometer.read_anemometer(wdt)
        wdt.feed()
        data.append([rps, mwpd])
    with open(calibration_file_filename, "w") as outfile:
        json.dump(data, outfile)
    return
def restore_vars():
    with open(persistent_vars_filename,"r") as openfile:
        vars_dict = json.load(openfile)
    globals().update(vars_dict)
    return
def mwpd_calib_to_perc_flow(mwpd,calib):
        if rps == None:
            return None
        if rps >-5:
            return None #can't get  a valid reading at very low rps levels.
        should_be_mwpd = interpolate_airflow(rps, calib)
        pre_perc_filter_health = 100*mwpd/should_be_mwpd
        perc_filter_health = 100-5*(100-pre_perc_filter_health)
        return perc_filter_health
def interpolate_airflow(rps, points): #uses calibration data to return the expected mw per degree the anemometer would read if filters are new, at a given rps
    # Sort by RPS in case list is unordered
    points = sorted(points, key=lambda x: x[0])

    # Edge case: below lowest data point
    if rps <= points[0][0]:
        x0, y0 = points[0]
        x1, y1 = points[1]
    # Edge case: above highest data point
    elif rps >= points[-1][0]:
        x0, y0 = points[-2]
        x1, y1 = points[-1]
    else:
        # Find two data points to interpolate between
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            if x0 <= rps <= x1:
                break

    # Linear interpolation
    mwpd = y0 + (rps - x0) * (y1 - y0) / (x1 - x0)
    return mwpd
last_b = 3
last_a = 3
main_power_perc = 3
def combine_percs(a, b): #when either a or be changes, it goes in to the main power percentage variable.
    global last_b
    global last_a
    global main_power_perc
    if abs(last_b - b)>2:
        main_power_perc = b
        last_b = b
    if abs(last_a - a)>2:
        main_power_perc = a
        last_a = a
    print("mqtt_percent:", b)
    print("main_power_perc:", main_power_perc)
    return main_power_perc
#main program

print("you have 5 seconds to start the repl before I enable the watchdog")
sleep(5)
wdt = WDT(timeout = 8000)
wdt.feed()
root_files = os.listdir('/')
if calibration_file_filename not in root_files: # if the file doesn't exist then do calibration and save file
    wdt.feed()
    calibrate_anemometer()
    wdt.feed()
led=machine.Pin('LED', machine.Pin.OUT)
if 'persistent_vars.json' not in root_files: # if the file doesn't exist then create it with the defaults.
    save_vars()
else:
    restore_vars()

ip = None
ip, wlan = try_connect_forever()
mqtt_connected = 0
client = big_connect_mqtt()
with open(calibration_file_filename,"r") as openfile:
    calib = json.load(openfile)
timer1 = 0
timer2 = 0
timer3 = 3

while True:
    if ticks_diff(ticks_ms(),timer3)>1000: 
        pot_val = check_pot()
        print("pot_val, percent: ",pot_val)
        combine_percs(pot_val, mqtt_perc) #puts the result in a global variable main_power_perc
        wdt.feed()
        v = perc_to_voltage(main_power_perc)
        wdt.feed()
        set_v(v)
        wdt.feed()
        sleep(0.05)
        rps = read_rps()
        wdt.feed()
        print("rps: ",rps)
        timer3 = ticks_ms()
        #relocate the below later
        mwpd = anemometer.read_anemometer(wdt) #this takes a long time
        perc_filter_health = mwpd_calib_to_perc_flow(mwpd,calib)
        set_pwm_perc(main_power_perc)#this is an a
        print("apparent percent filter health, remember it is not valid after power changes for a bit:", perc_filter_health)
    if ticks_diff(ticks_ms(),timer1)>30_000:
        if wlan.isconnected()==False:
            wdt.feed()
            ip, wlan = check_wifi_reconnect(ip, wlan)#need to reconnect mqtt if wifi disconnects, probably
            wdt.feed()
            big_connect_mqtt()
            wdt.feed()
        if mqtt_connected == 1:
            wdt.feed()
            comm_mqtt(mqtt_connected)
            wdt.feed()
        timer1 = ticks_ms()
        

    if ticks_ms()>86_400_000: #just reset every day in case something stops working.
        sleep(10)#this should cause the watchdog to reset the system 
