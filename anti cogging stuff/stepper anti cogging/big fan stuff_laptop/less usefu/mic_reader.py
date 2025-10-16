import serial
from time import sleep
def read_mic(port="COM4", baudrate= 115200):
    loops = 100
    seconds_to_listen = 8
    for i in range(loops):    
        with serial.Serial(port, baudrate) as ser:
            if ser.in_waiting > 0:
                sleep(0.01)
                response = ser.readline().decode().strip()
                print(response)
                val = float(response)
                return val # Convert to float and return
        sleep(seconds_to_listen/loops)
        
print(read_mic())
