import serial
from time import sleep
def read_rpm(port="COM62", baudrate= 1_000_000):#it will sit there and do nothing forever I guess if it recieves nothing.
    with serial.Serial(port, baudrate) as ser:
        ser.write(b's')  # Send 's' command to request RPM
        sleep(0.1)
        line = ser.readline().decode('utf-8').strip()  # Read and decode response
        line = line[1:]     
        val = float(line)
        print(val)
        return val # Convert to float and return
if __name__ == "__main__":
    read_rpm()
    
