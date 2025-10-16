import json
import serial
import time
import numpy as np
from scipy.interpolate import CubicSpline
def smooth_waveform(approx_waveform):
    # Number of input points (50) and desired output points (1000)
    num_approx_points = len(approx_waveform)
    num_output_points = 200
    
    # Generate an array of x-values for the original data (0 to 1 range)
    x_approx = np.linspace(0, 1, num_approx_points)
    
    # Generate an array of x-values for the interpolated data (also 0 to 1, but with 1000 points)
    x_output = np.linspace(0, 1, num_output_points)
    
    # Perform cubic spline interpolation
    spline = CubicSpline(x_approx, approx_waveform)
    smooth_waveform = spline(x_output)
    
    return smooth_waveform

def send_lut(lut):
    serial_port="COM62" # have to change this on different puters etc.
    timeout=0.5
    baudrate=1000000
    if not isinstance(lut, list):
        raise ValueError("LUT data must be a list of values.")
    lut = smooth_waveform(lut)
    ser = serial.Serial(serial_port, baudrate, timeout=timeout)
    ser.flushInput()
    print("Sending LUT to Arduino...")
    ser.write(b'L')  # Send command to start LUT transfer
    while ser.in_waiting == 0:
        time.sleep(0.1)
    if ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        if response:
            print("Arduino replied, proceeding:", response)  
    for value in lut:
        ser.write(f"{value:.4f}\n".encode())  # Send each value as a newline-separated string
        #print("sent:",f"{value:.4f}\n".encode())
        time.sleep(0.0001)  # Small delay to prevent buffer overflow
        
        # **Listen for Arduino responses during the transfer**
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response:
                print("Arduino:", response)  # Print any message received
    
    print("LUT transfer complete?")
    end_time = time.time() + 0.3
    while time.time() < end_time:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response:
                print("Arduino:", response)
    ser.close()
if __name__ == "__main__":    #this is so it doesn't run on import, but you can run this file to test it directly.
    with open("test_lut.json", "r") as file:
        lut = json.load(file)
    send_lut(lut)
