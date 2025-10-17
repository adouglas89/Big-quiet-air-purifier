import serial
import struct
import time
import json
from time import sleep
import numpy as np
from scipy.interpolate import CubicSpline
from mic_reader2 import listen_mic

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
def calculate_checksum(float_bytes):
    checksum = 0
    for b in float_bytes:
        checksum ^= b
    return checksum

def send_float(ser, value):
    # Pack float into bytes
    float_bytes = struct.pack('<f', value)  # Little-endian float
    checksum = calculate_checksum(float_bytes)
    #print(checksum)#to check the object type etc.
    packet = float_bytes + bytes([checksum])
    #print(packet)# again to check it.
    ser.write(packet)

def wait_for_ack(ser):
    response = ser.readline().strip()
    #print("arduino says:", response)
    return response.decode('utf-8')

def send_lut(lut):
    lut = smooth_waveform(lut)
    lut = lut.tolist()
    num_values = len(lut)
    with serial.Serial('COM12', 1000000, timeout=2) as ser:
        print("Starting transfer...")
        ser.write(b'L')
        sleep(0.01)
        response = ser.readline().decode().strip()
        print("Arduino says:",response)
        for index, value in enumerate(lut):
            while True:
                send_float(ser, value)
                ack = wait_for_ack(ser)
                if ack == "A":
                    #print(f"Sent {index + 1}/{num_values}: {value}")
                    break
                if ack == "N":
                    print(f"Resending {index + 1}/{num_values}: {value}")
                else:
                    print(f"unknown reply, Resending {index + 1}/{num_values}: {value}")
        print("Transfer complete.")
    with open("smoothedsentlut.json", "w") as file:
         json.dump(lut, file)
    return lut
if __name__ == "__main__":    #not clear this works this is so it doesn't run on import, but you can run this file to test it directly.
    with open("best_lut.json", "r") as file:
        lut = json.load(file)
    send_lut(lut)
    while True:
        listen_mic()