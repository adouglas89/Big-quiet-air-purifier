import json
import serial
import time

def send_lut(serial_port, baudrate=1000000, timeout=2):
    # Load LUT from JSON file
    with open("lut.json", "r") as file:
        lut = json.load(file)
    
    if not isinstance(lut, list):
        raise ValueError("LUT data must be a list of values.")
    
    # Open serial connection
    ser = serial.Serial(serial_port, baudrate, timeout=timeout)
    time.sleep(2)  # Wait for Arduino to reset after opening serial
    
    print("Sending LUT to Arduino...")
    ser.write(b'L')  # Send command to start LUT transfer
    
    for value in lut:
        ser.write(f"{value}\n".encode())  # Send each value as a newline-separated string
        time.sleep(0.01)  # Small delay to prevent buffer overflow
    
    print("LUT transfer complete.")
    ser.close()

if __name__ == "__main__":
    serial_port = "COM62"  # Change to the correct port (e.g., "/dev/ttyUSB0" on Linux/Mac)
    send_lut(serial_port)
