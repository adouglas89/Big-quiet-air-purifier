import json
import serial
import time

def send_lut(serial_port, baudrate=1000000, timeout=30):
    # Load LUT from JSON file
    with open("lut.json", "r") as file:
        lut = json.load(file)
    
    if not isinstance(lut, list):
        raise ValueError("LUT data must be a list of values.")
    
    # Open serial connection
    ser = serial.Serial(serial_port, baudrate, timeout=timeout)
    time.sleep(2)  # Wait for Arduino to reset after opening serial
    
    # Flush any old data from the input buffer
    ser.flushInput()
    
    print("Sending LUT to Arduino...")
    ser.write(b'L')  # Send command to start LUT transfer
    while ser.in_waiting == 0:
        time.sleep(0.1)
    if ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        if response:
            print("Arduino replied, proceeding:", response)  # Print any message received
    for value in lut:
        ser.write(f"{value:.4f}\n".encode())  # Send each value as a newline-separated string
        print("sent:",f"{value:.4f}\n".encode())
        time.sleep(0.001)  # Small delay to prevent buffer overflow
        
        # **Listen for Arduino responses during the transfer**
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response:
                print("Arduino:", response)  # Print any message received
    
    print("LUT transfer complete?")

    # **Continue listening for Arduino responses after transfer**
    end_time = time.time() + 10  # Listen for 5 more seconds
    while time.time() < end_time:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            if response:
                print("Arduino:", response)

    ser.close()

if __name__ == "__main__":
    serial_port = "COM62"  # Change to the correct port (e.g., "/dev/ttyUSB0" on Linux/Mac)
    send_lut(serial_port)
