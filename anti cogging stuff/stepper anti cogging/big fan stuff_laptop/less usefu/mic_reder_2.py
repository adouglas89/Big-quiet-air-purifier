import serial
def listen_mic(port="COM4", baudrate=115200, timeout=1.0):
    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            print(f"Listening for mic reading on {port}...")
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print("mic reading: ", line)
                    return(float(line))
    except serial.SerialException as e:
        print(f"Serial error: {e}")

if __name__ == "__main__":
    listen_mic()
