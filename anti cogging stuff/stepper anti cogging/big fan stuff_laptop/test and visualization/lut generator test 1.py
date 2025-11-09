import json
import numpy as np

def generate_lut(filename="test_lut.json", size=200, amplitude=0.25):
    # Create a sine wave LUT
    lut = [amplitude * np.sin(2 * np.pi * i / size) for i in range(size)]
    with open(filename, "w") as file:
        json.dump(lut, file, indent=4)
    
    print(f"LUT saved to {filename} with {size} values. length: ", len(lut))

if __name__ == "__main__":
    generate_lut()
