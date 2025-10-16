import json
import numpy as np

def generate_lut(filename="lut.json", size=1000, amplitude=8.0):
    # Create a sine wave LUT
    lut = [amplitude * np.sin(2 * np.pi * i / size) for i in range(size)]
    
    # Save to JSON file
    multiplier = 45.0
    lut.append(multiplier)
    with open(filename, "w") as file:
        json.dump(lut, file, indent=4)
    
    print(f"LUT saved to {filename} with {size} values. length: ", len(lut))

if __name__ == "__main__":
    generate_lut()
