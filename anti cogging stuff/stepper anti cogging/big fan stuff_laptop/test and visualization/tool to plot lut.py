import json
import matplotlib.pyplot as plt

def plot_lut(filename="lut.json"):
    # Load LUT from JSON file
    with open(filename, "r") as file:
        lut = json.load(file)
    
    if not isinstance(lut, list):
        raise ValueError("LUT data must be a list of values.")
    
    # Plot the LUT
    plt.figure(figsize=(10, 4))
    plt.plot(lut, label="LUT Values", color="blue")
    plt.xlabel("Index")
    plt.ylabel("Amplitude")
    plt.title("Lookup Table (LUT) Graph")
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    plot_lut()
