import time
from mic_reader2 import listen_mic as read_mic
from lutsender2_done_stepper import send_lut
import matplotlib.pyplot as plt
import numpy as np
def generate_lut(freq=4, size=200, amplitude=1.35, phase_offset = 0):
    # Create a sine wave LUT
    lut = [amplitude * np.sin((2 * np.pi * i *freq/ size)+phase_offset) for i in range(size)]
    return lut

def plot_lut(lut):
    # Load LUT from JSON file
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
    return

tests = 20
max_amplitude = 2.5
# lut= generate_lut(phase_offset=np.pi*0,amplitude = 0)
# send_lut(lut)
# read_mic()
# send_lut(lut)
for i in range(tests):
    lut= generate_lut(amplitude = max_amplitude*(i/tests), phase_offset = 1)
    print("amplitude of above: ", max_amplitude*(i/tests))
    send_lut(lut)
    read_mic()
    read_mic()
    read_mic()

