import numpy as np
import matplotlib.pyplot as plt

# Function to generate 50 points approximating a sine wave
def generate_sine_wave(num_points=50, amplitude=1, frequency=1, phase_shift=0):
    # Create an array of x-values (from 0 to 2*pi for one full sine wave)
    x_values = np.linspace(0, 2 * np.pi, num_points)
    
    # Calculate the corresponding y-values for the sine wave
    y_values = amplitude * np.sin(frequency * x_values + phase_shift)
    
    return y_values

# Generate 50 points approximating a sine wave
y = generate_sine_wave()

# Plot the sine wave with the 50 points
plt.figure(figsize=(10, 6))
plt.plot(np.linspace(0, 2 * np.pi, 50), y, label="Sine Wave (50 points)", marker='o')
plt.xlabel("X values")
plt.ylabel("Sine of X")
plt.title("50 Points Approximating a Sine Wave")
plt.legend()
plt.grid(True)
plt.show()

# Output the 50 points as a single list
print("Sine Wave Values (50 points):")
print(y)
