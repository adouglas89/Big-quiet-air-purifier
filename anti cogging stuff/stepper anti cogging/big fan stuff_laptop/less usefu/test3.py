import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

def generate_sine_wave(num_points=50, amplitude=1, frequency=1, phase_shift=0):
    # Create an array of x-values (from 0 to 2*pi for one full sine wave)
    x_values = np.linspace(0, 2 * np.pi, num_points)
    
    # Calculate the corresponding y-values for the sine wave
    y_values = amplitude * np.sin(frequency * x_values + phase_shift)
    
    return y_values

# Generate 50 points approximating a sine wave
y = generate_sine_wave()
# Function to generate a smooth fit from 50 data points to 1000 data points
def smooth_waveform(approx_waveform):
    # Number of input points (50) and desired output points (1000)
    num_approx_points = len(approx_waveform)
    num_output_points = 1000
    
    # Generate an array of x-values for the original data (0 to 1 range)
    x_approx = np.linspace(0, 1, num_approx_points)
    
    # Generate an array of x-values for the interpolated data (also 0 to 1, but with 1000 points)
    x_output = np.linspace(0, 1, num_output_points)
    
    # Perform cubic spline interpolation
    spline = CubicSpline(x_approx, approx_waveform)
    smooth_waveform = spline(x_output)
    
    return smooth_waveform

# Example: 50 floating-point numbers (approximated waveform)
approx_waveform = y  # Replace with your actual 50 values

# Get the smooth waveform (1000 points)
smooth_waveform = smooth_waveform(approx_waveform)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(np.linspace(0, 1, 50), approx_waveform, label="Approximate Waveform (50 points)", marker='o')
plt.plot(np.linspace(0, 1, 1000), smooth_waveform, label="Smooth Waveform (1000 points)", marker='o')
plt.legend()
plt.xlabel("Normalized x")
plt.ylabel("Voltage Value")
plt.title("Smooth Fit of Voltage Waveform")
plt.show()

# Output the smooth waveform (1000 points)
print(smooth_waveform)