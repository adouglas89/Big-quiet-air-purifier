import numpy as np
from scipy.interpolate import CubicSpline

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

