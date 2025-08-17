import numpy as np
import matplotlib.pyplot as plt
import json

def load_coordinates(filename):
    """Load coordinates from a JSON file."""
    with open(filename, 'r') as json_file:
        coordinates = json.load(json_file)
    return coordinates

def extract_elements(array_2d, element):
    """Extract a specific element (column) from a 2D NumPy array."""
    # Ensure the input is a 2D NumPy array
    if array_2d.ndim != 2:
        raise ValueError("Input must be a 2D NumPy array")
    return array_2d[:, element]

# Load the data from the JSON file
samples = load_coordinates('samplescombined.json')

# Prepare two datasets, extracting relevant columns
samplest1 = [[i[0], i[2]] for i in samples]
samplest2 = [[i[1], i[2]] for i in samples]

# Convert lists to NumPy arrays
array1 = np.array(samplest1)
array2 = np.array(samplest2)

# Extract x and z components for each dataset
x_flat1 = extract_elements(array1, 0)
z_flat1 = extract_elements(array1, 1)

x_flat2 = extract_elements(array2, 0)
z_flat2 = extract_elements(array2, 1)

# Clean up invalid data (remove NaN or None)
mask1 = ~np.isnan(x_flat1) & ~np.isnan(z_flat1)
x_flat1, z_flat1 = x_flat1[mask1], z_flat1[mask1]

mask2 = ~np.isnan(x_flat2) & ~np.isnan(z_flat2)
x_flat2, z_flat2 = x_flat2[mask2], z_flat2[mask2]

# Create a figure and axes
fig, ax = plt.subplots()

# Plot both datasets as scatter plots
ax.scatter(z_flat1, x_flat1, label='Data from samplest1', color='b', s= 2)  # Points for samplest1
ax.scatter(z_flat2, x_flat2, label='Data from samplest2', color='r', s=2)  # Points for samplest2

# Add labels and title
ax.set_xlabel('particles 1-4 micron per cc')
ax.set_ylabel('time_ms')
ax.set_title('Temperature vs ms timestamp')

# Show the legend
plt.legend()

# Display the plot
plt.show()
