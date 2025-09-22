import json
import numpy as np
import matplotlib.pyplot as plt
import json
filename = "samplescombined.json"
file = open(filename, 'r')
samples = json.load(file)
relevant_data = []
for i in samples:
    for j in i:
        pm1count = j[5]
        pm4count = j[7]
        diff = pm4count-pm1count
        #print(diff)
        time = j[-1]
        #print(time)
        relevant_data_point =[diff,time]
        relevant_data.append(relevant_data_point)

with open("relevant_data.json", "w") as outfile:
    json.dump(relevant_data, outfile)
def load_coordinates(filename):
    with open(filename, 'r') as json_file:
        coordinates = json.load(json_file)
    return coordinates

def extract_elements(array_2d, element):
    # Ensure the input is a 2D NumPy array
    if array_2d.ndim != 2:
        raise ValueError("Input must be a 2D NumPy array")
    
    # Extract the first element from each row
    first_elements = array_2d[:, element]
    
    return first_elements

samples = load_coordinates('relevant_data.json')
samplest1 = []
for i in samples:
    samplest1.append([i[0], i[1]])
array1 = np.array(samplest1)  # Convert list to array

# Flatten the arrays for plotting
x_flat = extract_elements(array1, 0)
z_flat = extract_elements(array1, 1)

# Create a figure
fig, ax = plt.subplots()

# Plot a 2D line plot
ax.plot(z_flat, x_flat, label='Data', color='b')  # Add line plot

# Add labels and title
ax.set_xlabel('throttle')
ax.set_ylabel('rpm')
ax.set_title('Throttle vs RPM')

# Show the plot
plt.legend()
plt.show()

