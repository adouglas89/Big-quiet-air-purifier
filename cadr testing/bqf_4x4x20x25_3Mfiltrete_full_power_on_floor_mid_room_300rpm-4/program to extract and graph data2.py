import json
import numpy as np
import matplotlib.pyplot as plt
import json
import math
import json
filename = "sps30samples"
file_ending = ".json"
name_counter = 0
samples = []
while True:
    try:
        file = open(filename+str(name_counter)+file_ending, 'r')
        samples_batch = json.load(file)
        name_counter += 1
        samples = samples + samples_batch
    except:
        print("done?")
        #print(samples)
        print("number of samples apparently:",len(samples))
        with open("samplescombined.json", "w") as outfile:
            json.dump(samples, outfile)
        break
filename = "samplescombined.json"
file = open(filename, 'r')
samples = json.load(file)
relevant_data = []
chamber_volume = 2544 #(in cubic feet)
for i in samples:
    pm1count = i[5]
    pm4count = i[7]
    diff = pm4count-pm1count
    #print(diff)
    time = i[-1]
    #print(time)
    relevant_data_point =[diff,time]
    relevant_data.append(relevant_data_point)
def calculate_cadr(time1, pc1, time2, pc2, volume_cuft):
    """
    Calculate Clean Air Delivery Rate (CADR) from two measurements.

    Parameters:
        time1 (float): Time of first measurement in seconds.
        pc1 (float): Particle concentration at time1 (particles/cm^3).
        time2 (float): Time of second measurement in seconds.
        pc2 (float): Particle concentration at time2 (particles/cm^3).
        volume_m3 (float): Volume of the sealed chamber in cubic meters.

    Returns:
        cadr_m3_per_hr (float): CADR in cubic meters per hour.
    """
    print(time1)
    print(time2)
    print(pc1)
    print(pc2)
    if pc1 <= 0 or pc2 <= 0:
        raise ValueError("Particle counts must be positive numbers.")

    delta_t = time2 - time1
    if delta_t <= 0:
        raise ValueError("time2 must be greater than time1.")

    # Decay constant (per second)
    k = math.log(pc1 / pc2) / delta_t

    # CADR in m³/s
    cadr_cuft_per_s = k * volume_cuft

    # Convert to m³/hour
    cadr_cuft_per_m = cadr_cuft_per_s * 60

    return cadr_cuft_per_m
def extract_elements(array_2d, element):
    # Ensure the input is a 2D NumPy array
    if array_2d.ndim != 2:
        raise ValueError("Input must be a 2D NumPy array")
    
    # Extract the first element from each row
    first_elements = array_2d[:, element]
    
    return first_elements

samples = relevant_data
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
ax.set_yscale("log")
# Add labels and title
ax.set_xlabel('ms_time')
ax.set_ylabel('1 to 4 micron particles per cc')
ax.set_title('particles per cc vs time')
first_grab = False
second_grab = False
control_point2 =[]
test_point1 = []
test_point2 = []
control_time1  = 1*60*1000
control_time2 = 4.5*60*1000
test_time1 = 7*60*1000
test_time2 = 9.5*60*1000
for i in samples:
    if i[1]>control_time1 and first_grab==False:
        control_point1 = i
        first_grab = True
    if i[1]>control_time2 and second_grab==False:
        control_point2 = i
        second_grab = True
first_grab = False
second_grab = False
for i in samples:
    if i[1]>test_time1 and first_grab==False:
        test_point1 = i
        first_grab = True
    if i[1]>test_time2 and second_grab==False:
        test_point2 = i
        second_grab = True

cadr_control = calculate_cadr(control_point1[1]/1000, control_point1[0], control_point2[1]/1000, control_point2[0], chamber_volume)
cadr_test_beforesubcontrl = calculate_cadr(test_point1[1]/1000, test_point1[0], test_point2[1]/1000, test_point2[0], chamber_volume)
print("control cadr: ", cadr_control)
print("test cadr before subtracting control: ",cadr_test_beforesubcontrl)
print("apparent machine cadr: ",cadr_test_beforesubcontrl-cadr_control)

# Draw vertical lines at specific x positions (same units as your x-axis)
ax.axvline(x=control_time1, color='red', linestyle='--', label='Control Point 1')
ax.axvline(x=control_time2, color='green', linestyle='--', label='Control Point 2')
ax.axvline(x=test_time1, color='red', linestyle='--', label='test Point 1')
ax.axvline(x=test_time2, color='green', linestyle='--', label='test Point 2')

plt.legend()
plt.show()

