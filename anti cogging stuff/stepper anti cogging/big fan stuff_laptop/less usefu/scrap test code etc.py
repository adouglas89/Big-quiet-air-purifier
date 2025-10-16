
# def send_lut(lut):
#     global mic_reading
#     x = np.linspace(0, 2 * np.pi, len(lut))  # Generate sine wave x values
#     sine_wave = 2.4 * np.sin(x)  # Compute sine wave
#     squared_diffs = np.square(np.array(lut) - sine_wave)  # Vectorized operation
#     mic_reading = np.mean(squared_diffs)
#     return mic_reading  # Faster averaging

# def read_mic():
#     global mic_reading
#     return mic_reading
# def read_rpm():
#     return 300