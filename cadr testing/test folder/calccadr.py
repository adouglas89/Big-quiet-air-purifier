import math
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

test = calculate_cadr(15*60, 4*1_000_000, 60*60, 9.9*10_000, 1200)
print(test)