import machine
import time

from sps30 import SPS30

if __name__ == "__main__":
    sda = machine.Pin(18)
    scl = machine.Pin(15)
    i2c = machine.I2C(1, sda=sda, scl=scl, freq=100000)
    sps30 = SPS30(i2c, print_output=True)
    time.sleep(5)
    sps30.start_measurement()
    run = True
    errors = 0
    while run:
        try:
            sps30.read_data()
            pm1count = sps30.last_measurement[5]
            pm4count = sps30.last_measurement[7]
            relevant_particle_count = pm4count-pm1count
            print(relevant_particle_count)
            print(time.ticks_ms())
            print(sps30.last_measurement)
            time.sleep(5)
            errors = 0
        except OSError as err:
            errors += 1
            print(err, errors)
        except KeyboardInterrupt:
            run = False
