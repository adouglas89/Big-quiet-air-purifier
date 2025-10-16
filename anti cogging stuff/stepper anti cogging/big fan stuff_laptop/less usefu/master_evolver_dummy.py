import json
import time
import random
import numpy as np
from deap import base, creator, tools, algorithms
from datetime import datetime
import os
import numpy as np

#from lutsender_module import send_lut

# def send_lut(lut):
#     """Compares the LUT to a sine wave (one oscillation, amplitude 2.4),
#        calculates the squared difference at each point, and returns the average."""
#     global mic_reading
#     # Generate a sine wave with one full oscillation over the LUT length
#     x = np.linspace(0, 2 * np.pi, len(lut))  # Creates 1001 points from 0 to 2π
#     sine_wave = 2.4 * np.sin(x)  # Sine wave with amplitude 2.4
#     
#     # Compute squared differences
#     squared_diffs = [(lut[i] - sine_wave[i]) ** 2 for i in range(len(lut))]
#     
#     # Return the average squared difference
#     mic_reading = sum(squared_diffs) / len(lut)
#     return mic_reading
def send_lut(lut):
    global mic_reading
    x = np.linspace(0, 2 * np.pi, len(lut))  # Generate sine wave x values
    sine_wave = 2.4 * np.sin(x)  # Compute sine wave
    squared_diffs = np.square(np.array(lut) - sine_wave)  # Vectorized operation
    mic_reading = np.mean(squared_diffs)
    return mic_reading  # Faster averaging

def read_mic():
    global mic_reading
    return mic_reading
def read_rpm():
    return 300
def obj_func(mic, rpm):
    value = mic*100+abs(rpm-300)*100
    print("obj func evaluates to:",value)
    return value
tests = 0
# Constants
LUT_SIZE = 101
POP_SIZE = 20
CXPB, MUTPB = 0.5, 0.2  # Crossover & mutation probabilities
GENS = 1000
STATE_FILE = "optimizer_state.json"
BEST_FILE = "best_lut.json"
Best_actual_lut = "best_actual_lut.json"
# DEAP setup
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, -1, 1)  # Adjust range if needed
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, LUT_SIZE)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

def evaluate(individual):
    global tests
    send_lut(list(individual))
    #time.sleep(10)
    mic = read_mic()
    rpm = read_rpm()
    tests = tests+1
    print("tests: ",tests)
    return obj_func(mic, rpm),

toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.1, indpb=0.1)
toolbox.register("select", tools.selTournament, tournsize=3)
toolbox.register("evaluate", evaluate)

def save_state(population, gen, best):
    state = {
        "generation": gen,
        "population": [list(ind) for ind in population],
        "best": list(best)
    }
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)

def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r") as f:
            state = json.load(f)
        return state["generation"], [creator.Individual(ind) for ind in state["population"]], state["best"]
    return 0, None, None

def save_best(best):
    global BEST_FILE
    global Best_actual_lut
    with open(BEST_FILE, "w") as f:
        json.dump(best, f)
    with open(Best_actual_lut, "w") as f:
        json.dump(list(best), f)

def main():
    gen, population, best_lut = load_state()
    if population is None:
        population = toolbox.population(n=POP_SIZE)

    best_ind = creator.Individual(best_lut) if best_lut else None

    hall_of_fame = tools.HallOfFame(1)
    if best_ind:
        hall_of_fame.insert(best_ind)

    start_time = time.time()
    try:
        while True:
            offspring = algorithms.varAnd(population, toolbox, CXPB, MUTPB)
            fits = list(map(toolbox.evaluate, offspring))
            for ind, fit in zip(offspring, fits):
                ind.fitness.values = fit

            population = toolbox.select(offspring, k=POP_SIZE)
            hall_of_fame.update(population)

            if best_ind is None or hall_of_fame[0].fitness.values < best_ind.fitness.values:
                best_ind = creator.Individual(hall_of_fame[0])
                save_best(best_ind)

            if time.time() - start_time > 300:  # Save every 5 minutes
                save_state(population, gen, best_ind)
                start_time = time.time()

            gen += 1  # Increment generation counter
            print("gen: ",gen)
    except KeyboardInterrupt:
        print("\nOptimization interrupted. Saving state...")
        save_state(population, gen, best_ind)
        print("State saved. Exiting.")
if __name__ == "__main__":
    main()

