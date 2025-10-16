import json
import time
import random
import numpy as np
import multiprocessing
from deap import base, creator, tools, algorithms
import os

# Function to evaluate LUT similarity to sine wave
def send_lut(lut):
    x = np.linspace(0, 2 * np.pi, len(lut))
    sine_wave = 2.4 * np.sin(x)
    squared_diffs = np.square(np.array(lut) - sine_wave)
    return np.mean(squared_diffs)

def read_mic():
    return send_lut.last_result

def read_rpm():
    return 300

def obj_func(mic, rpm):
    return mic * 100 + abs(rpm - 300) * 100  # Objective function

# Constants
LUT_SIZE = 1001
POP_SIZE = 50
CXPB, MUTPB = 0.5, 0.2
GENS = 1000
STATE_FILE = "optimizer_state.json"
BEST_FILE = "best_lut.json"
Best_actual_lut = "best_actual_lut.json"

# DEAP setup
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, -1, 1)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, LUT_SIZE)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Ensure pool is only created inside the main guard
def create_pool():
    return multiprocessing.Pool(processes=multiprocessing.cpu_count())  # Use all available CPU cores

def evaluate(individual):
    """Evaluate function compatible with multiprocessing"""
    lut = list(individual)  # Convert individual to list
    mic = send_lut(lut)  # Send LUT and get noise level
    #time.sleep(10)  # Simulate processing delay
    rpm = read_rpm()  # Get RPM
    #print(f"Evaluating LUT: {lut[:5]}... with mic {mic} and rpm {rpm}")  # Debugging print statement
    return obj_func(mic, rpm),  # Return as tuple for DEAP compatibility

toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.1, indpb=0.1)
toolbox.register("select", tools.selTournament, tournsize=3)
toolbox.register("evaluate", evaluate)

def save_state(population, gen, best):
    """Save optimizer state to JSON."""
    state = {
        "generation": gen,
        "population": [list(ind) for ind in population],
        "best": list(best)
    }
    with open(STATE_FILE, "w") as f:
        json.dump(state, f)

def load_state():
    """Load optimizer state from JSON if available."""
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r") as f:
            state = json.load(f)
        return state["generation"], [creator.Individual(ind) for ind in state["population"]], state["best"]
    return 0, None, None

def save_best(best):
    """Save best LUT to JSON."""
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

    pool = create_pool()  # Create the multiprocessing pool

    try:
        while True:
            offspring = algorithms.varAnd(population, toolbox, CXPB, MUTPB)
            #print(f"Created offspring: {offspring[:2]}")  # Debugging print statement
            fits = pool.map(toolbox.evaluate, offspring)  # Parallel evaluation
            #print(f"Evaluated fitness values: {fits[:2]}")  # Debugging print statement

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

            gen += 1
            print(f"Generation: {gen}")
            print("best score so far: ", evaluate(best_ind))

    except KeyboardInterrupt:
        print("\nOptimization interrupted. Saving state...")
        save_state(population, gen, best_ind)
        print("State saved. Exiting.")
    finally:
        pool.close()
        pool.join()

if __name__ == "__main__":
    main()
