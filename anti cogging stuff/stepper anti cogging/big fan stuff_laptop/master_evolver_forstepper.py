import json
import time
import random
import numpy as np
from deap import base, creator, tools, algorithms
from datetime import datetime
import os
import numpy as np
from mic_reader2 import listen_mic as read_mic
from rpm_reader import read_rpm
from lutsender2_done_stepper import send_lut

starting_rpm = read_rpm() #it's actually rads per second oh well
def obj_func(lut, mic, rpm): # lut is in the form of a list.
    global starting_rpm
    mic_noise_floor = 3000
    lut_sum = 0
    for i in lut:
        lut_sum = lut_sum+i # net deviation from zero, waves whose peak and trough cancel out dont' contribute to this sum
    lut_offset_size = abs(lut_sum) #negative or positive net offset of average voltage counts the same
    print("lut_sum:", lut_sum)
    value = (mic-mic_noise_floor)*10+lut_offset_size*30 # mic minus X is because it reads 4366 when quiet so reduce noise by subtracting that, check the values are reasonabl efor this linerpm is minus because it's going that direction but minus a negative. The lut sum term is to try to reduce any tendency to produce a net voltage offset/keep it to the minimum mods.
    print("obj func evaluates to:",value)
    return value
tests = 0
# Constants
LUT_SIZE = 50
POP_SIZE = 10
CXPB, MUTPB = 0.5, 0.2  # Crossover & mutation probabilities
GENS = 1000
STATE_FILE = "optimizer_state.json"
BEST_FILE = "best_lut.json"
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
    time.sleep(5)
    mic = read_mic()
    rpm = read_rpm()
    tests = tests+1
    print("tests: ",tests)
    return obj_func(list(individual),mic, rpm),

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
    with open(BEST_FILE, "w") as f:
        json.dump(best, f)


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

