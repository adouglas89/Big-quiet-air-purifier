import json
import time
import random
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
    dc_pen = abs(np.mean(lut))*500
    print("dc_pen:", dc_pen)
    value = (mic)*10+dc_pen # mic minus X is because it reads 4366 when quiet so reduce noise by subtracting that, check the values are reasonabl efor this linerpm is minus because it's going that direction but minus a negative. The lut sum term is to try to reduce any tendency to produce a net voltage offset/keep it to the minimum mods.
    print("obj func evaluates to:",value)
    return value
tests = 0
# Constants
LUT_SIZE = 200
POP_SIZE = 10
CXPB, MUTPB = 0.5, 0.2  # Crossover & mutation probabilities
GENS = 1000
STATE_FILE = "optimizer_state.json"
BEST_FILE = "best_lut.json"
# DEAP setup
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, 0, 0)  # Adjust range if needed
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
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.25)
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

def load_lut():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE, "r") as f:
            state = json.load(f)
        return state["best"]

lut=load_lut()
print(lut)




