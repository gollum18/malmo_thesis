#! /usr/bin/python

import numpy
import pandas
import random
import statistics

import seaborn as sns
import statsmodels.stats as stats
import statsmodels.api as sm
from statsmodels.formula.api import ols

import agent

SAMPLE_SIZE = 200
KEYS = ["RT", "PL", "HC", "DEG"]

MODELS_FILES = ["../stats/astar_models.txt", 
                "../stats/rrt_models.txt", 
                "../stats/threaded_rrt_models.txt",
                "../stats/gpu_rrt_models.txt"]
MODELS_FILE_HEADERS = ["A* Models:",
                        "RRT Models:", 
                        "CPU Threaded RRT Models:",
                        "GPU RRT Models:"]
STATS_FILES = ["../stats/astar_stats.txt", 
                "../stats/rrt_stats.txt", 
                "../stats/threaded_rrt_stats.txt",
                "../stats/gpu_rrt_stats.txt"]
STATS_FILE_HEADERS = ["A* Statistics:",
                        "RRT Statistics:", 
                        "CPU Threaded RRT Statistics:",
                        "GPU RRT Statistics:"]
OLS_FORMULA = "{0} ~ {1} + {2} + {3}".format(agent.CSV_HEADERS[0], agent.CSV_HEADERS[1],
                    agent.CSV_HEADERS[2], agent.CSV_HEADERS[3])

def get_pandas_data():
    """
    Reads in the raw data from the files stored in out.
    """
    astar_frames = []
    rrt_frames = []
    rrt_threaded_frames = []
    gpu_frames = []

    for array in [agent.MAPS_ASTAR, agent.MAPS_RRT, agent.MAPS_RRT_THREADED, agent.MAPS_RRT_GPU]:
        for filename in array:
            if array == agent.MAPS_ASTAR:
                astar_frames.append(pandas.read_csv(filepath_or_buffer=filename, memory_map=True, header=0))
            elif array == agent.MAPS_RRT:
                rrt_frames.append(pandas.read_csv(filepath_or_buffer=filename, memory_map=True, header=0))
            elif array == agent.MAPS_RRT_THREADED:
                rrt_threaded_frames.append(pandas.read_csv(filepath_or_buffer=filename, memory_map=True, header=0))
            elif array == agent.MAPS_RRT_GPU:
                gpu_frames.append(pandas.read_csv(filepath_or_buffer=filename, memory_map=True, header=0))

    return astar_frames, rrt_frames, rrt_threaded_frames, gpu_frames

def random_sample(pandas_data):
    """
    Returns a normalized random sample from the overall population.
    pandas_data: Pandas data frame that contains the population.
    """
    # pandas_data contains a list of data frame objects
    #   each data frame object contains
    random_indices = set()
    while(len(random_indices) < SAMPLE_SIZE):
        random_indices.add(random.randint(0, 999))

    # This is a literal mess, but it should work
    astar_samples = {0:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    1:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    2:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    3:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}}
    rrt_samples = {0:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    1:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    2:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    3:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}}
    rrt_threaded_samples = {0:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    1:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    2:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    3:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}}
    gpu_samples = {0:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    1:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    2:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}, 
                    3:{KEYS[0]:[], KEYS[1]:[], KEYS[2]:[], KEYS[3]:[]}}
    for array in range(len(pandas_data)):
        for frame in range(len(pandas_data[array])):
            for index in random_indices:
                # Access the data frame calling
                #   pandas_data[array][frame]
                if array == 0:
                    astar_samples[frame][KEYS[0]].append(
                        pandas_data[array][frame].get(KEYS[0])[index])
                    astar_samples[frame][KEYS[1]].append(
                        pandas_data[array][frame].get(KEYS[1])[index])
                    astar_samples[frame][KEYS[2]].append(
                        pandas_data[array][frame].get(KEYS[2])[index])
                    astar_samples[frame][KEYS[3]].append(
                        pandas_data[array][frame].get(KEYS[3])[index])
                elif array == 1:
                    rrt_samples[frame][KEYS[0]].append(
                        pandas_data[array][frame].get(KEYS[0])[index])
                    rrt_samples[frame][KEYS[1]].append(
                        pandas_data[array][frame].get(KEYS[1])[index])
                    rrt_samples[frame][KEYS[2]].append(
                        pandas_data[array][frame].get(KEYS[2])[index])
                    rrt_samples[frame][KEYS[3]].append(
                        pandas_data[array][frame].get(KEYS[3])[index])
                elif array == 2:
                    rrt_threaded_samples[frame][KEYS[0]].append(
                        pandas_data[array][frame].get(KEYS[0])[index])
                    rrt_threaded_samples[frame][KEYS[1]].append(
                        pandas_data[array][frame].get(KEYS[1])[index])
                    rrt_threaded_samples[frame][KEYS[2]].append(
                        pandas_data[array][frame].get(KEYS[2])[index])
                    rrt_threaded_samples[frame][KEYS[3]].append(
                        pandas_data[array][frame].get(KEYS[3])[index])
                elif array == 3:
                    gpu_samples[frame][KEYS[0]].append(
                        pandas_data[array][frame].get(KEYS[0])[index])
                    gpu_samples[frame][KEYS[1]].append(
                        pandas_data[array][frame].get(KEYS[1])[index])
                    gpu_samples[frame][KEYS[2]].append(
                        pandas_data[array][frame].get(KEYS[2])[index])
                    gpu_samples[frame][KEYS[3]].append(
                        pandas_data[array][frame].get(KEYS[3])[index])
    return (astar_samples, 
            rrt_samples, 
            rrt_threaded_samples, 
            gpu_samples)

def seaborn_plot(pandas_data):
    """
    Outputs a plot of all data using seaborn.
    """
    for i in range(len(pandas_data)):
        for j in range(len(pandas_data[i])):
            sns.regplot(x=agent.CSV_HEADERS[2], y=agent.CSV_HEADERS[3], data=pandas_data[i][j])

def write_stats_to_file(pandas_data):
    """
    Writes statistics from the files contained in out to file.
    """
    for iteration in range(len(pandas_data)):
        with open(STATS_FILES[iteration], 'w') as f:
            f.write(("-" * 80) + "\n")
            f.write(STATS_FILE_HEADERS[iteration] + "\n")
            f.write(("-" * 80) + "\n")
            for i in range(len(pandas_data[0])):
                f.write("Map " + str(i+1) + ":\n")
                f.write(str(pandas_data[iteration][i].describe()) + "\n")
                if i < 3: f.write("\n")
            f.write(("-" * 80) + "\n")

if __name__ == '__main__':
    samples = random_sample(get_pandas_data())
    astar_samples = samples[0]
    gpu_samples = samples[3]
    with open("../stats/sample_data.txt", 'w') as f:
        for i in range(4):
            f.write("A* Sample Data Map {0}\r\n".format(i+1))
            f.write(str(pandas.DataFrame.from_dict(astar_samples[i]).describe())+"\r\n")
            f.write("RRT-GPU Sample Data Map {0}:\r\n".format(i+1))
            f.write(str(pandas.DataFrame.from_dict(gpu_samples[i]).describe())+"\r\n")
            f.write("\r\n")