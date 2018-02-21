#! /usr/bin/python

import numpy
import pandas

import seaborn as sns
import statsmodels.api as sm
from statsmodels.formula.api import ols

import agent

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

def seaborn_plot(pandas_data):
    """
    Outputs a plot of all data using seaborn.
    """
    for i in range(len(pandas_data)):
        for j in range(len(pandas_data[i])):
            sns.regplot(x=agent.CSV_HEADERS[2], y=agent.CSV_HEADERS[3], data=pandas_data[i][j])

def write_models_to_file(pandas_data):
    """
    Writes statistical models from the files contained in out to file.
    """
    for i in range(len(pandas_data)):
        with open(MODELS_FILES[i], 'w') as f:
            f.write(("-"*80) + "\n")
            f.write(MODELS_FILE_HEADERS[i] + "\n")
            f.write(("-"*80) + "\n")
            for j in range(len(pandas_data[0])):
                f.write("Map " + str(j+1) + " Type 2 Anova:\n")
                f.write(str(sm.stats.anova_lm(ols(formula=OLS_FORMULA, data=pandas_data[i][j]).fit(), typ=2)) + "\n")
                if i < 3: f.write("\n")
            f.write(("-" * 80) + "\n")

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
    data = get_pandas_data()
    write_stats_to_file(data)