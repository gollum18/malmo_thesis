#! /usr/bin/python

import numpy
import pandas

import statsmodels.formula.api as sm

import agent

MODELS_FILES = ["../stats/astar_models.txt", 
                "../stats/rrt_models.txt", 
                "../stats/threaded_rrt_models.txt"]
MODELS_FILE_HEADERS = ["A* Models:",
                        "RRT Models:", 
                        "CPU Threaded RRT Models:"]
STATS_FILES = ["../stats/astar_stats.txt", 
                "../stats/rrt_stats.txt", 
                "../stats/threaded_rrt_stats.txt"]
STATS_FILE_HEADERS = ["A* Statistics:",
                        "RRT Statistics:", 
                        "CPU Threaded RRT Statistics:"]

def get_pandas_data():
    """
    Reads in the raw data from the files stored in out.
    """
    astar_frames = []
    rrt_frames = []
    rrt_threaded_frames = []

    for array in [agent.MAPS_ASTAR, agent.MAPS_RRT, agent.MAPS_RRT_THREADED]:
        for filename in array:
            if array == agent.MAPS_ASTAR:
                astar_frames.append(pandas.read_csv(filepath_or_buffer=filename, memory_map=True, header=0))
            elif array == agent.MAPS_RRT:
                rrt_frames.append(pandas.read_csv(filepath_or_buffer=filename, memory_map=True, header=0))
            elif array == agent.MAPS_RRT_THREADED:
                rrt_threaded_frames.append(pandas.read_csv(filepath_or_buffer=filename, memory_map=True, header=0))

    return astar_frames, rrt_frames, rrt_threaded_frames

def write_models_to_file(pandas_data):
    for i in range(len(pandas_data)):
        with open(MODELS_FILES[i], 'w') as f:
            f.write(("-"*80) + "\n")
            f.write(MODELS_FILE_HEADERS[i] + "\n")
            f.write(("-"*80) + "\n")
            for j in range(len(pandas_data)):
                f.write("Map " + str(i+1) + ":\n")
                f.write(str(sm.ols(formula="", data=pandas_data[i][j]).fit()) + "\n")
                if i < 3: f.write("\n")
            f.write(("-" * 80) + "\n")

def write_stats_to_file(pandas_data):
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