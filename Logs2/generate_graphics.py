#!/usr/bin/python

import os
from fnmatch import fnmatch
import yaml
import numpy as np
import matplotlib.pyplot as plt
import math
from math import sqrt

root = '/home/elerson/NetworkedRobotsProject/Logs2/Logs/'
pattern = "*.txt"


files_list = []
for path, subdirs, files in os.walk(root):
    for name in files:
        if fnmatch(name, pattern):
            files_list.append(os.path.join(path, name))


def getInfo(file_name):
    fileHandle = open(file_name, "r")
    lineList = fileHandle.readlines()
    fileHandle. close()
    time_  = lineList[-1].rstrip().split(',')[-2]
    robots_ = lineList[-1].rstrip().split(',')[-1]
    return float(time_)/1000, float(robots_)



def filterFiles(file_list, algorithm, com_distance, experiment, type_):
    filtered_list = []
    for file_name in file_list:
        if algorithm in file_name and com_distance in file_name and experiment in file_name and type_ in file_name:
            filtered_list.append(file_name)
    return filtered_list


def getAnchorInfo(file):
    file_name   = file[:-11]+'network.txt'
    fileHandle  = open(file_name, "r")
    lineList    = fileHandle.readlines()
    fileHandle.close()
    steiner_str = lineList[-1] 
    graph       = yaml.load(steiner_str.split('steiner')[1].split('\'')[2])
    return lineList[-1]
    #print(lineList[-1])
    




algorithms  = ['gradient', 'cefsm']
radius      = ['60', '90', '110', '140', '170']
experiments = ['1euclideanexperiment', '2euclideanexperiment', '3euclideanexperiment', '5euclideanexperiment']


data = {}

for algorithm in algorithms:
    for experiment in experiments:
        for r in radius:
            files_ = filterFiles(files_list, algorithm, r, experiment, 'log.txt')
            time_total   = 0
            robots_total = 0
            count = 0.0000001
            time_vec = []
            for file_name in files_:
                #print(file_name)
                times, robots = getInfo(file_name)
                time_vec.append(times)
                time_total   += times
                robots_total += robots
                count        += 1
            data[(algorithm,experiment,r)] = (np.mean(time_vec), np.std(time_vec), round(robots_total/count))

                

2


#raio de communicação vs tempo
N    = 5
i = 1
for exp in experiments:
    #
    fig = plt.figure()
    ax  = plt.axes()
    #CEFSM GRAPHICS
    index = [s for s in data.keys() if 'cefsm' in s and exp in s]
    index = sorted(index, key=lambda tup: int(tup[2]))
    mean  = [ data[idx][2] for idx in index]
    #
    #
    #
    rects1 = ax.bar([1,4,7,10,13], mean, 1, color='r')
    #plt.plot(radius, mean, linewidth=2, label='CEFSM')
    #
    #Gradient GRAPHICS
    index  = [s for s in data.keys() if 'gradient' in s and exp in s]
    index  = sorted(index, key=lambda tup: int(tup[2]))
    mean   = [ data[idx][2] for idx in index]
    rects2 = ax.bar([2,5,8,11,14], mean, 1, color='g')
    #
    #
    ax.set_ylabel('Radius')
    ax.set_title('# Robots vs Radius')
    ax.set_xticks([2,5,8,11,14])
    ax.set_xticklabels(('60', '90', '110', '140', '170'))
    #
    ax.legend((rects1[0], rects2[0]), ('CEFSM', 'Proposed'))
    plt.title('# Robots vs Radius - ' + "Simulated Enviroment "+str(i)  ,  fontsize=20)
    i+= 1
    plt.xlabel('Radius',  fontsize=16)
    plt.ylabel('# Robots',  fontsize=16)
    #
    plt.legend(loc='best')
    plt.savefig('simulated_num_robots' + str(i) + 'pdf')
    #plt.show()







#raio de communicação vs tempo
N    = 5
i    = 1
for exp in experiments:
    #
    fig = plt.figure()
    ax  = plt.axes()
    #CEFSM GRAPHICS
    index = [s for s in data.keys() if 'cefsm' in s and exp in s]
    index = sorted(index, key=lambda tup: int(tup[2]))
    mean = [ data[idx][0] for idx in index]
    std_ = [ data[idx][1] for idx in index]
    error = list(1.96*np.array(std_)/sqrt(N))
    #
    plt.errorbar(radius, mean, error, linewidth=2, fmt='r*-', label='CEFSM')
    #
    #Gradient GRAPHICS
    index = [s for s in data.keys() if 'gradient' in s and exp in s]
    index = sorted(index, key=lambda tup: int(tup[2]))
    mean = [ data[idx][0] for idx in index]
    std_ = [ data[idx][1] for idx in index]
    error = list(1.96*np.array(std_)/sqrt(N))
    plt.errorbar(radius, mean, error, linewidth=2, fmt='gx-',  label='Proposed')
    #
    #
    plt.title('Time vs Radius - ' + "Simulated Enviroment "+str(i)  ,  fontsize=20)
    i += 1
    plt.xlabel('Radius',  fontsize=16)
    plt.ylabel('Time (seconds)',  fontsize=16)
    #
    plt.legend(loc='best')
    plt.savefig('simulated_time_robots' + str(i) + 'pdf')
    #plt.show()


# for exp in experiments:
#     index = [s for s in data.keys() if 'gradient' in s and exp in s]
#     index = sorted(index, key=lambda tup: int(tup[2]))
#     print([ data[idx][0] for idx in index])
#     print([ data[idx][1] for idx in index])






# #raio de communicação vs número de robots

# #ambiente vs numero de robots vs raio de communicacao


# mean = list(np.array(sum_['odom'])/N)
# std_ = list(np.sqrt(np.array(sum_2['odom'])/N - (np.array(sum_['odom'])/N)**2))








# for exp in experiments:
#     index = [s for s in data.keys() if 'cefsm' in s and exp in s]
#     index = sorted(index, key=lambda tup: int(tup[2]))
#     print([ data[idx][1] for idx in index])



# for exp in experiments:
#     index = [s for s in data.keys() if 'gradient' in s and exp in s]
#     index = sorted(index, key=lambda tup: int(tup[2]))
#     print([ data[idx][1] for idx in index])
