import numpy as np
import matplotlib.pyplot as plt

def trajectory():
    file = open("robot_pose.txt","r")
    lines = file.readlines()
    x = []
    y = []
    for line in lines:
        line = line.split(', ')
        x.append(float(line[0]))
        y.append(float(line[1]))

    fig = plt.figure()
    plt.plot(x,y)
    plt.show()

trajectory()

