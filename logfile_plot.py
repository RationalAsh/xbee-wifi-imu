#!/usr/bin/python

SENSOR_TO_PLOT = 0

from pylab import *

f = open('log.txt', 'r')

dataset = []

for i in xrange(39000):
    line = f.readline()
    sensor_strings = [e.strip() for e in line.split("||")]    
    #print sensor_strings
    sensor_str2 = [e.split(" ") for e in sensor_strings]
    sensor_vals = []

    for string in sensor_str2:
        try:
            temp = map(float, string)
        except ValueError:
            temp = [0.0, 0.0, 0.0]
        sensor_vals.append(temp)

    dataset.append(sensor_vals)

    #print sensor_vals


t = np.arange(0.0,39000.0)/100
ctr = 0

for s in xrange(7):
    yaw = []
    pitch = []
    roll = []

    for line in dataset:
        yaw.append(line[s][0])
        pitch.append(line[s][1])
        roll.append(line[s][2])

    figure(ctr)
    plot(t, yaw, label='yaw')
    plot(t, pitch, label='pitch')
    plot(t, roll, label='roll')
    legend(framealpha=0.5)
    xlabel('time')
    ylabel('value')
    ctr += 1

show()



    
    
