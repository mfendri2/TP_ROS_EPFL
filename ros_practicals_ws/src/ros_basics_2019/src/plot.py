#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np 
import pickle 
from itertools import izip, tee
from math import pow, atan2, sqrt , pi,atan
def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return izip(a, b)
#fix frequency 
freq=10
with open("/home/hedi/Documents/ROS/ros_practicals_ws/speed", 'rb') as f:
    speed = pickle.load(f)
with open("/home/hedi/Documents/ROS/ros_practicals_ws/postionx", 'rb') as f:
    positionx = pickle.load(f)
with open("/home/hedi/Documents/ROS/ros_practicals_ws/postiony", 'rb') as f:
	positiony = pickle.load(f)



speed_array_x=list() 
speed_array_y=list() 
speed_array_x[1:]=[freq*(y-x) for x,y in pairwise(positionx)]
speed_array_x[0]=0

 
speed_array_y[1:]=[freq*(y-x) for x,y in pairwise(positiony)]
speed_array_y[0]=0


 
plt.figure()
plt.plot(positionx)
plt.plot(positiony)
plt.grid(True)
plt.xlabel("Time samples")
plt.ylabel("Postion [m]")
plt.legend(("X postion","Y postion")) 
plt.savefig("Positions.png")
plt.figure()
plt.plot(speed)
#plt.plot(speed_array_x)
#plt.plot(speed_array_y)
speed_array_y=np.asarray(speed_array_y)
speed_array_x=np.asarray(speed_array_x)
plt.plot(np.sqrt(np.square(speed_array_x)+np.square(speed_array_x)))
plt.grid(True)

plt.xlabel("Time Samples")
plt.ylabel("Speed [m/s]")
#plt.legend(("command speed","Robot speed x","Robot speed y","robot speed norm"))
plt.legend(("command speed","robot speed norm"))
plt.savefig("speed.png")


