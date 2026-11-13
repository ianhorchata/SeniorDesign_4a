from controller import Robot
from controller import InertialUnit
from controller import LidarPoint
import numpy as np
import sys
import random

timestep = 64
robot = Robot()

# Unitree 4D Lidar Setup
lidar = robot.getDevice("lidar")
lidar.enable(100)            # 100 ms → 10 Hz → 64,000 pts/s with 16×400
lidar.enablePointCloud()     # for x, y, z points

#points = lidar.getPointCloud()       
#ranges = lidar.getRangeImage()      

# GPS enable
gps = robot.getDevice("gps")
gps.enable(timestep)
gpsValues = gps.getValues()

# IMU enable
imu = robot.getDevice("imu")
imu.enable(timestep)

# For Wheels
wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
avoidObstacleCounter = 0
while robot.step(timestep) != -1:
    leftSpeed = 1.0
    rightSpeed = 1.0
    
    wheels[0].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[3].setVelocity(rightSpeed)
    
