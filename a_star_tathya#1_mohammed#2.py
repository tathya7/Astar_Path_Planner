import numpy as np
import cv2
import heapq
import time
from math import dist

############### Initial Params ##############

width = 1200
height = 500
dist_threshold = 0.5
theta_threshold = 30

check = True
while check:
    ssize = int(input("Enter the Step Size (1-10): "))
    if 1<= ssize <=10:
        print("Step Size:", ssize)
        check = False
    else:
        print("Step size should be in between 1 and 10")

clearance = int(input("Enter the Clearance for the obstacle space:"))
rob_radius =  int(input("Enter the Radius of the robot:"))

clearance = clearance + rob_radius

############ Defining Action Sets #############

# Defining Move 1 representing straight movement
def Move1(x_pos, y_pos, theta):
    angle = theta + 0

    x = x_pos + ssize * np.cos(np.deg2rad(angle))
    y = y_pos + ssize * np.sin(np.deg2rad(angle))
    # cost = ssize
    return x,y,angle

# Defining Move 2 representing 30 degree movement
def Move2(x_pos, y_pos, theta):
    angle = theta + 30
    if angle > 180:
        angle -= 360

    x = x_pos + ssize * np.cos(np.deg2rad(angle))
    y = y_pos + ssize * np.sin(np.deg2rad(angle))
    # cost = ssize
    return x,y,angle

# Defining Move 3 representing 60 degree movement
def Move3(x_pos, y_pos, theta):
    angle = theta + 60
    if angle > 180:
        angle -= 360

    x = x_pos + ssize * np.cos(np.deg2rad(angle))
    y = y_pos + ssize * np.sin(np.deg2rad(angle))
    # cost = ssize
    return x,y,angle

# Defining Move 4 representing -30 degree movement
def Move4(x_pos, y_pos, theta):
    angle = theta - 30
    if angle < -180:
        angle += 360

    x = x_pos + ssize * np.cos(np.deg2rad(angle))
    y = y_pos + ssize * np.sin(np.deg2rad(angle))
    # cost = ssize
    return x,y,angle

# Defining Move 5 representing -60 degree movement
def Move5(x_pos, y_pos, theta):
    angle = theta - 60
    if angle < -180:
        angle += 360

    x = x_pos + ssize * np.cos(np.deg2rad(angle))
    y = y_pos + ssize * np.sin(np.deg2rad(angle))
    # cost = ssize
    return x,y,angle

# Checks if the node is goal node or not
def goal_check(x_curr, y_curr, th_curr, x_tar, y_tar, th_tar):
    if np.sqrt((x_curr-x_tar)**2 + (y_curr-y_tar)**2) <=1.5 and abs(th_curr-th_tar) <= theta_threshold:
        return True
    else:
        return False
    
