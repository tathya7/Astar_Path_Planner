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
    
# To convert it into visited node space representation
def modify_value(elem, thresh):
    modified = int(round(elem*2)/2)/thresh
    return modified


def generate_path(x_start,y_start,th_start, x_target, y_target, th_target, dics):
    path = []
    path.append((x_target, y_target))
    x,y, th = x_target,y_target, th_target
    while (x,y,th) in dics:
        path.append((x,y, th))
        (x,y, th) = dics[(x,y, th)]

    path.append((x_start,y_start, th_start))
    path.reverse()

    return path


############ MAP GENERATION ###############

# Create a white canvas which represents the clearance
map = np.full((500, 1200, 3), 255, dtype=np.uint8)
# Create a black canvas
cv2.rectangle(map, (clearance, clearance), (width-clearance, height-clearance), (0, 0, 0), thickness=cv2.FILLED)

map = np.full((500, 1200, 3), 255, dtype=np.uint8)
# Create a black canvas
cv2.rectangle(map, (clearance, clearance), (width-clearance, height-clearance), (0, 0, 0), thickness=cv2.FILLED)

# center1 = (650,250)
# length1 = 155
# Define the center and size of the hexagon
center2 = (650, 250)
length2 = 150

center1 = (650,250)
length1 = length2 + clearance

vertices1 = []
for i in range(6):
    x1 = int(center1[0] + length1 * np.cos(np.deg2rad(60 * i - 30)))
    y1 = int(center1[1] + length1 * np.sin(np.deg2rad(60 * i - 30)))
    vertices1.append((x1, y1))

# Calculate the vertices of the hexagon
vertices2 = []
for i in range(6):
    x2 = int(center2[0] + length2 * np.cos(np.deg2rad(60 * i - 30)))
    y2 = int(center2[1] + length2 * np.sin(np.deg2rad(60 * i - 30)))
    vertices2.append((x2, y2))

# Convert vertices to numpy array
pts1 = np.array(vertices1, np.int32)
pts1 = pts1.reshape((-1, 1, 2))

# Convert vertices to numpy array
pts2 = np.array(vertices2, np.int32)
pts2 = pts2.reshape((-1, 1, 2))

cv2.fillPoly(map, [pts1], color=(255, 255, 255))
cv2.fillPoly(map, [pts2], color=(0, 180, 0))

################## RECT1 ##################################
cv2.rectangle(map, (100-clearance, 0), (175+clearance, 400+clearance), (255, 255, 255), thickness=cv2.FILLED)
cv2.rectangle(map, (100, 0), (175, 400), (0, 180, 0), thickness=cv2.FILLED)

################## RECT2 ##################################
cv2.rectangle(map, (275-clearance, 500), (350+clearance, 100-clearance), (255, 255, 255), thickness=cv2.FILLED)
cv2.rectangle(map, (275, 500), (350, 100), (0, 180, 0), thickness=cv2.FILLED)



#################### C SHAPE ##############################

# White Rect rep clearance
cv2.rectangle(map, (900-clearance, 50-clearance), (1100+clearance, 450+clearance), (255, 255, 255), thickness=cv2.FILLED)

# First green rect
cv2.rectangle(map, (900, 50), (1100, 450), (0, 180, 0), thickness=cv2.FILLED)

# Second Smaller White Rect
cv2.rectangle(map, (900-clearance, 135), (1025, 375), (255, 255, 255), thickness=cv2.FILLED)

# Final Black Rect
cv2.rectangle(map, (900-clearance, 125+clearance+clearance), (1020-clearance, 375-clearance), (0, 0, 0), thickness=cv2.FILLED)
