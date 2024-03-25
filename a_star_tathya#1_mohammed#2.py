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

################### MAIN A-STAR ALGO ##############

moves = [Move1, Move2, Move3, Move4, Move5]

check = True

while check:
    x_start = int(input("Enter the initial X position ({} to {}): ".format(0+clearance, width-clearance-1)))
    y_start = int(input("Enter the initial Y Position ({} to {}): ".format(0+clearance, height-clearance-1)))
    th_start = int(input("Enter the initial Orientation (0 to 360): "))
    print("Your Start Node Is (X,Y,Angle): ", x_start, y_start, th_start)
    # Converting the coordinates to the instructed coordinate system
    y_start = height-y_start-1
    # Checks if the given node is in the free space
    if map[y_start, x_start,1] == 0:
        cv2.circle(map,(x_start, y_start), 2, (0, 180, 0), -1)
        check = False
    # If the starting node is in obstacle space
    else:
        print("Starting Position is in the Obstacle space! Re-Enter the Position")

check = True

while check:
    x_goal = int(input("Enter the Destination X Position ({} to {}): ".format(0+clearance, width-clearance-1)))
    y_goal = int(input("Enter the Destination Y Position ({} to {}): ".format(0+clearance, height-clearance-1)))
    th_goal= int(input("Enter the Goal Orientation (0 to 360): "))

    print("Your Goal Node Is (X,Y,Angle): ", x_goal, y_goal, th_goal)
    # Converting the coordinates to the instructed coordinate system
    y_goal = height-y_goal-1 
    # Checks if the given node is in the free space
    if map[y_goal, x_goal,1] == 0:
        # Checks if the start node and goal node are same
        if (x_start, y_start) == (x_goal, y_goal):
            print("Error! Start and Goal Position Cannot be Same")
        else:
            check = False
    else:
        print("Starting Position is in the Obstacle space! Re-Enter the Position")


start = time.time()
q = []
heapq.heappush(q,(0,x_start,y_start,th_start))
child2parent = {}

# Modify the initial values to the visited space
x_start_mod = modify_value(x_start,dist_threshold)
y_start_mod = modify_value(y_start,dist_threshold)
th_start_mod = modify_value(th_start, theta_threshold)

#Initializing visited
visited = {(x_start_mod, y_start_mod, th_start_mod):True}
node_cost = {(x_start_mod, y_start_mod, th_start_mod):0} 
cost2come = {(x_start_mod, y_start_mod, th_start_mod):0}

reached = False

while q:
    cst, x_pos, y_pos,th = heapq.heappop(q)

    x_mod = modify_value(x_pos,dist_threshold)
    y_mod = modify_value(y_pos,dist_threshold)
    th_mod = modify_value(th,theta_threshold)
    prev_cst = cost2come[(x_mod,y_mod,th_mod)] 

    if goal_check(x_pos, y_pos,th, x_goal, y_goal, th_goal) == True:
        print("Goal Reached")
        reached = True
        break

    for move in moves:
        node = move(x_pos,y_pos,th)
        if node is not None:
            new_x, new_y, new_th = node

            # Converting it to int to plot in map
            x_map = int(round(new_x*2)/2)
            y_map = int(round(new_y*2)/2)
            th_map = int(round(new_th*2)/2)


            if 0 <= new_x < width-1 and 0 <= new_y < height-1 and map[y_map, x_map, 1] == 0:

                # Modifying for visited node space
                newx_mod = modify_value(x_map, dist_threshold)
                newy_mod = modify_value(y_map, dist_threshold)
                newth_mod = modify_value(th_map, theta_threshold)

                if (newx_mod, newy_mod, newth_mod) not in visited:


                    cost2come[(newx_mod,newy_mod,newth_mod)] = prev_cst + ssize
                    node_cost[(newx_mod,newy_mod,newth_mod)] = cost2come[(newx_mod,newy_mod,newth_mod)] + dist((new_x, new_y), (x_goal, y_goal))

                    heapq.heappush(q,(node_cost[(newx_mod,newy_mod,newth_mod)], new_x, new_y, new_th))
 
                    child2parent[(new_x, new_y,new_th)] = (x_pos, y_pos, th)
                    visited[(newx_mod,newy_mod,newth_mod)] = True

                if prev_cst > prev_cst + ssize :
                    cost2come[(newx_mod,newy_mod,newth_mod)] = prev_cst + ssize 
                    node_cost[(newx_mod,newy_mod,newth_mod)] = cost2come[(newx_mod,newy_mod,newth_mod)] +  dist((new_x, new_y), (x_goal, y_goal))
                    child2parent[(new_x, new_y,new_th)] = (x_pos, y_pos, th)

end = time.time()          

# If the goal is not reachable
if reached == False:
     print("Goal out of bounds")

# Printing the runtime of the algorithm
print("Generating Video..., Algorithm Time is: ", (end-start))

############ GENERATING VIDEO ##############

gen_path = generate_path(x_start,y_start, th_start, x_pos,y_pos, th, child2parent)

path_vid = cv2.VideoWriter('a_star_path_final.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 50, (width, height))


cv2.circle(map, (x_start,y_start), 6, (92,11,227),-1)
cv2.circle(map, (x_goal,y_goal), 6, (0, 165, 255),-1)

num_frames = 350
j = 0

for (x_,y_,angle) in child2parent:
    j+=1
    x1 = int(child2parent[(x_,y_,angle)][0])
    y1 = int(child2parent[(x_,y_,angle)][1])
    cv2.arrowedLine(map,(x1,y1), (int(x_),int(y_)), (225, 105, 65),tipLength=0.2)
    if j == num_frames:
        cv2.imshow('Video', map)
        path_vid.write(map)
        cv2.waitKey(1)  
        j=0

cv2.circle(map, (x_start,y_start), 6, (92,11,227),-1)
cv2.circle(map, (x_goal,y_goal), 6 , (0, 165, 255),-1)

for i in range(len(gen_path)-1):
    cv2.arrowedLine(map,(int(gen_path[i][0]),int(gen_path[i][1])), (int(gen_path[i+1][0]),int(gen_path[i+1][1])), (0, 0, 255),tipLength=0.6)
    cv2.imshow('Video', map)
    path_vid.write(map)
    cv2.waitKey(1) 

last_frame = map
for _ in range(200): 
    path_vid.write(last_frame)

path_vid.release()


cv2.waitKey(0)
cv2.destroyAllWindows()
