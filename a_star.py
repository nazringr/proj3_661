#!/usr/bin/env python
# coding: utf-8

import numpy as np
from math import dist
import matplotlib.pyplot as plt
import time
import heapq
import os
import cv2

# Initializing the class for our "Node" objects
class Node:

    def __init__(self, x, y, orient, cost, parent_id, cost_to_go = 0):
        self.x = x
        self.y = y
        self.orient = orient
        self.cost = cost
        self.parent_id = parent_id
        self.cost_to_go = cost_to_go 
        
    def __lt__(self,other):
        return self.cost + self.cost_to_go < other.cost + other.cost_to_go

# Defining the action set, and calculating the costs
def move_60up(x,y,orient,step_size, cost):
    orient = orient + 60
    x = x + (step_size*np.cos(np.radians(orient)))
    y = y + (step_size*np.sin(np.radians(orient)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,orient,cost

def move_60down(x,y,orient, step_size, cost):
    orient = orient - 60
    x = x + (step_size*np.cos(np.radians(orient)))
    y = y + (step_size*np.sin(np.radians(orient)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,orient,cost

def move_30up(x,y,orient, step_size, cost):
    orient = orient + 60
    x = x + (step_size*np.cos(np.radians(orient)))
    y = y + (step_size*np.sin(np.radians(orient)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,orient, cost

def move_30down(x,y,orient, step_size, cost):
    orient = orient - 30
    x = x + (step_size*np.cos(np.radians(orient)))
    y = y + (step_size*np.sin(np.radians(orient)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,orient, cost

def move_straight(x,y,orient, step_size, cost):
    orient = orient + 0
    x = x + (step_size*np.cos(np.radians(orient)))
    y = y + (step_size*np.sin(np.radians(orient)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,orient, cost

# Initializing the action set functions
def Action_set(move,x,y,orient,step_size,cost):

    if move == 'extreme_right':
        return move_60up(x,y,orient, step_size,cost)
    elif move == 'right':
        return move_30up(x,y,orient, step_size,cost)
    elif move == 'straight':
        return move_straight(x,y,orient,step_size,cost)
    elif move == 'left':
        return move_30down(x,y,orient,step_size,cost)
    elif move == 'extreme_left':
        return move_60down(x,y,orient,step_size,cost)
    else:
        return None

#Initializing the canvas with the obstacle definitions
def obstacle_space(width, height, total_clearance, robot_radius):

    obs_space_array = np.full((height,width),0)
    
    for y in range(height) :
        for x in range(width):
        
            # Boundary total_clearance with buffer
            if x<total_clearance+robot_radius or x>width-(total_clearance+robot_radius) or y<total_clearance+robot_radius or y>height-(total_clearance+robot_radius):
                obs_space_array[y,x] = 1  # Filled as obstacle

            # Rightmost mirrored C shaped obstacle
            if x>900-total_clearance+robot_radius and x<1100+total_clearance+robot_radius and y>50-(total_clearance+robot_radius) and y<125+total_clearance+robot_radius:
                obs_space_array[y,x] = 1
            if x>1020-(total_clearance+robot_radius) and x<1100+total_clearance+robot_radius and y>125-(total_clearance+robot_radius) and y<375+total_clearance+robot_radius:
                obs_space_array[y,x] = 1      
            if x>900-(total_clearance+robot_radius) and x<1100+total_clearance+robot_radius and y>375-(total_clearance+robot_radius) and y<450+total_clearance+robot_radius:
                obs_space_array[y,x] = 1  

            # Left rectangle obstacle
            if x>100-(total_clearance+robot_radius) and x<175+total_clearance+robot_radius and y>100-(total_clearance+robot_radius):
                obs_space_array[y,x] = 1

            # Right rectangle obstacle
            if x>275-(total_clearance+robot_radius) and x<350+total_clearance+robot_radius and y<400+total_clearance+robot_radius:
                obs_space_array[y,x] = 1

            # Hexagonal obstacle
            # Appropriate total_clearance buffer has been added to the  
            # intercept values for simpler implementation
            if x<785+(total_clearance+robot_radius) and x>515-(total_clearance+robot_radius):
                if 1.73*y+x-(1342.25+2*(total_clearance+robot_radius))<0:
                    if 1.73*y+x-(823-2*(total_clearance+robot_radius))>0:
                        if 1.73*y-x-(42+2*(total_clearance+robot_radius))<0:
                            if 1.73*y-x+477.25+2*(total_clearance+robot_radius)>0:
                                obs_space_array[y,x] = 1              
    
    return obs_space_array
    
# Generating random key for nodes
def key(node):
    key = 1022*node.x + 111*node.y 
    return key

# Checking whether the goal is reached or not
def goal_reached(present, goal):
    
    dt = dist((present.x, present.y), (goal.x, goal.y))             

    if dt < 1.5 and present.orient == goal.orient:    # Goal threshold. Given=1.5
        return True
    else:
        return False
    
# Checking for the move validity on boundary conditions
def avail_move(x, y, obs_space_array):

    e = obs_space_array.shape

    if( x > e[1] or x < 0 or y > e[0] or y < 0 ):
        return False
    
    else:
        try:
            if(obs_space_array[y][x] == 1  or obs_space_array[y][x]==2):
                return False
        except:
            pass
    return True

# Checking for valid orientation
def valid_orientation(orient):
    if((orient%30) == 0):
        return orient
    elif(orient == 0):
        return orient
    else:
        return False


# A* Algorithm
def a_star_algorithm(start,goal,obs_space_array,step_size):                       

    if goal_reached(start, goal):
        return None,1
    goal_node = goal
    start_node = start
    
    available_moves = ['extreme_right','right', 'straight', 'left', 'extreme_left']   
    unexplored_nodes = {}  # List of all open nodes
    
    start_key = key(start_node) # New key for pointing new node
    unexplored_nodes[(start_key)] = start_node
    
    nodes_explored = {} #List of all closed nodes
    priority_list = []  #List to store all dictionary entries with cost as the sorting variable
    heapq.heappush(priority_list, [start_node.cost, start_node]) # Least cost entries will be prioritized
    
    all_nodes = [] #storing all the nodes that have been traversed, for visualization purposes.

    while (len(priority_list) != 0):

        current_node = (heapq.heappop(priority_list))[1]
        all_nodes.append([current_node.x, current_node.y, current_node.orient])          
        current_id = key(current_node)

        # If the goal is reached
        if goal_reached(current_node, goal_node):
            goal_node.parent_id = current_node.parent_id
            goal_node.cost = current_node.cost
            print("Found the GOAL!")
            return all_nodes,1

        if current_id in nodes_explored:  
            continue
        else:
            nodes_explored[current_id] = current_node
        
        del unexplored_nodes[current_id]

        for move in available_moves:
            x,y,orient,cost = Action_set(move,current_node.x,current_node.y,current_node.orient, step_size, current_node.cost)  
            
            # Calculating the cost-to-go
            cost_to_go = dist((x, y), (goal.x, goal.y))  

            new_node = Node(x,y,orient, cost,current_node, cost_to_go)   

            new_node_id = key(new_node) 

            if not avail_move(new_node.x, new_node.y, obs_space_array):
                continue
            elif new_node_id in nodes_explored:
                continue

            # If unvisited node
            if new_node_id in unexplored_nodes:
                if new_node.cost < unexplored_nodes[new_node_id].cost: 
                    unexplored_nodes[new_node_id].cost = new_node.cost
                    unexplored_nodes[new_node_id].parent_id = new_node.parent_id
            else:
                unexplored_nodes[new_node_id] = new_node
            
            heapq.heappush(priority_list, [(new_node.cost + new_node.cost_to_go), new_node]) 

    return  all_nodes,0

# backtracking the path and getting the "reversed" path
def backtrack_path(goal_node):  
    x_path = []
    y_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)

    parent_node = goal_node.parent_id
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        parent_node = parent_node.parent_id
        
    x_path.reverse()
    y_path.reverse()
    
    x = np.asarray(x_path)
    y = np.asanyarray(y_path)
    
    return x,y

# Plotting and capturing the progress
def plot(start_node, goal_node, x_path, y_path, all_nodes, obs_space_array, frame_count, ideal_path):
    plt.figure()
    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")

    # Plotting the canvas
    plt.imshow(obs_space_array, "GnBu")
    ax = plt.gca()
    ax.invert_yaxis() # Y-axis inversion
    
    # plotting all the visited nodes
    for i in range(len(all_nodes)):
        plt.plot(all_nodes[i][0], all_nodes[i][1], "2g-")

    # Plotting the final path once all nodes are plotted
    if ideal_path:  # Checks if 'yes'
        plt.plot(x_path[:frame_count+1], y_path[:frame_count+1], ':r')

    # Saving each frame as an image
    plt.savefig(f"frame_{frame_count}.png")
    plt.close()

# Creating video from all the saved frames
def generate_video(frame_prefix, generate_video_path, frame_rate):
    frames = []
    frame_files = [f for f in os.listdir() if f.startswith(frame_prefix) and f.endswith('.png')]
    frame_files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))  # Sorting the frames by the frame number
    for frame_file in frame_files:
        frames.append(cv2.imread(frame_file))
        os.remove(frame_file)  # Deleting the frames after appending them to the video

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    write_video = cv2.VideoWriter(generate_video_path, fourcc, frame_rate, (frames[0].shape[1], frames[0].shape[0]))

    # Writing the frames to video
    for frame in frames:
        write_video.write(frame)

    # Releasing the video writer
    write_video.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    
    # Taking Input of the desired total_clearance
    total_clearance = input("Please input the obstacle clearance: ")
    total_clearance = int(total_clearance)
    
    # Taking Input of the desired radius of the robot
    robot_radius = input("Please input the Robot Radius: ") 
    robot_radius = int(robot_radius)
    
    # Taking Input of the desired step size for A*
    robot_step_size = input("Please input the Robot Step size(between 1 to 10): ")
    robot_step_size = int(robot_step_size)
    
    width = 1200
    height = 500
    obs_space_array = obstacle_space(width, height, total_clearance, robot_radius)
    cost_to_go = 0 # Cost-to-Go
    
    # Taking Input of the desired starting coordinates
    s_x = int(input("Enter the x coordinate for the Start Node: "))
    s_y = int(input("Enter the y coordinate for the Start Node: "))

    # Checking if it's a valid move
    if not avail_move(s_x, s_y, obs_space_array):
        print("Start node is out of bounds(or in obstacle zone)")
        exit(-1)

    # Taking Input of the desired starting orientation of the robot
    s_orient = input("Enter Orientation of the robot at start node: ")
    s_t = int(s_orient)
    
    # Check if it's a valid orientation
    if not valid_orientation(s_t):
        print("Please enter a multiple of 30")
        exit(-1)
            
    # Taking Input of the desired goal coordinates 
    g_x = int(input("Enter the x-coordinate for the Goal Node: "))
    g_y = int(input("Enter the y-coordinate for the Goal Node: "))
    
    ## Checking if it's a valid move
    if not avail_move(g_x, g_y, obs_space_array):
        print("Goal node is out of bounds(or in obstacle zone)")
        exit(-1)

    # Taking input of the desired goal orientation of the robot
    g_orient = input("Enter the Orientation of the robot at the goal node: ")
    g_t = int(g_orient)

    ## Checking if it's a valid orientation
    if not valid_orientation(g_t):
        print("Please enter a multiple of 30")
        exit(-1)

    # Set Timer to calculate the processing time
    timer_start = time.time()
    
    # Initializing the start and goal node 
    start_node = Node(s_x, s_y,s_t, 0.0, -1,cost_to_go)
    goal_node = Node(g_x, g_y,g_t, 0.0, -1, cost_to_go)
    all_nodes,flag = a_star_algorithm(start_node, goal_node, obs_space_array, robot_step_size)
    
    # Calculating the optimal path from start to the goal node 
    if (flag)==1:
        x_path,y_path = backtrack_path(goal_node)
    else:
        print("No path could be found")
        
    # Plotting the exploration, and the final path
    frame_count = 0  # Initializing frame count
    for i in range(len(all_nodes)):
        plot(start_node, goal_node, x_path, y_path, all_nodes[:i+1], obs_space_array, frame_count, ideal_path=False)
        frame_count += 1
        if(frame_count%15==0):
            print("Generating video: ", int((i/len(all_nodes))*70), "%")

    for i in range(len(x_path)):
        plot(start_node, goal_node, x_path, y_path, all_nodes, obs_space_array, frame_count, ideal_path=True)
        frame_count += 1
        if(frame_count%15==0):
            print("Generating video: ", 70+int((i/len(x_path))*30), "%")

    # Creating the video from the saved frames
    generate_video("frame", "visualize_path.mp4", 30)  # Default frame rate = 30

    timer_stop = time.time()
    C_time = timer_stop - timer_start
    print("Finished executing in:  ", C_time) 
    