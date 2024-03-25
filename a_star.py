#!/usr/bin/env python
# coding: utf-8

import numpy as np
from math import dist
import matplotlib.pyplot as plt
import time
import heapq
import os
import cv2

# Initialising the class for our "Node" objects
class Node:

    def __init__(self, x, y, theta, cost, parent_id, cost_to_go = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.parent_id = parent_id
        self.cost_to_go = cost_to_go 
        
    def __lt__(self,other):
        return self.cost + self.cost_to_go < other.cost + other.cost_to_go

# Defining action set, and calculating costs
def move_ext_up(x,y,theta,step_size, cost):
    theta = theta + 60
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta,cost

def move_ext_down(x,y,theta, step_size, cost):
    theta = theta - 60
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta,cost

def move_up(x,y,theta, step_size, cost):
    theta = theta + 60
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta, cost

def move_down(x,y,theta, step_size, cost):
    theta = theta - 30
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta, cost

def move_straight(x,y,theta, step_size, cost):
    theta = theta + 0
    x = x + (step_size*np.cos(np.radians(theta)))
    y = y + (step_size*np.sin(np.radians(theta)))
    x = round(x)
    y = round(y)
    cost = 1 + cost
    return x,y,theta, cost

# Initialising the action set function
def Action_set(move,x,y,theta,step_size,cost):

    if move == 'extreme_right':
        return move_ext_up(x,y,theta, step_size,cost)
    elif move == 'right':
        return move_up(x,y,theta, step_size,cost)
    elif move == 'straight':
        return move_straight(x,y,theta,step_size,cost)
    elif move == 'left':
        return move_down(x,y,theta,step_size,cost)
    elif move == 'extreme_left':
        return move_ext_down(x,y,theta,step_size,cost)
    else:
        return None

#Initialising the "arena" with obstacle definitions
def obstacle_space(width, height, clearance, robot_radius):

    obs_space_array = np.full((height,width),0)
    
    for y in range(height) :
        for x in range(width):
        
            # Boundry clearance buffer
            if x<clearance+robot_radius or x>width-(clearance+robot_radius) or y<clearance+robot_radius or y>height-(clearance+robot_radius):
                obs_space_array[y,x] = 1  # Fill as obstacle

            # Rightmost U shaped obstace
            if x>900-clearance+robot_radius and x<1100+clearance+robot_radius and y>50-(clearance+robot_radius) and y<125+clearance+robot_radius:
                obs_space_array[y,x] = 1
            if x>1020-(clearance+robot_radius) and x<1100+clearance+robot_radius and y>125-(clearance+robot_radius) and y<375+clearance+robot_radius:
                obs_space_array[y,x] = 1      
            if x>900-(clearance+robot_radius) and x<1100+clearance+robot_radius and y>375-(clearance+robot_radius) and y<450+clearance+robot_radius:
                obs_space_array[y,x] = 1  

            # Left rectangle obstace
            if x>100-(clearance+robot_radius) and x<175+clearance+robot_radius and y>100-(clearance+robot_radius):
                obs_space_array[y,x] = 1

            # Right rectangle obstace
            if x>275-(clearance+robot_radius) and x<350+clearance+robot_radius and y<400+clearance+robot_radius:
                obs_space_array[y,x] = 1

            # Hexagonal obstacle
            # Appropriate clearance buffer has been added to the  
            # incercept values for simpler implementation
            if x<785+(clearance+robot_radius) and x>515-(clearance+robot_radius):
                if 1.73*y+x-(1342.25+2*(clearance+robot_radius))<0:
                    if 1.73*y+x-(823-2*(clearance+robot_radius))>0:
                        if 1.73*y-x-(42+2*(clearance+robot_radius))<0:
                            if 1.73*y-x+477.25+2*(clearance+robot_radius)>0:
                                obs_space_array[y,x] = 1              
    
    return obs_space_array
    
# Generating random key for nodes
def key(node):
    key = 1022*node.x + 111*node.y 
    return key

# Check whether goal is reached or not
def goal_reached(present, goal):
    
    dt = dist((present.x, present.y), (goal.x, goal.y))             

    if dt < 1.5 and present.theta == goal.theta:    # Goal threshold. Given=1.5
        return True
    else:
        return False
    
# Checking for the move validity on boundary conditions
def valid_move(x, y, obs_space_array):

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

# Check valid orientation
def valid_orientation(theta):
    if((theta%30)==0):
        return theta
    else:
        return False


# A* Algorithm
def a_star_algorithm(start,goal,obs_space_array,step_size):                       

    if goal_reached(start, goal):
        return None,1
    goal_node = goal
    start_node = start
    
    possible_moves = ['extreme_right','right', 'straight', 'left', 'extreme_left']   
    nodes_unexplored = {}  # List of all open nodes
    
    start_key = key(start_node) # New key for pointing new node
    nodes_unexplored[(start_key)] = start_node
    
    nodes_explored = {} #List of all closed nodes
    priority_list = []  #List to store all dictionary entries with cost as the sorting variable
    heapq.heappush(priority_list, [start_node.cost, start_node]) # Least cost entries will be prioritized
    
    all_nodes = [] #stores all nodes that have been traversed, for visualization purposes.

    while (len(priority_list) != 0):

        current_node = (heapq.heappop(priority_list))[1]
        all_nodes.append([current_node.x, current_node.y, current_node.theta])          
        current_id = key(current_node)

        # If goal reached
        if goal_reached(current_node, goal_node):
            goal_node.parent_id = current_node.parent_id
            goal_node.cost = current_node.cost
            print("Found the GOAL!")
            return all_nodes,1

        if current_id in nodes_explored:  
            continue
        else:
            nodes_explored[current_id] = current_node
        
        del nodes_unexplored[current_id]

        for move in possible_moves:
            x,y,theta,cost = Action_set(move,current_node.x,current_node.y,current_node.theta, step_size, current_node.cost)  
            
            # Calculate cost to go
            cost_to_go = dist((x, y), (goal.x, goal.y))  

            new_node = Node(x,y,theta, cost,current_node, cost_to_go)   

            new_node_id = key(new_node) 

            if not valid_move(new_node.x, new_node.y, obs_space_array):
                continue
            elif new_node_id in nodes_explored:
                continue

            # If unvisited node
            if new_node_id in nodes_unexplored:
                if new_node.cost < nodes_unexplored[new_node_id].cost: 
                    nodes_unexplored[new_node_id].cost = new_node.cost
                    nodes_unexplored[new_node_id].parent_id = new_node.parent_id
            else:
                nodes_unexplored[new_node_id] = new_node
            
            heapq.heappush(priority_list, [(new_node.cost + new_node.cost_to_go), new_node]) 

    return  all_nodes,0

# Backtracking and getting the "reversed" path
def Backtrack(goal_node):  
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

    # Plots the "arena"
    plt.imshow(obs_space_array, "GnBu")
    ax = plt.gca()
    ax.invert_yaxis() # Y-axis inversion
    
    # plotting all the visited nodes
    for i in range(len(all_nodes)):
        plt.plot(all_nodes[i][0], all_nodes[i][1], "2g-")

    # Plotting the final path once all nodes are plotted
    if ideal_path:  # Checks if 'yes'
        plt.plot(x_path[:frame_count+1], y_path[:frame_count+1], ':r')

    # Save each frame as an image
    plt.savefig(f"frame_{frame_count}.png")
    plt.close()

# Creating video from all the saved frames
def output_video(frame_prefix, output_video_path, frame_rate):
    frames = []
    frame_files = [f for f in os.listdir() if f.startswith(frame_prefix) and f.endswith('.png')]
    frame_files.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))  # Sort files by frame number
    for frame_file in frame_files:
        frames.append(cv2.imread(frame_file))
        os.remove(frame_file)  # Delete the frames after appending it to the video

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    write_video = cv2.VideoWriter(output_video_path, fourcc, frame_rate, (frames[0].shape[1], frames[0].shape[0]))

    # Write frames to video
    for frame in frames:
        write_video.write(frame)

    # Releasing the video writer
    write_video.release()
    cv2.destroyAllWindows()

# Main function
if __name__ == '__main__':
    
    # Input the desired clearance
    clearance = input("Input the obstacle clearance: ")
    clearance = int(clearance)
    
    # Input the desired radius of the robot
    robot_radius = input("Input the Robot Radius: ") 
    robot_radius = int(robot_radius)
    
    # Input the desired step size for A*
    robot_step_size = input("Input the Robot Step size: ")
    robot_step_size = int(robot_step_size)
    
    width = 1200
    height = 500
    obs_space_array = obstacle_space(width, height, clearance, robot_radius)
    cost_to_go = 0 # Cost to Go
    
    # Input the desired starting coordinates
    start_coordinates = input("Enter coordinates for Start Node(space separated): ")
    s_x, s_y = start_coordinates.split()
    s_x = int(s_x)
    s_y = int(s_y)

    # Check if it's a valid move
    if not valid_move(s_x, s_y, obs_space_array):
        print("Start node is out of bounds(or in obstacle zone)")
        exit(-1)

    # Input the desired starting orientation of the robot
    s_theta = input("Enter Orientation of the robot at start node: ")
    s_t = int(s_theta)
    
    # Check if it's a valid orientation
    if not valid_orientation(s_t):
        print("Please enter a multiple of 30")
        exit(-1)
            
    # Input the desired goal coordinates 
    goal_coordinates = input("Enter coordinates for Goal Node: ")
    g_x, g_y = goal_coordinates.split()
    g_x = int(g_x)
    g_y = int(g_y)
    
    ## Check if it's a valid move
    if not valid_move(g_x, g_y, obs_space_array):
        print("Goal node is out of bounds(or in obstacle zone)")
        exit(-1)

    # Input the desired goal orientation of the robot
    g_theta = input("Enter Orientation of the robot at goal node: ")
    g_t = int(g_theta)

    ## Check if it's a valid orientation
    if not valid_orientation(g_t):
        print("Please enter a multiple of 30")
        exit(-1)

    #Timer to calculate the processing time
    timer_start = time.time()
    
    # Initialising the start and goal node 
    start_node = Node(s_x, s_y,s_t, 0.0, -1,cost_to_go)
    goal_node = Node(g_x, g_y,g_t, 0.0, -1, cost_to_go)
    all_nodes,flag = a_star_algorithm(start_node, goal_node, obs_space_array, robot_step_size)
    
    # Calculate the optimal backtracking path from start to goal 
    if (flag)==1:
        x_path,y_path = Backtrack(goal_node)
    else:
        print("No path could be found")
        
    # Plotting the exploration, and the final path
    frame_count = 0  # Initialize frame count
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

    # Create video from saved frames
    output_video("frame", "output_video.mp4", 30)  # Default frame rate=30

    timer_stop = time.time()
    C_time = timer_stop - timer_start
    print("Finished executing in:  ", C_time) 
    
