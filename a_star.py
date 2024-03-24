import numpy as np
import cv2 as cv
from queue import PriorityQueue
import math

boundry = []    
Pth = {}        #Stores the path for backtracking
UncheckedList = PriorityQueue()     #Used to store unvisited nodes
b_track = []            
CloseList = []
CheckedList = np.zeros((500,1200),dtype='uint8')     # Used to store the visited nodes
#Creating the Obstacle Space
def obstacle_space(s):
    h,w,_ = s.shape
    for y in range(h):
        for x in range(w):
            if ((500-y) - 5 < 0) or ((x) - 5 < 0) or ((500-y) - 495 > 0) or ((x) - 1195 > 0): #boundary
                s[y][x] = [0,255,0]
                boundry.append((x,500-y))
            if (x > 95) and (x < 180) and (500-y > 95) and (500-y <498):
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if (x > 100) and (x < 175) and (500-y > 100) and (500-y < 498):   #rectangle1
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if(x > 270) and (x < 355) and (500-y >2) and (500-y < 405):
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if (x > 275) and (x < 350) and (500-y >2) and (500-y < 400):  #rectangle2
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if ((x-570)>=0) and (((500-y)-(1.623*x)+595.11)<=0) and (((500-y)+(1.623*x)-1514.85)<=0) and ((x-730)<=0) and (((500-y)-(1.623*x)+1014.85)>=0) and (((500-y)+(1.623*x)-1095.11)>=0):
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if ((x-575)>=0) and ((x-725) <= 0) and (((500-y)-(1.732*x)+670.9) <= 0) and (((500-y)+(1.732*x)-1580.7)<=0) and ((x-725)<=0) and (((500-y)-(1.732*x)+1080.7)>=0) and (((500-y)+(1.732*x)-1170.9)>=0):   #hexagon
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if ((x-900)>=0) and ((x-1025)<=0) and (((((500-y)-455)<=0) and (((500-y)-370)>=0)) or ((((500-y)-45)>=0) and (((500-y)-130)<=0))):
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if ((x-1020)>=0) and ((x-1105)<=0) and (((500-y)-45)>=0) and (((500-y)-455)<=0):
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if ((x-905)>=0) and ((x-1025)<=0) and (((((500-y)-450)<=0) and (((500-y)-375)>=0)) or ((((500-y)-50)>=0) and (((500-y)-125)<=0))):  # C shape
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
            if ((x-1025)>=0) and ((x-1100)<=0) and (((500-y)-50)>=0) and (((500-y)-450)<=0):
                s[y][x] = [128,128,144]
                boundry.append((x,500-y))
    return boundry

