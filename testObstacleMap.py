import imutils
import matplotlib.pyplot as plt

import cv2 as cv

import numpy as np
def isValidStep(position, clearance):
    posX = position[0]
    posY = position[1]
    i = 0
    while(i <= clearance):
        if gray[posX][posY] > 127 or gray[posX+i][posY] > 127 or gray[posX][posY+i] > 127 or gray[posX+i][posY+i] > 127:
            return False
        i = i+1

    #if gray[int(position[0])][int(position[1])] == 0 | gray[int(position[0]+clearnace)][int(position[1])] == 0 | \
     #       gray[int(position[0]-clearnace)][int(position[1])] == 0 | gray[int(position[0])][int(position[1]+clearnace)] == 0 |\
      #          gray[int(position[0])][int(position[1]-clearnace)] == 0:
    return True



shared_map = cv.imread("mapping.png")
#shared_map = cv.flip(shared_map, 0)
edited_map = cv.imread("mapping.png")
gray = cv.cvtColor(shared_map, cv.COLOR_BGR2GRAY)
x, y,_  = shared_map.shape
MAX_X = x
MAX_Y = y



for i in range(0,MAX_X):
    for j in range(0, MAX_Y):
        if isValidStep([i,j],0) == True:
            cv.circle(edited_map, (j,i), 1, (0, 0, 255), 1)
cv.circle(edited_map, (10,270), 1, (255, 0, 0), 7)
cv.circle(edited_map, (180,200), 1, (255, 0, 0), 7)
cv.imshow("Map1", shared_map)
cv.imshow("Map2", edited_map)
while (1):
    key = cv.waitKey(10000) & 0xff