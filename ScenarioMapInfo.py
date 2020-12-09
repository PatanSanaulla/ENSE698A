import imutils
import matplotlib.pyplot as plt

import cv2 as cv

# import matplotlib.lines as mlines
#plt.style.use('seaborn-pastel')
import numpy as np


MAX_X = 1000
MAX_Y = 1000

shared_map = cv.imread("mapping.png")
#shared_map = cv.flip(shared_map, 0)
gray = cv.cvtColor(shared_map, cv.COLOR_BGR2GRAY)
x, y,_  = shared_map.shape
MAX_X = x
MAX_Y = y

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


def showPath(pathValues, Explored):

    for exp in Explored.keys():
        pos = exp.split(',')
        cv.circle(shared_map, (int(pos[1]), int(pos[0])), 1, (255, 255, 0), 1)

    for pathpos in pathValues:
        cv.circle(shared_map, (int(pathpos[1]), int(pathpos[0])), 1, (0, 0, 255), 1)

    cv.imshow("Map", shared_map)
    while (1):
        key = cv.waitKey(10000) & 0xff
#     count = 0
#     circle1 = plt.Circle((plotCircle1[1]), plotCircle1[0], fc=None)
#     circle2 = plt.Circle((plotCircle2[1]), plotCircle2[0], fc=None)
#     circle3 = plt.Circle((plotCircle3[1]), plotCircle3[0], fc=None)
#     circle4 = plt.Circle((plotCircle4[1]), plotCircle4[0], fc=None)
#     square1 = plt.Polygon(plotSquare1)
#     square2 = plt.Polygon(plotSquare2)
#     square3 = plt.Polygon(plotSquare3)
#     borderWall = plt.Polygon(plotBorderWall, fill=None)
#     obstacles = [circle1, circle2, circle3, circle4, square1, square2, square3]
#
#     for item in obstacles:
#         axis.add_patch(item)
#         axis.add_patch(borderWall)
#
#     # xTracepoint1 = []
#     # yTracepoint2 = []
#     #
#     # yTracepoint1 = []
#     # xTracepoint2 = []
#     #
#     # xTrackpoint1 = []
#     # yTrackpoint1 = []
#     #
#     # xTrackpoint2 = []
#     # yTrackpoint2 = []
#     #
#     # imageList = []
#     #
#     # framerate = 30
#     # for itr in range(1, len(STEP_OBJECT_LIST)):
#     #     try:
#     #         startTrace = STEP_OBJECT_LIST[itr * framerate].parent
#     #         xTracepoint1 = startTrace.position[0]
#     #         yTracepoint1 = startTrace.position[1]
#     #         xTracepoint2 = STEP_OBJECT_LIST[itr * framerate].position[0] - startTrace.position[0]
#     #         yTracepoint2 = STEP_OBJECT_LIST[itr * framerate].position[1] - startTrace.position[1]
#     #         axis.quiver(xTracepoint1, yTracepoint1, xTracepoint2, yTracepoint2, units='xy', scale=1, color='blue')
#     #         plt.savefig("./images/frame" + str(count) + ".png", dpi = 500, quality = 80)
#     #         imageList.append("images/frame" + str(count) + ".png")
#     #         # print(len(STEP_OBJECT_LIST))
#     #         print("count:", count)
#     #         count = count + 1
#     #     except:
#     #         break
#     #
#     # for itr in range(1, len(pathValues)):
#     #     try:
#     #         xTrackpoint1 = pathValues[itr][0]
#     #         yTrackpoint1 = pathValues[itr][1]
#     #         xTrackpoint2 = pathValues[itr + 1][0] - pathValues[itr][0]
#     #         yTrackpoint2 = pathValues[itr + 1][1] - pathValues[itr][1]
#     #         axis.quiver(xTrackpoint1, yTrackpoint1, xTrackpoint2, yTrackpoint2, units='xy', scale=1, color='red')
#     #         plt.savefig("./images/frame" + str(count) + ".png", dpi = 500)
#     #         imageList.append("images/frame" + str(count) + ".png")
#     #         print("count:", count)
#     #         count = count + 1
#     #     except:
#     #         break
#     #
#     # output = cv2.VideoWriter("Simulation_Video.avi", cv2.VideoWriter_fourcc(*'XVID'), 20.0, (1280, 720))
#     # for image in imageList:
#     #     display = cv2.imread(image)
#     #     display = cv2.resize(display, (1280, 720))
#     #     output.write(display)
#     # output.release()
#
#     plt.show()

