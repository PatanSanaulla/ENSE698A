import cv2 as cv
import sys
import astar
import time

# Definition of Class Planner:
class Planner:
    # Class to initiate the planner and to store the values of the output path
    # self: Object of class planner
    def __init__(self, strPoint, endPoint, mapName):
        self.START_POINT = strPoint #[x, y]
        self.GOAL_POINT = endPoint #[x, y]
        self.EXPLORED = {}  # x,y,theta and Index
        self.RADIUS = 1  # Radius of bot 0.263 m
        self.STEP_OBJECT_LIST = []
        self.COST_MAP_DICT = {}  # Index and Cost
        self.CLEARANCE = 7

        self.obstacle_map = cv.imread(mapName) # "obs_map_easy.png"
        self.gray_map = cv.cvtColor(self.obstacle_map, cv.COLOR_BGR2GRAY)
        x, y, _ = self.obstacle_map.shape
        self.MAX_X = x
        self.MAX_Y = y

    def initiatePlanning(self):
        isPossible = 0
        if self.START_POINT[0] >= -self.MAX_X and self.START_POINT[0] <= self.MAX_X and self.START_POINT[1] >= -self.MAX_Y and self.START_POINT[
            1] <= self.MAX_Y and (
                self.isValidStep(self.START_POINT, self.RADIUS + self.CLEARANCE) == True):
            isPossible += 1
        else:
            print("Invalid Start Point")

        if self.GOAL_POINT[0] >= -self.MAX_X and self.GOAL_POINT[0] <= self.MAX_X and self.GOAL_POINT[1] >= -self.MAX_Y and self.GOAL_POINT[1] <= self.MAX_Y and (
                self.isValidStep(self.GOAL_POINT, self.RADIUS + self.CLEARANCE) == True):
            isPossible += 1
        else:
            print("Invalid Goal Point")

        # To check if both the values are possible to work with in the puzzle
        if isPossible == 2:
            root = astar.Step(None, self.START_POINT, self)  # START_POINT[2], None, None)  # Starting the linked list with start point as the root

            start_time = time.time()
            while True:  # to keep traversing until the goal area is found
                topKey = next(iter(self.COST_MAP_DICT))
                self.COST_MAP_DICT.pop(topKey)
                poppedStep = self.STEP_OBJECT_LIST[topKey]
                if self.inGoal(poppedStep.position) == True:
                    break
                else:
                    poppedStep.generateSteps()
                    self.COST_MAP_DICT = {index: totalcost for index, totalcost in
                                     sorted(self.COST_MAP_DICT.items(), key=lambda cost: cost[1])}  # EXPLORED.sort()

            end_time = time.time()

            #print("Total Cost to reach the final Point:", poppedStep.costToCome)

            #print("total time for A star in seconds: ", end_time - start_time)
            return(self.backtrack(poppedStep)) # To show the backtrack on the graph

        else:
            print("Exiting the Algorithm")
            sys.exit(0)

    def isValidStep(self, position, clearance):
        posX = position[0]
        posY = position[1]
        i = 0
        while (i <= clearance):
            if self.gray_map[posX][posY] > 127 or self.gray_map[posX + i][posY] > 127 or self.gray_map[posX][posY + i] > 127 or self.gray_map[posX + i][
                posY + i] > 127:
                return False
            i = i + 1
        return True

    def showPath(self, pathValues, Explored):
        for exp in Explored.keys():
            pos = exp.split(',')
            cv.circle(self.obstacle_map, (int(pos[1]), int(pos[0])), 1, (255, 255, 0), 1)

        for pathpos in pathValues:
            cv.circle(self.obstacle_map, (int(pathpos[1]), int(pathpos[0])), 1, (0, 0, 255), 1)

        cv.imshow("Map", self.obstacle_map)
        while (1):
            key = cv.waitKey(100) & 0xff

    def backtrack(self, stepObj):
        pathValues = []
        while stepObj.parent != None:
            pathValues.append([stepObj.position[0], stepObj.position[1]])
            stepObj = stepObj.parent
        pathValues.append([stepObj.position[0], stepObj.position[1]])

        pathValues.reverse()

        #print("length of step_object_list", len(self.STEP_OBJECT_LIST))
        #print("length of the pathvalues", len(pathValues))
        #print(pathValues)
        #self.showPath(pathValues, self.EXPLORED)
        return pathValues

    def inGoal(self, position):
        x, y = position[0], position[1]
        if ((x - self.GOAL_POINT[0]) ** 2 + (y - self.GOAL_POINT[1]) ** 2 <= (0.1) ** 2):
            return True
        else:
            return False