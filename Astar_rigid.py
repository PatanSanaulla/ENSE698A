import sys
import math
import time
from ScenarioMapInfo import *

# Global Variables
START_POINT = []  # [x, y]
GOAL_POINT = []  # [x, y]
EXPLORED = {}  # x,y,theta and Index
RADIUS = 1 # Radius of bot 0.263 m
STEP_OBJECT_LIST = []
SPEEDS = []
COST_MAP_DICT = {}  # Index and Cost
CLEARANCE = 7


# Definition of Class Step:
class Step:
    # Method to initialize the node with the values/attributes and add the step
    # self: Object of class step
    # parent: Object of class step
    # position: the x,y values of the current step
    # cost: cost of the step to move from the parent to the current position
    def __init__(self, parent, position): #, angle, curveSteps, rpm):

        self.position = position  # [x, y]
        self.parent = parent
        if parent == None:
            self.costToCome = 0.0
        else:
            self.costToCome = parent.costToCome + abs(
                (parent.position[0] - position[0]) ** 2 - (parent.position[1] - position[1]) ** 2) ** 0.5  # cost
        self.cost = self.costToCome + float(
            ((GOAL_POINT[0] - self.position[0]) ** 2 + (GOAL_POINT[1] - self.position[1]) ** 2) ** (
                0.5))  # Euclidean Distance
        print("creating a new step with", position, "and cost", self.cost)
        self.addToGraph()

    def addToGraph(self):
        key = str(self.position[0]) + "," + str(self.position[1])
        value = EXPLORED.get(key)
        if value == None:  # Not Visited
            EXPLORED.update({key: len(STEP_OBJECT_LIST)})
            COST_MAP_DICT.update({len(STEP_OBJECT_LIST): self.cost})
            STEP_OBJECT_LIST.append(self)

    def generateSteps(self):
        X = self.position[0]
        Y = self.position[1]
        for move in [1, 0, -1]:
            for step in [1, 0, -1]:
                newX = X + move
                newY = Y + step

                if newX >= -MAX_X and newX <= MAX_X and newY >= -MAX_Y and newY <= MAX_Y and (
                        isValidStep([newX, newY], RADIUS + CLEARANCE) == True):
                    #plt.plot([xS, newX], [yS, newY], color="blue")
                    print("New x:" + str(newX) + ", New y:" + str(newY))
                    newPosition = [newX, newY]
                    try:
                        if (self.parent.position == newPosition):
                            pass
                        else:
                            # plt.plot([self.position[0], newX], [self.position[0], newY], color="blue")
                            newStep = Step(self, newPosition)
                    except AttributeError:
                        # plt.plot([self.position[0], newX], [self.position[0], newY], color="blue")
                        newStep = Step(self, newPosition)
                else:
                    pass


def backtrack(stepObj):
    pathValues = []
    while stepObj.parent != None:
        pathValues.append([stepObj.position[0], stepObj.position[1]])
        stepObj = stepObj.parent
    pathValues.append([stepObj.position[0], stepObj.position[1]])

    pathValues.reverse()

    print("length of step_object_list", len(STEP_OBJECT_LIST))
    print("length of the pathvalues", len(pathValues))
    print(pathValues)
    showPath(pathValues, EXPLORED)
    return pathValues


def inGoal(position):
    x, y = position[0], position[1]
    if ((x - GOAL_POINT[0]) ** 2 + (y - GOAL_POINT[1]) ** 2 <= (0.1) ** 2):
        return True
    else:
        return False


    # def initiatePlanning(stpt, ftpt, clr):
try:
    #startPoints = input("Enter the Start Points (x,y,theta) position: ")
    START_POINT = [270,10,0]#[int(each) for each in startPoints.split(" ")]
    #goalPoints = input("Enter the Goal Points (x,y) position: ")
    GOAL_POINT = [200,180]#[int(each) for each in goalPoints.split(" ")]

except:
    print("Please enter the proper points: Example: 200 30 30")
    print("Exiting the Algorithm")
    sys.exit(0)

isPossible = 0

if START_POINT[0] >= -MAX_X and START_POINT[0] <= MAX_X and START_POINT[1] >= -MAX_Y and START_POINT[
    1] <= MAX_Y and (
        isValidStep(START_POINT, RADIUS + CLEARANCE) == True):
    isPossible += 1
else:
    print("Invalid Start Point")

if GOAL_POINT[0] >= -MAX_X and GOAL_POINT[0] <= MAX_X and GOAL_POINT[1] >= -MAX_Y and GOAL_POINT[1] <= MAX_Y and (
        isValidStep(GOAL_POINT, RADIUS + CLEARANCE) == True):
    isPossible += 1
else:
    print("Invalid Goal Point")

# To check if both the values are possible to work with in the puzzle
if isPossible == 2:
    root = Step(None, START_POINT[:2])# START_POINT[2], None, None)  # Starting the linked list with start point as the root

    start_time = time.time()
    while True:  # to keep traversing until the goal area is found
        topKey = next(iter(COST_MAP_DICT))
        COST_MAP_DICT.pop(topKey)
        poppedStep = STEP_OBJECT_LIST[topKey]
        if inGoal(poppedStep.position) == True:
            break
        else:
            poppedStep.generateSteps()
            COST_MAP_DICT = {index: totalcost for index, totalcost in
                             sorted(COST_MAP_DICT.items(), key=lambda cost: cost[1])}  # EXPLORED.sort()

    end_time = time.time()

    print("Total Cost to reach the final Point:", poppedStep.costToCome)

    print("total time for A star in seconds: ", end_time - start_time)
    SPEEDS = backtrack(poppedStep)  # To show the backtrack on the graph

else:
    print("Exiting the Algorithm")
    sys.exit(0)
