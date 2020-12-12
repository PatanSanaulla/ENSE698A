# Definition of Class Step:
class Step:
    # Method to initialize the node with the values/attributes and add the step
    # self: Object of class step
    # parent: Object of class step
    # position: the x,y values of the current step
    # cost: cost of the step to move from the parent to the current position
    def __init__(self, parent, position, planner): #, angle, curveSteps, rpm):

        self.position = position  # [x, y]
        self.parent = parent
        self.planner = planner
        if parent == None:
            self.costToCome = 0.0
        else:
            self.costToCome = parent.costToCome + abs(
                (parent.position[0] - position[0]) ** 2 - (parent.position[1] - position[1]) ** 2) ** 0.5  # cost
        self.cost = self.costToCome + float(
            ((planner.GOAL_POINT[0] - self.position[0]) ** 2 + (planner.GOAL_POINT[1] - self.position[1]) ** 2) ** (
                0.5))  # Euclidean Distance
        #print("creating a new step with", position, "and cost", self.cost)
        self.addToGraph()

    def addToGraph(self):
        key = str(self.position[0]) + "," + str(self.position[1])
        value = self.planner.EXPLORED.get(key)
        if value == None:  # Not Visited
            self.planner.EXPLORED.update({key: len(self.planner.STEP_OBJECT_LIST)})
            self.planner.COST_MAP_DICT.update({len(self.planner.STEP_OBJECT_LIST): self.cost})
            self.planner.STEP_OBJECT_LIST.append(self)

    def generateSteps(self):
        X = self.position[0]
        Y = self.position[1]
        for move in [1, 0, -1]:
            for step in [1, 0, -1]:
                newX = X + move
                newY = Y + step

                if newX >= -self.planner.MAX_X and newX <= self.planner.MAX_X and newY >= -self.planner.MAX_Y and newY <= self.planner.MAX_Y and (
                        self.planner.isValidStep([newX, newY], self.planner.RADIUS + self.planner.CLEARANCE) == True):
                    #print("New x:" + str(newX) + ", New y:" + str(newY))
                    newPosition = [newX, newY]
                    try:
                        if (self.parent.position == newPosition):
                            pass
                        else:
                            # plt.plot([self.position[0], newX], [self.position[0], newY], color="blue")
                            newStep = Step(self, newPosition, self.planner)
                    except AttributeError:
                        # plt.plot([self.position[0], newX], [self.position[0], newY], color="blue")
                        newStep = Step(self, newPosition, self.planner)
                else:
                    pass
