try:
    import vrep
    import planner as plnr
    import sys
    from threading import Thread
    import time
    import math
    from time import sleep

except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

GPS_Target = []
GPS_GroundAgent = []
ALL_PATHS = dict()
TARGET_POINTS = []

def getTargetPosition():
    while True:
        trgt = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking )
        GPS_Target = [math.ceil(500 - (trgt[1][1]*(1000/100))), math.ceil(500 + (trgt[1][0]*(1000/100)))]

def getGroundAgentPosition():
    while True:
        gndAgt = vrep.simxGetObjectPosition(clientID, groundAgent, -1, vrep.simx_opmode_blocking )
        GPS_GroundAgent = [math.ceil(500 - (gndAgt[1][1]*(1000/100))), math.ceil(500 + (gndAgt[1][0]*(1000/100)))]

def astarAlgorithmRun():
    file = open('OBJS_easy.txt', 'r')
    Lines = file.readlines()
    #goalPoint = [980,500]
    for line in Lines:
        coordinates = [float(x) for x in line.split(" ")]
        goalPoint = [math.ceil(500 - (coordinates[1]*(1000/100))), math.ceil(500 + (coordinates[0]*(1000/100)))]
        astarPlanner = plnr.Planner(GPS_GroundAgent, goalPoint, "obs_map_easy.png")
        path = astarPlanner.initiatePlanning()
        ALL_PATHS[line] = path
        del astarPlanner

    minPath = min(ALL_PATHS.keys(), key=(lambda k: len(ALL_PATHS[k])))
    TARGET_POINTS = ALL_PATHS[minPath]
    print(minPath)


vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

if clientID != -1:
    print('Connection Established to remote API server')
    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Conneted to Python Code base',vrep.simx_opmode_oneshot)
    #This message should be printed on your CopelliaSim in the bottm

    returnCode, target = vrep.simxGetObjectHandle(clientID, 'GV_target', vrep.simx_opmode_oneshot_wait)

    returnCode, groundAgent = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_oneshot_wait)

    gndAgt = vrep.simxGetObjectPosition(clientID, groundAgent, -1, vrep.simx_opmode_blocking)
    GPS_GroundAgent = [math.ceil(500 - (gndAgt[1][1] * (1000 / 100))), math.ceil(500 + (gndAgt[1][0] * (1000 / 100)))]

    trgt = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
    GPS_Target = [math.ceil(500 - (trgt[1][1] * (1000 / 100))), math.ceil(500 + (trgt[1][0] * (1000 / 100)))]

    #Threaded Function to get the GPS value of the target position
    thread1 = Thread(target=getTargetPosition)
    thread1.start()
    #Threaded Function to get the GPS value of the Ground agent position
    thread2 = Thread(target=getGroundAgentPosition)
    thread2.start()

    file = open('OBJS_easy.txt', 'r')
    Lines = file.readlines()
    # goalPoint = [980,500]
    for line in Lines:
        coordinates = [float(x) for x in line.split(" ")]
        goalPoint = [math.ceil(500 - (coordinates[1] * (1000 / 100))), math.ceil(500 + (coordinates[0] * (1000 / 100)))]
        astarPlanner = plnr.Planner(GPS_GroundAgent, goalPoint, "obs_map_easy.png")
        path = astarPlanner.initiatePlanning()
        ALL_PATHS[line] = path
        del astarPlanner

    minPath = min(ALL_PATHS.keys(), key=(lambda k: len(ALL_PATHS[k])))
    TARGET_POINTS = ALL_PATHS[minPath]
    print(minPath)

    for pos in TARGET_POINTS:
        if GPS_Target ==  pos:
            continue
        else:
            x = (pos[1]-500)*(100/1000)
            y = (-pos[0]+500)*(100/1000)
            z = 0
            vrep.simxSetObjectPosition(clientID, target, -1, [x, y, z], vrep.simx_opmode_blocking)
            while True:
                if vrep.simxGetObjectPosition(clientID, groundAgent, -1, vrep.simx_opmode_blocking ) == vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking ):
                    break

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    #vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    #vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection failed")
print ('Program ended')


