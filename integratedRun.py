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
        try:
            trgt = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking )
            GPS_Target = convertToPxlCoord(trgt[1])
        except ValueError:
            continue


def getGroundAgentPosition():
    while True:
        try:
            gndAgt = vrep.simxGetObjectPosition(clientID, groundAgent, -1, vrep.simx_opmode_blocking )
            GPS_GroundAgent = convertToPxlCoord(gndAgt[1])
        except ValueError:
            continue

def compareTargetAndGA():
    print("current GroundAgent at: "+str(GPS_GroundAgent)+" and Target at: "+str(GPS_Target))
    if GPS_GroundAgent == GPS_Target or GPS_GroundAgent == [GPS_Target[0], GPS_Target[1]+1] or GPS_GroundAgent == [GPS_Target[0]+1, GPS_Target[1]]:
        return True
    else:
        return False

def convertToPxlCoord(vrepCoord):
    return [math.ceil(500 - (vrepCoord[1]*(1000/100))), math.ceil(500 + (vrepCoord[0]*(1000/100)))]


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
    GPS_GroundAgent = convertToPxlCoord(gndAgt[1])

    trgt = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)
    GPS_Target = convertToPxlCoord(trgt[1])

    #Threaded Function to get the GPS value of the target position
    thread1 = Thread(target=getTargetPosition)
    thread1.start()
    #Threaded Function to get the GPS value of the Ground agent position
    thread2 = Thread(target=getGroundAgentPosition)
    thread2.start()

    file = open('OBJS_easy.txt', 'r')
    Lines = file.readlines()
    for line in Lines:
        coordinates = [float(x) for x in line.split(" ")]
        goalPoint = convertToPxlCoord(coordinates)
        astarPlanner = plnr.Planner(GPS_GroundAgent, goalPoint, "obs_map_easy.png")
        path = astarPlanner.initiatePlanning()
        ALL_PATHS[line] = path
        del astarPlanner

    minPath = min(ALL_PATHS.keys(), key=(lambda k: len(ALL_PATHS[k])))
    TARGET_POINTS = ALL_PATHS[minPath]
    print("Found the Closest object")
    print(minPath)
    print(TARGET_POINTS)

    for pos in TARGET_POINTS:
        if GPS_Target ==  pos:
            continue
        else:
            x = (pos[1]-500)*(100/1000)
            y = (-pos[0]+500)*(100/1000)
            z = 0
            vrep.simxSetObjectPosition(clientID, target, -1, [x, y, z], vrep.simx_opmode_blocking)
            #sleep(0.5)
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


