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
OBJECTS_LIST = ["-4.301942795144567100e+01 1.668216601234799512e+01", "-3.903965686868058071e+01 3.878300354844295139e+00", "-3.596882883644629914e+01 -1.925359988963654345e+01",
                "-2.779374694923166089e+01 -6.786084238198500707e+00", "-3.571816138295204723e+01 3.494265815045015700e+01", "-3.274305282709623555e+01 3.527596906749624139e+01",
                "-3.273008800231556847e+01 3.526363670854166088e+01", "-2.280438795227334836e+01 3.061318893123140938e+01", "-1.766011883552472739e+01 1.485091621136611373e+01",
                "-1.745021714791523948e+01 1.572756443608808574e+01", "-1.762746734797226367e+01 1.500315160025669137e+01", "-1.365122833879108200e+01 3.194807128906250071e+00",
                "-2.128056850333196692e+01 3.194807128906250071e+00", "-1.416444155733705657e+01 -3.831263618950078165e+00", "-1.416795748296407353e+01 -3.996696249250648592e+00",
                "-8.889802254166550455e+00 -1.646985904062735173e+01", "-8.863012412951995600e+00 -1.660381190694694098e+01", "-7.915420401983931598e+00 3.168938626365390832e+01",
                "7.011463087285690676e+00 2.103207979694845520e+01", "2.388495074492705594e+00 3.929433142321048233e+00", "1.299595617362090572e+01 -2.783283076362472030e+01",
                "3.085184920661876973e+01 -3.675865612396758131e-01", "3.019226145403777650e+01 -2.287676242565955675e+01", "3.020265316122621613e+01 -2.287676800407635014e+01"]
COLLECTED_OBJECTS = []
CURRENT_OBJECT_LOCATION = []
AGENT_ORIENTED = False

def getTargetPosition():
    global GPS_Target
    while True:
        try:
            trgt = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking )
            GPS_Target = convertToPxlCoord(trgt[1])
            sleep(1)
        except ValueError:
            continue


def getGroundAgentPosition():
    global GPS_GroundAgent
    while True:
        try:
            gndAgt = vrep.simxGetObjectPosition(clientID, groundAgent, -1, vrep.simx_opmode_blocking )
            GPS_GroundAgent = convertToPxlCoord(gndAgt[1])
            sleep(1)
        except ValueError:
            continue

def compareTargetAndGA():
    global AGENT_ORIENTED
    Tx, Ty, Tz = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking )[1]
    Gx, Gy, Gz = vrep.simxGetObjectPosition(clientID, groundAgent, -1, vrep.simx_opmode_blocking )[1]
    Ox, Oy = CURRENT_OBJECT_LOCATION
    if AGENT_ORIENTED == False and ( ((Ox - Tx) ** 2 + (Oy - Ty) ** 2 <= (19) ** 2)) :
        orientGroundAgent()
        Tx, Ty, Tz = vrep.simxGetObjectPosition(clientID, target, -1, vrep.simx_opmode_blocking)[1]
        Gx, Gy, Gz = vrep.simxGetObjectPosition(clientID, groundAgent, -1, vrep.simx_opmode_blocking)[1]
        AGENT_ORIENTED = True
    if ((Gx - Tx) ** 2 + (Gy - Ty) ** 2 <= (0.5) ** 2): #radius of 10Cms
        return True
    else:
        return False

def orientGroundAgent():
    import perception
    perception.fetchVSDataAndOrient(clientID)

def convertToPxlCoord(vrepCoord):
    return [math.ceil(500 - (vrepCoord[1]*(1000/100))), math.ceil(500 + (vrepCoord[0]*(1000/100)))]


vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

if clientID != -1:
    print('Connection Established to remote API server')
    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Connected to Python Code base',vrep.simx_opmode_oneshot)
    #This message should be printed on your CopelliaSim in the bottm
    start_time = time.time() #Start of the whole code

    #getting the CoppeliaSm Handles
    returnCode, target = vrep.simxGetObjectHandle(clientID, 'GV_target', vrep.simx_opmode_oneshot_wait)
    returnCode, groundAgent = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_oneshot_wait)
    err, cam_handle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
    #err, prox_sensor = vrep.simxGetObjectHandle(clientID, 'BaxterGripper_attachProxSensor', vrep.simx_opmode_blocking)

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

    #file = open('OBJS_easy.txt', 'r')
    #Lines = file.readlines()
    while len(OBJECTS_LIST) != len(COLLECTED_OBJECTS):

        for object in OBJECTS_LIST:
            vrep.simxAddStatusbarMessage(clientID, 'Planning ... ', vrep.simx_opmode_oneshot)
        #for line in Lines:
            startPoint = GPS_GroundAgent
            if object in COLLECTED_OBJECTS:
                continue
            else:
                coordinates = [float(x) for x in object.split(" ")]
                goalPoint = convertToPxlCoord(coordinates)
                astarPlanner = plnr.Planner(startPoint, goalPoint, "obs_map_easy.png")
                path = astarPlanner.initiatePlanning()
                if len(path) == 0:
                    break
                ALL_PATHS[object] = path
                del astarPlanner

        closestObject = min(ALL_PATHS.keys(), key=(lambda k: len(ALL_PATHS[k])))
        TARGET_POINTS = ALL_PATHS[closestObject]
        CURRENT_OBJECT_LOCATION = [float(x) for x in object.split(" ")]
        AGENT_ORIENTED = False

        ALL_PATHS = dict()#clear all the path values for the next iteration
        vrep.simxAddStatusbarMessage(clientID,'Found the Closest object',vrep.simx_opmode_oneshot)
        print(closestObject)
        print(TARGET_POINTS)
        start_moving_time = time.time()  # Start of the whole code
        for i in range(0, len(TARGET_POINTS), 1):
            if GPS_Target ==  TARGET_POINTS[i]:
                continue
            else:
                x = (TARGET_POINTS[i][1]-500)*(100/1000)
                y = (-TARGET_POINTS[i][0]+500)*(100/1000)
                z = 0
                vrep.simxSetObjectPosition(clientID, target, -1, [x, y, z], vrep.simx_opmode_blocking)
                while True:
                    if compareTargetAndGA() == True:
                        break
        COLLECTED_OBJECTS.append(closestObject)
        vrep.simxAddStatusbarMessage(clientID,'Reached the object '+str(len(COLLECTED_OBJECTS))+'!!',vrep.simx_opmode_oneshot)
        end_time = time.time()
        print("Time for Reaching the object: ", end_time - start_moving_time)
    print("Total Time: ", end_time - start_time)
    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    #vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    #vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection failed")
print ('Program ended')


