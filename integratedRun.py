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

def sweepScanAerialAgent():
    while True:
        return True

def astarAlgorithmRun():
    file = open('OBJS_easy.txt', 'r')
    Lines = file.readlines()
    goalPoint = [980,500]
    for line in Lines:
        coordinates = [float(x) for x in line.split(" ")]
        goalPoint[0] = math.ceil(500 - (coordinates[1]*(1000/100)))
        goalPoint[1] = math.ceil(500 + (coordinates[0]*(1000/100)))
        astarPlanner = plnr.Planner([980,500], goalPoint, "obs_map_easy.png")
        path = astarPlanner.initiatePlanning()
        del astarPlanner


vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

if clientID != -1:
    print('Connection Established to remote API server')
    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Conneted to Python Code base',vrep.simx_opmode_oneshot) #This message should be printed on your CopelliaSim in the bottm


    #main code to execute
    #Threaded Function 1 to scan the environment with aerial agent
    thread1 = Thread(target=sweepScanAerialAgent)
    thread1.start()

    thread2 = Thread(target=astarAlgorithmRun)
    thread2.start()

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    #vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    #vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection failed")
print ('Program ended')


