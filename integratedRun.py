try:
    import vrep
    import planner as plnr
    import sys
    from threading import Thread
    import time
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
    # while True:
    #     print("Running astar Active")
    #points = [[450, 200], ]
    #for i in points:

    astarPlanner = plnr.Planner([980,500], [450,290], "obs_map_easy.png")
    astarPlanner.initiatePlanning()


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


