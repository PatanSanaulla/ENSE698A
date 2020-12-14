# Make sure to have the server side running in Coppelia:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import vrep
    import sys
    # from threading import Thread
    import time
    from time import sleep
    import numpy as np
    import cv2

except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

print('Program started to execute in V-Rep')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP

if clientID != -1:
    print('Connection Established to remote API server')

    print('Connected to remote API server')

    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID, 'Object Detection Start',
                                 vrep.simx_opmode_oneshot)  # This message should be printed on your CopelliaSim in
    # the bottm

    # Get Object Handles
    (err, ground_handle) = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_oneshot)
        (err, cam_handle) = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
    (err, target_handle) = vrep.simxGetObjectHandle(clientID, 'GV_target', vrep.simx_opmode_oneshot)

    (err, pos_tar) = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot)
    (err, pos_ground) = vrep.simxGetObjectPosition(clientID, ground_handle, -1, vrep.simx_opmode_oneshot)
    (err, linvel, angvel) = vrep.simxGetObjectVelocity(clientID, ground_handle, -1, vrep.simx_opmode_oneshot)
    (err, orientation) = vrep.simxGetObjectOrientation(clientID, ground_handle, -1, vrep.simx_opmode_oneshot)

    # Get Detected image
    (err, res, img) = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_streaming)
    time.sleep(0.1)
    (err, res, img) = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_buffer)

    # Format image
    sensorImage = []
    sensorImage = np.array(img, dtype=np.uint8)

    if (len(sensorImage) != 0):
        sensorImage = np.reshape(sensorImage, np.append(res, 3))
        sensorImage = cv2.resize(sensorImage, (1000, 1000), interpolation=cv2.INTER_AREA)
        sensorImage = cv2.flip(sensorImage, 0)  # vertical flip

        edges = cv2.Canny(sensorImage, 100, 200)

        while vrep.simxGetConnectionId(clientID) != -1:
            (err, res, img) = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_buffer)
            sensorImage = np.array(img, dtype=np.uint8)
            if (len(sensorImage) != 0):
                sensorImage = np.reshape(sensorImage, np.append(res, 3))
                sensorImage = cv2.resize(sensorImage, (1000, 1000), interpolation=cv2.INTER_AREA)
                sensorImage = cv2.flip(sensorImage, 0)  # vertical flip
                edges = cv2.Canny(sensorImage, 100, 200)
                cv2.imshow('Fig', edges)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                elif err == vrep.simx_return_novalue_flag:
                    print("no image yet")
                    pass
                else:
                    print(err)

        cv2.waitKey(0)  # waits until a key is pressed
        cv2.destroyAllWindows()

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can
    # guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
    sys.exit("Connection failed")
print('Program ended')
