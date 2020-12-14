try:
    import vrep
    import sys
    # from threading import Thread
    import time
    from time import sleep
    import math
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

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP

if clientID != -1:
    print('Connection Established to remote API server')

    # Object Handles
    (err, ground_handle) = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_oneshot)
    (err, cam_handle) = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
    (err, target_handle) = vrep.simxGetObjectHandle(clientID, 'GV_target', vrep.simx_opmode_oneshot)

    # Get Detected image
    (err, res, img) = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_streaming)
    time.sleep(0.5)
    (err, res, img) = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_buffer)

    # Get target position
    (err, pos_tar) = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_streaming)
    (err, tar_or) = vrep.simxGetObjectOrientation(clientID, target_handle, -1, vrep.simx_opmode_streaming)

    print(pos_tar, tar_or)

    # Format image
    sensorImage = []
    sensorImage = np.array(img, dtype=np.uint8)

    # backSub = cv2.createBackgroundSubtractorMOG2()

    if len(sensorImage) != 0:
        sensorImage = np.reshape(sensorImage, np.append(res, 3))
        sensorImage = cv2.resize(sensorImage, (1000, 1000), interpolation=cv2.INTER_AREA)
        sensorImage = cv2.flip(sensorImage, 0)  # vertical flip
        # sensorImage = backSub.apply(sensorImage)
        # sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_RGB2HSV)
        sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_RGB2GRAY)  # gray scale
        sensorImage = cv2.GaussianBlur(sensorImage, (5, 5), 0)
        # ret, binImg = cv2.threshold(sensorImage, 145, 255, 0)  # Binary image
        # contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print(contours)
        edges = cv2.Canny(sensorImage, 100, 200)
        ret, binImg = cv2.threshold(edges, 0, 255, 0)
        contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            M = cv2.moments(c)
            if M["m00"] != 0 and M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(edges, (cX, cY), 5, (255, 255, 255), -1)
            else:
                break
                # if cX < 500 or cX > 525:
                # gamma = tar_or[2] + 0.5
                # err = vrep.simxSetObjectOrientation(clientID, target_handle, -1, [tar_or[0], tar_or[1], gamma],
                #                               vrep.simx_opmode_oneshot)

            # print(cX, cY)

    while vrep.simxGetConnectionId(clientID) != -1:
        (err, res, img) = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_buffer)
        sensorImage = np.array(img, dtype=np.uint8)
        if len(sensorImage) != 0:
            sensorImage = np.reshape(sensorImage, np.append(res, 3))
            sensorImage = cv2.resize(sensorImage, (1000, 1000), interpolation=cv2.INTER_AREA)
            sensorImage = cv2.flip(sensorImage, 0)  # vertical flip
            sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_RGB2GRAY)  # gray scale
            sensorImage = cv2.GaussianBlur(sensorImage, (5, 5), 0)
            edges = cv2.Canny(sensorImage, 100, 200)

            ret, binImg = cv2.threshold(edges, 0, 255, 0)
            contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cX_avg = 0
            cY_avg = 0
            for c in contours:
                time.sleep(0.5)
                M = cv2.moments(c)
                if M["m00"] != 0 and M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(edges, (cX, cY), 5, (255, 255, 255), -1)
                    cX_avg = cX_avg + cX
                    cY_avg = cY_avg + cY

                else:
                    break
            cX_avg = int(cX_avg / (len(contours) - 1))
            cY_avg = int(cY_avg / (len(contours) - 1))
            print(cX_avg, cY_avg)
            if cX_avg < 500 or cY_avg > 525:
                x = pos_tar[0] - (cX_avg - 512)*(100/1000)
                y = pos_tar[1]
                z = pos_tar[2]
                gamma = math.atan2(y, x)
                vrep.simxSetObjectOrientation(clientID, target_handle, -1,
                                              [x,y,z],
                                              vrep.simx_opmode_oneshot)
                #vrep.simxSetObjectOrientation(clientID, target_handle, -1,[tar_or[0],tar_or[1],gamma],vrep.simx_opmode_oneshot)
                (err, tar_or) = vrep.simxGetObjectOrientation(clientID, target_handle, -1,
                                                              vrep.simx_opmode_buffer)
                time.sleep(0.5)

                print(tar_or)
            else:
                break

            cv2.imshow('Fig', edges)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            elif err == vrep.simx_return_novalue_flag:
                print("no image yet")
                pass

    cv2.waitKey(0)  # waits until a key is pressed
    cv2.destroyAllWindows()
    vrep.simxFinish(clientID)
    vrep.simxFinish(-1)

else:
    print('Failed connecting to remote API server')
    sys.exit("Connection failed")
