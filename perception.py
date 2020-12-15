#try:
import vrep
#import sys
# from threading import Thread
import time
#from time import sleep
import math
import numpy as np
import cv2


#vrep.simxFinish(-1)  # just in case, close all opened connections
#clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP


def fetchVSDataAndOrient(clientID):
    # Object Handles
    err, ground_handle = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_oneshot)
    err, cam_handle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
    err, target_handle = vrep.simxGetObjectHandle(clientID, 'GV_target', vrep.simx_opmode_blocking)
    err, prox_sensor = vrep.simxGetObjectHandle(clientID, 'BaxterGripper_attachProxSensor', vrep.simx_opmode_blocking)

    # Get Detected image
    err, res, img = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_streaming)
    time.sleep(0.5)
    err, res, img = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_buffer)

    # Get target position
    err, pos_tar = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_streaming)
    err, tar_or = vrep.simxGetObjectOrientation(clientID, target_handle, -1, vrep.simx_opmode_streaming)

    #Prox sensor:
    ret, det_state, det_Point, det_objHandle, det_n_vec = vrep.simxReadProximitySensor(clientID, prox_sensor, vrep.simx_opmode_streaming)
    #print(pos_tar, tar_or)

    # Format image
    sensorImage = []
    sensorImage = np.array(img, dtype=np.uint8)
    x = 512
    y = 512
    flag = 0

    if len(sensorImage) != 0:
        sensorImage = np.reshape(sensorImage, np.append(res, 3))
        sensorImage = cv2.resize(sensorImage, (1000, 1000), interpolation=cv2.INTER_AREA)
        sensorImage = cv2.flip(sensorImage, 0)  # vertical flip
        sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_RGB2GRAY)  # gray scale
        sensorImage = cv2.GaussianBlur(sensorImage, (5, 5), 0)
        edges = cv2.Canny(sensorImage, 100, 200)
        ret, binImg = cv2.threshold(edges, 0, 255, 0)
        trash, contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            for c in contours:
                M = cv2.moments(c)
                if M["m00"] != 0 and M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(edges, (cX, cY), 5, (255, 255, 255), -1)
                    if 500 < cX < 530:
                        break
                    else:
                        alpha = tar_or[0]
                        beta = tar_or[1]
                        gamma = math.atan2(pos_tar[1], pos_tar[0])  # this will change to object coordinates
                        orientation = [alpha, beta, gamma]
                        vrep.simxSetObjectOrientation(clientID, target_handle, -1, orientation,
                                                      vrep.simx_opmode_blocking)
                        break
                else:
                    alpha = tar_or[0]
                    beta = tar_or[1]
                    gamma = math.atan2(pos_tar[1], pos_tar[0])  # this will change to object coordinates
                    orientation = [alpha, beta, gamma]
                    vrep.simxSetObjectOrientation(clientID, target_handle, -1, orientation,
                                                  vrep.simx_opmode_blocking)
                    break

    while vrep.simxGetConnectionId(clientID) != -1:
        err, res, img = vrep.simxGetVisionSensorImage(clientID, cam_handle, 0, vrep.simx_opmode_streaming)
        time.sleep(0.5)
        sensorImage = np.array(img, dtype=np.uint8)
        if len(sensorImage) != 0:
            sensorImage = np.reshape(sensorImage, np.append(res, 3))
            sensorImage = cv2.resize(sensorImage, (1000, 1000), interpolation=cv2.INTER_AREA)
            sensorImage = cv2.flip(sensorImage, 0)  # vertical flip
            sensorImage = cv2.cvtColor(sensorImage, cv2.COLOR_RGB2GRAY)  # gray scale
            sensorImage = cv2.GaussianBlur(sensorImage, (5, 5), 0)
            edges = cv2.Canny(sensorImage, 100, 200)
            ret, binImg = cv2.threshold(edges, 0, 255, 0)
            trash, contours, hierarchy = cv2.findContours(binImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) != 0:
                for c in contours:
                    M = cv2.moments(c)
                    if M["m00"] != 0 and M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        print (cX,cY)
                        cv2.circle(edges, (cX, cY), 5, (255, 255, 255), -1)
                        if 470 < cX < 530:
                            ret, det_state, det_Point, det_objHandle, det_n_vec = vrep.simxReadProximitySensor(clientID,
                                                                                                               prox_sensor,
                                                                                                               vrep.simx_opmode_streaming)
                            ret, pos_tar = vrep.simxGetObjectPosition(clientID,target_handle,-1,vrep.simx_opmode_blocking)
                            ret, tar_or = vrep.simxGetObjectOrientation(clientID,target_handle,-1,vrep.simx_opmode_blocking)
                            x = pos_tar[0]
                            y = pos_tar[1]+1.5
                            z = pos_tar[2]
                            pos = [x,y,z]
                            vrep.simxSetObjectPosition(clientID, target_handle,target_handle,pos,vrep.simx_opmode_blocking)
                            flag = 1
                            break
                        else:
                            if cX>512:
                                x = x + 5
                            else:
                                x = x - 5
                            y = 512
                            alpha = tar_or[0]
                            beta = tar_or[1]
                            gamma = math.atan2(y, x)
                            orientation = [alpha, beta, gamma]
                            vrep.simxSetObjectOrientation(clientID, target_handle, -1, orientation,
                                                          vrep.simx_opmode_blocking)
                            time.sleep(0.5)
                            break
                    else:
                        x = x - 50
                        y = 512
                        alpha = tar_or[0]
                        beta = tar_or[1]
                        gamma = math.atan2(y, x)
                        orientation = [alpha, beta, gamma]
                        vrep.simxSetObjectOrientation(clientID, target_handle, -1, orientation,
                                                      vrep.simx_opmode_blocking)
                        time.sleep(0.5)
                if flag == 1:
                    break
            else:
                x = x - 50
                y = 512
                alpha = tar_or[0]
                beta = tar_or[1]
                gamma = math.atan2(y, x)
                orientation = [alpha, beta, gamma]
                vrep.simxSetObjectOrientation(clientID, target_handle, -1, orientation,
                                              vrep.simx_opmode_blocking)

            cv2.imshow('Fig', edges)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            elif err == vrep.simx_return_novalue_flag:
                print("no image yet")
                pass

    cv2.waitKey(0)  # waits until a key is pressed
    cv2.destroyAllWindows()