import vrep
import sys
# from threading import Thread
import time
from time import sleep
import numpy as np
import cv2

def AerialAgentNavigation(k,OBJS):
    (ret, quad_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_base', vrep.simx_opmode_oneshot)
    (ret, target_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot)
    (ret, camera_handle) = vrep.simxGetObjectHandle(clientID, 'FPV_Camera', vrep.simx_opmode_oneshot_wait)

    # vrep.simxAddStatusbarMessage(clientID,repr(time.time()),vrep.simx_opmode_oneshot)

    (ret, pos_d) = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot)
    (ret, pos) = vrep.simxGetObjectPosition(clientID, quad_handle, -1, vrep.simx_opmode_oneshot)
    (ret, linvel, angvel) = vrep.simxGetObjectVelocity(clientID, quad_handle, vrep.simx_opmode_oneshot)
    (ret, Euler) = vrep.simxGetObjectOrientation(clientID, quad_handle, -1, vrep.simx_opmode_oneshot)

    pos = np.asarray(pos)

    vrep.simxSetObjectPosition(clientID, target_handle, -1, wpt[k], vrep.simx_opmode_oneshot)

    if (np.linalg.norm(np.subtract(pos, np.asarray(wpt[k]))) < acceptance_radius):
        if (k == 1):
            time.sleep(5)  # let the drone stabilize
            ## GET OBS MAP
            (ret, reso, raw_img) = vrep.simxGetVisionSensorImage(clientID, camera_handle, 0,
                                                                 vrep.simx_opmode_streaming)
            raw_img = np.array(raw_img, dtype=np.uint8)
            # print(raw_img)
            if (len(raw_img) != 0):
                raw_img = np.reshape(raw_img, np.append(reso, 3))
                raw_img = cv2.resize(raw_img, (1000, 1000), interpolation=cv2.INTER_AREA)
                raw_img = cv2.flip(raw_img, 0)  # vertical flip
                # raw_img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2HSV)

                # obstacle 1 (white)
                lower = (225, 225, 225)  # lower threshhold values
                upper = (255, 255, 255)  # upper threshhold values
                obs_1 = cv2.inRange(raw_img, lower, upper)

                # obstacle 2 (grey)
                lower = (145, 145, 145)  # lower threshhold values
                upper = (148, 148, 148)  # upper threshhold values
                obs_2 = cv2.inRange(raw_img, lower, upper)

                obs_map = cv2.bitwise_or(obs_1, obs_2, mask=None)

                # obstacle 3 (Red)
                lower = (200, 0, 0)  # lower threshhold values
                upper = (255, 50, 50)  # upper threshhold values
                obs_3 = cv2.inRange(raw_img, lower, upper)

                obs_map = cv2.bitwise_or(obs_map, obs_3, mask=None)

                # obstacle 4 (Green)
                lower = (0, 200, 0)  # lower threshhold values
                upper = (50, 255, 50)  # upper threshhold values
                obs_4 = cv2.inRange(raw_img, lower, upper)

                obs_map = cv2.bitwise_or(obs_map, obs_4, mask=None)

                # obstacle 5 (Blue)
                lower = (0, 0, 200)  # lower threshhold values
                upper = (50, 50, 255)  # upper threshhold values
                obs_5 = cv2.inRange(raw_img, lower, upper)

                obs_map = cv2.bitwise_or(obs_map, obs_5, mask=None)

                # obstacle 6 (Bluish green)
                lower = (150, 150, 200)  # lower threshhold values
                upper = (200, 255, 255)  # upper threshhold values
                obs_6 = cv2.inRange(raw_img, lower, upper)

                obs_map = cv2.bitwise_or(obs_map, obs_6, mask=None)

                # quad_x_pos_in_img = min(max(((pos[0] - arena_3Q[0]) * int(1000 / 100)),0),1000)
                # quad_y_pos_in_img = min(max((1000 - (pos[1] - arena_3Q[1]) * int(1000 / 100)),0),1000)

                # cam_len = int(cov * int(1000 / 100))
                #
                # cam_start_x = 100 + int(quad_x_pos_in_img) - int(0.5 * cam_len) - 1
                # cam_start_y = 100 + int(quad_y_pos_in_img) - int(0.5 * cam_len) - 1
                # cam_end_x = 100 + int(quad_x_pos_in_img) + int(0.5 * cam_len)
                # cam_end_y = 100 + int(quad_y_pos_in_img) + int(0.5 * cam_len)
                #
                # # if (cam_start_x>0 and cam_start_y>0 and cam_end_x<1000 and cam_end_y<1000):
                # obs_map[cam_start_y:cam_end_y,cam_start_x:cam_end_x] = cv2.resize(obs, (cam_len,cam_len), interpolation = cv2.INTER_AREA)

                # Erosion Dilation
                # kernel = np.ones((2,2), np.uint8)
                # obs_map = cv2.erode(obs_map, kernel, iterations=1)
                kernel = np.ones((2, 2), np.uint8)
                obs_map = cv2.dilate(obs_map, kernel, iterations=1)

                cv2.imwrite('obs_map.png', obs_map)

                cv2.imshow("image", obs_map)
                cv2.waitKey(1)
                k = k + 1
        else:
            ## waypoint complete
            # print(np.linalg.norm(np.subtract(pos,np.asarray(wpt[k]))))
            k = k + 1

    if (k > 2 and k < len(wpt) - 2):
        ## FIND OBJECTS LOACTION DURING ARENA SWEEP
        (ret, reso, raw_img) = vrep.simxGetVisionSensorImage(clientID, camera_handle, 0, vrep.simx_opmode_streaming)
        raw_img = np.array(raw_img, dtype=np.uint8)
        # print(raw_img)
        if (len(raw_img) != 0):
            raw_img = np.reshape(raw_img, np.append(reso, 3))
            raw_img = cv2.resize(raw_img, (1000, 1000), interpolation=cv2.INTER_AREA)
            raw_img = cv2.flip(raw_img, 0)  # vertical flip
            # raw_img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2HSV)

            # obstacle 2 (Grey)
            lower = (200, 0, 0)  # lower threshhold values
            upper = (255, 50, 50)  # upper threshhold values
            objs = cv2.inRange(raw_img, lower, upper)

            # obstacle 1 (white)
            lower = (225, 225, 225)  # lower threshhold values
            upper = (255, 255, 255)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # obstacle 2 (grey)
            lower = (145, 145, 145)  # lower threshhold values
            upper = (148, 148, 148)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # obstacle 3 (Red)
            lower = (200, 0, 0)  # lower threshhold values
            upper = (255, 50, 50)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # obstacle 3 (Pink)
            lower = (200, 100, 250)  # lower threshhold values
            upper = (255, 200, 255)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # obstacle 4 (Green)
            lower = (0, 200, 0)  # lower threshhold values
            upper = (50, 255, 50)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # obstacle 5 (Blue)
            lower = (0, 0, 200)  # lower threshhold values
            upper = (50, 50, 255)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # obstacle 5 (Yellow)
            lower = (220, 230, 160)  # lower threshhold values
            upper = (230, 250, 170)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # obstacle 5 (Bronze)
            lower = (220, 160, 150)  # lower threshhold values
            upper = (230, 200, 190)  # upper threshhold values
            frame = cv2.inRange(raw_img, lower, upper)

            objs = cv2.bitwise_or(objs, frame, mask=None)

            # cv2.imshow("image", objs)
            # cv2.waitKey(1)

            # Find contours and threshold on the area
            contours, hierarchy = cv2.findContours(objs, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            frame = np.zeros(shape=[1000, 1000, 1], dtype=np.uint8)
            # frame = cv2.drawContours(frame, [contours[0]], 0, 255, 3)

            for c in contours:
                if cv2.contourArea(c) < 25:  # threshold for max contour area
                    M = cv2.moments(c)
                    if (M["m00"] != 0):
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        if (cX > 50 and cX < 1000 - 50 and cY > 400 and cY < 1000 - 400):
                            cv2.circle(raw_img, (cX, cY), 30, (255, 0, 0), 5)
                            f = 500.0 / np.tan(0.5 * fov * (np.pi / 180.0))
                            x_pos = ((cX - 500.0) / f) * (pos[2] - 0.1) + pos[0]
                            if (wpt[k][1] > 0):
                                y_pos = (-(cY - 500.0) / f) * (pos[2] - 0.1) + pos[1] + 1.4 + 0.37
                            else:
                                y_pos = (-(cY - 500.0) / f) * (pos[2] - 0.1) + pos[1] - 1.4 - 0.27
                            print(x_pos, y_pos)
                            # OBJS_X = np.append(OBJS_X, x_pos)
                            # OBJS_Y = np.append(OBJS_Y, y_pos)
                            # print(OBJS_X,OBJS_Y)
                            # print(len(OBJS_X))
                            # OBJS = np.asarray([OBJS_X,OBJS_Y])
                            # # print(OBJS)
                            # np.savetxt('OBJS.txt',np.transpose(OBJS))

                            ## For clustering
                            OBJS = np.append(OBJS, x_pos)
                            OBJS = np.append(OBJS, y_pos)
                            if (len(OBJS) > 2):
                                ## call clustering
                                OBJS_clustered = clustering(OBJS)
                                print(OBJS_clustered)

            debug_image = raw_img

            cv2.imshow("image", debug_image)
            cv2.waitKey(1)

    return k, obs_map, OBJS, OBJS_clustered

def clustering(c):
    c = c.reshape((-1, 2))

    # print('Sorting...')
    idx = np.argsort(c[:, 0])
    c = c[idx]

    # print('Clustering...')

    epsilon = 0.5

    # Find euclidean distance between consecutive rows
    differences = np.diff(c, axis=0, prepend=0)
    euclidean_distance = np.linalg.norm(differences, axis=1)

    # Gather first index of unique measurments
    unique_idx = np.arange(len(c))[euclidean_distance > epsilon]
    print(f"There are {len(unique_idx)} objects to be collected.")

    # Average out values
    clustered = np.zeros(2)
    for i, idx in enumerate(unique_idx):
        idx1 = idx
        if i == len(unique_idx) - 1:
            average = np.average(c[idx1::], axis=0)
        else:
            idx2 = unique_idx[i + 1]
            average = np.average(c[idx1:idx2], axis=0)
        clustered = np.vstack((clustered, average))
        corrected = clustered[1:]

    return corrected