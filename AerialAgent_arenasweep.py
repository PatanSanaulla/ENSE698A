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

center = (0,0)

start = (0,-45)
arena_1Q = (50,50)
arena_2Q = (-50,50)
arena_3Q = (-50,-50)
arena_4Q = (50,-50)
fov = 45
h = 15
acceptance_radius = .5
V = .1

try:
    import vrep
    import sys
    #from threading import Thread
    import time
    from time import sleep
    import numpy as np
    import cv2

except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')


print ('Program started to execute in V-Rep')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP

if clientID != -1:
    print('Connection Established to remote API server')

    #main code to execute
    #can start multiple threads to work at same time
    #thread = Thread(target=threaded_function)
    #thread.start()

    print ('Connected to remote API server')

    # Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Sweep Start!',vrep.simx_opmode_oneshot) #This message should be printed on your CopelliaSim in the bottm

    obs_map = np.zeros(shape=[1000, 1000], dtype=np.uint8)
    OBJS_X = []
    OBJS_Y = []
    flag_saved = False

    traj = start

    cov = 2*h*np.tan(0.5*fov*(np.pi/180.0))
    N_cov = int(np.floor((arena_4Q[0] - arena_3Q[0])/cov))
    N_wpts = 2*N_cov+2+2
    wpt = [0 for x in range(N_wpts)]
    i_cov = 0
    k=1
    wpt[0] = (start[0], start[1], h)
    wpt[1] = (center[0], center[1], 50/np.tan(0.5*fov*(np.pi/180.0)) + 2.5)
    k=k+1
    while(i_cov<N_cov):

        if (np.mod(i_cov,2)!=0):
            wpt[k] = (arena_3Q[0]+(i_cov+0.5)*cov, arena_4Q[1]+(-1)*0.5*cov, h)
            wpt[k+1] = (arena_2Q[0]+(i_cov+0.5)*cov, arena_1Q[1]-(-1)*0.5*cov, h)
        else:
            wpt[k] = (arena_2Q[0]+(i_cov+0.5)*cov, arena_1Q[1]-(-1)*0.5*cov, h)
            wpt[k+1] = (arena_3Q[0]+(i_cov+0.5)*cov, arena_4Q[1]+(-1)*0.5*cov, h)

        k=k+2
        i_cov = i_cov+1

    wpt[k] = (start[0], start[1], h)
    wpt[k+1] = (start[0], start[1], 0)

    # print(N_cov)
    # print(len(wpt))
    # print(wpt)

    t0 = time.time()
    t = time.time()-t0
    k=0
    # while (k < 1):
    while (1):

        if (k>len(wpt)-1):
            break

        (ret, quad_handle) = vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_oneshot)
        (ret, target_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot)
        (ret, camera_handle) = vrep.simxGetObjectHandle(clientID, 'FPV_Camera', vrep.simx_opmode_oneshot_wait)

        # vrep.simxAddStatusbarMessage(clientID,repr(time.time()),vrep.simx_opmode_oneshot)

        (ret, pos_d) = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot)
        (ret, pos) = vrep.simxGetObjectPosition(clientID, quad_handle, -1,vrep.simx_opmode_oneshot)
        (ret, linvel, angvel) = vrep.simxGetObjectVelocity(clientID, quad_handle, vrep.simx_opmode_oneshot)
        (ret, Euler) = vrep.simxGetObjectOrientation(clientID, quad_handle, -1,vrep.simx_opmode_oneshot)

        pos = np.asarray(pos)

        vrep.simxSetObjectPosition(clientID,target_handle,-1,wpt[k],vrep.simx_opmode_oneshot)

        if (np.linalg.norm(np.subtract(pos,np.asarray(wpt[k]))) < acceptance_radius):
            if (k==1):
                time.sleep(5) # let the drone stabilize
                ## GET OBS MAP
                (ret, reso, raw_img) = vrep.simxGetVisionSensorImage(clientID, camera_handle, 0,vrep.simx_opmode_streaming)
                raw_img = np.array(raw_img,dtype=np.uint8)
                # print(raw_img)
                if (len(raw_img)!=0):
                    raw_img = np.reshape(raw_img,np.append(reso,3))
                    raw_img = cv2.resize(raw_img, (1000,1000), interpolation = cv2.INTER_AREA)
                    raw_img = cv2.flip(raw_img, 0) # vertical flip
                    # raw_img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2HSV)

                    # obstacle 1 (white)
                    lower = (225, 225, 225)  # lower threshhold values
                    upper = (255, 255, 255)  # upper threshhold values
                    obs_1 = cv2.inRange(raw_img, lower, upper)

                    # obstacle 2 (grey)
                    lower = (145, 145, 145)  # lower threshhold values
                    upper = (148, 148, 148)  # upper threshhold values
                    obs_2 = cv2.inRange(raw_img, lower, upper)

                    obs_map = cv2.bitwise_or(obs_1, obs_2, mask = None)

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
                    kernel = np.ones((2,2), np.uint8)
                    obs_map = cv2.dilate(obs_map, kernel, iterations=1)

                    cv2.imwrite('obs_map.png', obs_map)

                    cv2.imshow("image", obs_map)
                    cv2.waitKey(1)
                    k=k+1
            else:
                ## waypoint complete
                print(np.linalg.norm(np.subtract(pos,np.asarray(wpt[k]))))
                k=k+1

        if (k>2 and k<len(wpt)-2):
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

                objs = cv2.bitwise_or(objs, frame, mask = None)

                # obstacle 2 (grey)
                lower = (145, 145, 145)  # lower threshhold values
                upper = (148, 148, 148)  # upper threshhold values
                frame = cv2.inRange(raw_img, lower, upper)

                objs = cv2.bitwise_or(objs, frame, mask = None)

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
                    if cv2.contourArea(c) < 25: #threshold for max contour area
                        M = cv2.moments(c)
                        if (M["m00"]!=0):
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            if (cX>50 and cX<1000-50 and cY>300 and cY<1000-300):
                                cv2.circle(raw_img, (cX, cY), 30, (255,0,0), 5)
                                f = 500.0 / np.tan(0.5 * fov * (np.pi / 180.0))
                                x_pos = ((cX - 500.0) / f) * (pos[2]-0.1) + pos[0]
                                y_pos = (-(cY - 500.0) / f) * (pos[2]-0.1) + pos[1]
                                print(x_pos, y_pos)
                                OBJS_X = np.append(OBJS_X, x_pos)
                                OBJS_Y = np.append(OBJS_Y, y_pos)
                                # print(OBJS_X,OBJS_Y)
                                # print(len(OBJS_X))
                                # OBJS = np.asarray([OBJS_X,OBJS_Y])
                                # # print(OBJS)
                                # np.savetxt('OBJS.txt',np.transpose(OBJS))

                debug_image = raw_img

                cv2.imshow("image", debug_image)
                cv2.waitKey(1)

        if (k==len(wpt)-2 and flag_saved==False):
            OBJS = np.asarray([OBJS_X, OBJS_Y])
            print(OBJS)
            np.savetxt('OBJS.txt', np.transpose(OBJS))
            flag_saved=True
            print("File SAVED !!!!!!!!!")

        time.sleep(.01)
        t = time.time() - t0
    #eow

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection failed")
print ('Program ended')


