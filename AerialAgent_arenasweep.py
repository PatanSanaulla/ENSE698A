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


start = (-5,50)
arena_1Q = (100,100)
arena_2Q = (0,100)
arena_3Q = (0,0)
arena_4Q = (100,-0)
fov = 120
h = 5
acceptance_radius = .5
V = .1


try:
    import vrep
    import sys
    #from threading import Thread
    import time
    from time import sleep
    import numpy as np

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

    traj = start

    cov = 2*h*np.tan(0.5*fov*(np.pi/180))
    N_cov = int(np.ceil((arena_4Q[0] - arena_3Q[0])/cov))
    N_wpts = 2*N_cov+2+1
    wpt = [0 for x in range(N_wpts)]
    i_cov = 0
    k=1
    wpt[0] = (start[0], start[1], h)
    while(i_cov<N_cov):

        if (np.mod(i_cov,2)!=0):
            wpt[k] = (arena_3Q[0]+(i_cov+0.5)*cov, arena_4Q[1]+0*0.5*cov, h)
            wpt[k+1] = (arena_2Q[0]+(i_cov+0.5)*cov, arena_1Q[1]-0*0.5*cov, h)
        else:
            wpt[k] = (arena_2Q[0]+(i_cov+0.5)*cov, arena_1Q[1]-0*0.5*cov, h)
            wpt[k+1] = (arena_3Q[0]+(i_cov+0.5)*cov, arena_4Q[1]+0*0.5*cov, h)

        k=k+2
        i_cov = i_cov+1

    wpt[k] = (start[0], start[1], h)
    wpt[k+1] = (start[0], start[1], 0)

    print(N_cov)
    print(len(wpt))
    print(wpt)

    t0 = time.time()
    t = time.time()-t0
    k=0
    while (k<len(wpt)):
        (ret, quad_handle) = vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_oneshot)
        (ret, target_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot)

        # vrep.simxAddStatusbarMessage(clientID,repr(time.time()),vrep.simx_opmode_oneshot)

        (ret, pos_d) = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot)
        (ret, pos) = vrep.simxGetObjectPosition(clientID, quad_handle, -1,vrep.simx_opmode_oneshot)
        (ret, linvel, angvel) = vrep.simxGetObjectVelocity(clientID, quad_handle, vrep.simx_opmode_oneshot)
        (ret, Euler) = vrep.simxGetObjectOrientation(clientID, quad_handle, -1,vrep.simx_opmode_oneshot)

        pos = np.asarray(pos)

        vrep.simxSetObjectPosition(clientID,target_handle,-1,wpt[k],vrep.simx_opmode_oneshot)

        if (np.linalg.norm(np.subtract(pos,np.asarray(wpt[k]))) < acceptance_radius):
            # print(np.linalg.norm(np.subtract(pos,np.asarray(wpt[k]))))
            k=k+1

        time.sleep(.1)
        t = time.time() - t0

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection failed")
print ('Program ended')


