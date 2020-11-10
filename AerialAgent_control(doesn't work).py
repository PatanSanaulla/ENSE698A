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

m = 10; j11 = 1; j22 = 1; j33 =3
Kp = 1; Kv = 1; K_eul = 1; K_w = 1;
k_f = .1; k_m = 0.1*k_f; L = 1;

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
    vrep.simxAddStatusbarMessage(clientID,'Control Start!',vrep.simx_opmode_oneshot) #This message should be printed on your CopelliaSim in the bottm

    t0 = time.time();
    t = time.time()-t0
    while (t<=10):
        (ret, quad_handle) = vrep.simxGetObjectHandle(clientID,'Quadricopter_base',vrep.simx_opmode_oneshot)
        (ret, target_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot)

        # vrep.simxAddStatusbarMessage(clientID,repr(time.time()),vrep.simx_opmode_oneshot)

        (ret, pos_d) = vrep.simxGetObjectPosition(clientID, target_handle, -1, vrep.simx_opmode_oneshot)
        (ret, pos) = vrep.simxGetObjectPosition(clientID, quad_handle, -1,vrep.simx_opmode_oneshot)
        (ret, linvel, angvel) = vrep.simxGetObjectVelocity(clientID, quad_handle, vrep.simx_opmode_oneshot)
        (ret, Euler) = vrep.simxGetObjectOrientation(clientID, quad_handle, -1,vrep.simx_opmode_oneshot)
        # print(Euler)

        pos_d = np.asarray(pos_d)
        pos = np.asarray(pos)
        linvel = np.asarray(linvel)
        angvel = np.asarray(angvel)
        Euler = np.asarray(Euler)

        U = Kp*(pos_d-pos) + Kv*(-linvel) + m*np.array([0,0,9.81])
        T = np.linalg.norm(U)
        yaw_d = 0.0
        roll_d = np.arcsin((-U[0]*np.sin(yaw_d) + U[1]*np.cos(yaw_d))/T)
        pitch_d = np.arcsin(-U[0]*np.cos(yaw_d) - U[1]*np.sin(yaw_d))/(T*np.cos(roll_d))
        Eul_d = np.array([roll_d,pitch_d,yaw_d])

        M = K_eul*(Eul_d - Euler) + K_w*(-angvel)

        f1 = (0.25*T)/k_f - (0.25*M[2])/k_m - M[1]/(L*k_f)
        f2 = (0.25*T)/k_f + (0.25*M[2])/k_m + M[0]/(L*k_f)
        f3 = (0.25*T)/k_f - (0.25*M[2])/k_m + M[1]/(L*k_f)
        f4 = (0.25*T)/k_f + (0.25*M[2])/k_m - M[0]/(L*k_f)

        (ret, f1_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_propeller_joint1',vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID,f1_handle,f1,vrep.simx_opmode_oneshot)
        (ret, f2_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_propeller_joint2',vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID,f2_handle,f2,vrep.simx_opmode_oneshot)
        (ret, f3_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_propeller_joint3',vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID,f3_handle,f3,vrep.simx_opmode_oneshot)
        (ret, f4_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_propeller_joint4',vrep.simx_opmode_oneshot)
        vrep.simxSetJointForce(clientID,f4_handle,f4,vrep.simx_opmode_oneshot)

        # (ret, f1_handle) = vrep.simxGetObjectHandle(clientID, 'Quadricopter_propeller_respondable1', vrep.simx_opmode_oneshot)
        # vrep.simxSetObjectFloatParameter(clientID,f1_handle,'particleVelocity',f1,vrep.simx_opmode_oneshot)
        # vrep.simxCallScriptFunction(clientID,'Quadricopter_propeller_respondable1',,'sysCall_actuation',)

        # print(f1)

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


