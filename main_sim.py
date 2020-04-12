import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm, logm
import matplotlib.pyplot as plt
from scripts.robot_motion import *
from scripts.robot_localization import *
from scripts.arm_motion import *
from scripts.robot_lidar import *
from scripts.vision import *
from pf_test import *
import sys
import threading as th

show_pf = False
for arg in sys.argv:
    if arg == "-show_pf":
        show_pf = True

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# ======================================== Setup "handle"  ============================================= #

# Get a handle by name
def get_handle_blocking(name):
    result, obj = vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object: {}'.format(name))
    return obj

wheelJoints = [-1, -1, -1, -1]
wheelJoints[0]  = get_handle_blocking('rollingJoint_fl')
wheelJoints[1]  = get_handle_blocking('rollingJoint_rl')
wheelJoints[2]  = get_handle_blocking('rollingJoint_rr')
wheelJoints[3]  = get_handle_blocking('rollingJoint_fr')
youBot          = get_handle_blocking('youBot')
youBotRef       = get_handle_blocking('youBot_ref')
tip             = get_handle_blocking('youBot_positionTip')
prox_sensor     = get_handle_blocking('Proximity_sensor')
lidar_motor     = get_handle_blocking('Tower_Turning_Joint')
gripper 		= get_handle_blocking('youBotGripperJoint1')
vision_sens   = get_handle_blocking('Vision_sensor')

armJoints = [-1] * 5
for i in range(5):
    armJoints[i] = get_handle_blocking('youBotArmJoint{}'.format(i))
# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

vrep.simxSynchronous(clientID, 1)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

# initialize motion classes
robot_motion = robot_motion(clientID, youBotRef, wheelJoints, armJoints[0])
arm_motion = arm_motion(clientID, youBotRef, armJoints, youBot, gripper)
pf = particle_filter(80, KNOWN_MAP)

# Simulation dt is 50ms (0.05s)
dt = 0.05
lidar_v = 6

# initialize sensor classes
vision_sensor = vision_sensor(clientID, vision_sens)
robot_lidar = robot_lidar(clientID, prox_sensor, lidar_motor)
robot_lidar.set_lidar_velocity(lidar_v)

# Hack to do manual robot control
keep_going = True
vfb = 0
vlr = 0
vt = 0
def key_capture_thread():
    global keep_going
    global vfb
    global vlr
    global vt
    while True:
        in_str = input().strip()
        for c in in_str:
            if c == "q":
                keep_going = False
                break
            elif c == "w":
                vfb += 0.1
            elif c == "s":
                vfb -= 0.1
            elif c == "d":
                vlr += 0.1
            elif c == "a":
                vlr -= 0.1
            elif c == "x":
                vt -= 0.2
            elif c == "z":
                vt += 0.2

th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()
i = 0
readings = []
while keep_going: # Running for 100s
    robot_motion.set_move(vfb, vlr, vt)
    vision_sensor.read_sensor()
    # Trigger a "tick"
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

    lidar_result = robot_lidar.get_lidar_raw();
    if lidar_result:
        readings.append(lidar_result)
    else:
        readings.append(5)

    pf.update(vfb, vlr, vt, lidar_v, dt)
    i += 1
    if i == 30:
        i = 0
        pf.resample(readings)
        if show_pf:
            visualize_pf(pf)
            plt.ion()
            plt.show()
            plt.pause(0.001)
        readings = []
        print("Prediction:",pf.get_predicted_pose()[:2, -1])
        pos = robot_motion.get_global_position()
        print("Actual:[{} {}]".format(pos[0] + 2, pos[1] + 2))

    robot_motion.motion_update()
    arm_motion.motion_update()

# robot_motion.set_move(0,0,0)
# pos = robot_motion.get_global_position()
# print(pos)


# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")
