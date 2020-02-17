import vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm

# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

# Function that used to move joints
def SetJointPosition():
	print("attempting angles boyo")
	vrep.simxSetJointPosition(clientID, armJoints[0], np.pi/2, vrep.simx_opmode_blocking)
	time.sleep(0.5)
	vrep.simxSetJointPosition(clientID, armJoints[1], -np.pi/4, vrep.simx_opmode_blocking)
	time.sleep(0.5)
	vrep.simxSetJointPosition(clientID, armJoints[2], np.pi/4, vrep.simx_opmode_blocking)
	time.sleep(0.5)
	vrep.simxSetJointPosition(clientID, armJoints[3], np.pi/4, vrep.simx_opmode_blocking)
	time.sleep(0.5)
	vrep.simxSetJointPosition(clientID, armJoints[4], np.pi/4, vrep.simx_opmode_blocking)
	time.sleep(0.5)

# ======================================== Setup "handle"  =========================================== #

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
target          = get_handle_blocking('youBot_positionTarget')
prox_sensor     = get_handle_blocking('Proximity_sensor')


armJoints = [-1] * 5
for i in range(5):
    armJoints[i] = get_handle_blocking('youBotArmJoint{}'.format(i))

# ==================================================================================================== #

# Move at a given velocity
def set_move(forwBackVel, leftRightVel, rotVel):
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[0],-forwBackVel-leftRightVel-rotVel, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[1],-forwBackVel+leftRightVel-rotVel, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[2],-forwBackVel-leftRightVel+rotVel, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[3],-forwBackVel+leftRightVel+rotVel, vrep.simx_opmode_blocking)

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

forwBackVel = 1
leftRightVel = 0
rotVel = 2
# Moving forwards at 10
set_move(forwBackVel, leftRightVel, rotVel)
# Goal_joint_angles = np.array([[0,0,-0.5*np.pi,0.5*np.pi,-0.5*np.pi,-0.5*np.pi], \
# 							[0.5*np.pi,0,-0.5*np.pi,0.5*np.pi,0.5*np.pi,-0.5*np.pi],\
# 							[-0.5*np.pi,-0.5*np.pi,-0.5*np.pi,0,-0.5*np.pi,-0.5*np.pi]])
SetJointPosition()

# ******************************** Your robot control code goes here  ******************************** #

# Wait two seconds
for i in range(100):
    result = vrep.simxReadProximitySensor(clientID, prox_sensor, vrep.simx_opmode_blocking)
    print(result)
    time.sleep(0.01)
# **************************************************************************************************** #

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #
