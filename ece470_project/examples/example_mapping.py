import support.vrep as vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm
import matplotlib.pyplot as plt

# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

# Close all open connections (Clear bad cache)
vrep.simxFinish(-1)
# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
	raise Exception('Failed connecting to remote API server')

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
lidar_motor     = get_handle_blocking('Tower_Turning_Joint')



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

def get_lidar_angle():
    result, theta = vrep.simxGetJointPosition(clientID, lidar_motor, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get lidar angle')
    return theta

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

forwBackVel = 0
leftRightVel = 0
rotVel = 0
# Moving forwards at 10
set_move(forwBackVel, leftRightVel, rotVel)

# ******************************** Your robot control code goes here  ******************************** #

points = []
# Wait two seconds
for i in range(200):
    # 1: True if detected, False if not.
    # 2: 3D vector containing position of detected point.
    # 3: Object handle of the detected object.
    # 4: Surface normal vector of the detected point.
    result = vrep.simxReadProximitySensor(clientID, prox_sensor, vrep.simx_opmode_blocking)
    # print(result)
    if result[1]:
        angle = get_lidar_angle()
        print("angle", angle)
        depth = result[2][2]
        print("z", depth)
        print(result[2])
        points.append((depth * np.cos(angle), depth * np.sin(angle)))
    
    time.sleep(0.01)

points = np.array(points).T
print(points)
plt.plot(points[0], points[1], 'o')
plt.show()
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