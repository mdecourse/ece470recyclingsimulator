import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm, logm
import matplotlib.pyplot as plt
from scripts.robot_motion import *
from scripts.robot_lidar import *

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
target          = get_handle_blocking('youBot_positionTarget')
prox_sensor     = get_handle_blocking('Proximity_sensor')
lidar_motor     = get_handle_blocking('Tower_Turning_Joint')

armJoints = [-1] * 5
for i in range(5):
    armJoints[i] = get_handle_blocking('youBotArmJoint{}'.format(i))
# ======================================================================================================= #
# ======================================= Start Simulation ============================================== #
# ======================================================================================================= #

vrep.simxSynchronous(clientID, 1)
vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
vrep.simxSynchronousTrigger(clientID)

robot_motion = robot_motion(clientID, youBotRef, wheelJoints, armJoints[0])
robot_lidar = robot_lidar(clientID, prox_sensor, lidar_motor)
robot_lidar.set_lidar_velocity(np.pi)

pos = robot_motion.get_global_position()
ori = robot_motion.get_global_orientation()
endpos = [0,0,0]
endpos[0] = pos[0] + 1
endpos[1] = pos[1] - 1
endpos[2] = pos[2]

robot_motion.move_global_position(pos, endpos, 0.01)
# robot_motion.set_move(0,0,math.pi)
# time.sleep((360/240)*1-0.1)
# robot_motion.get_global_orientation()
# robot_motion.set_move(0,0,0)
time.sleep(3)
robot_motion.get_global_position()
robot_motion.get_global_orientation()
robot_motion.set_move(1,0,0)

# Simulation dt is 50ms (0.05s)
dt = 0.05
points = []
for i in range(50):
    # Trigger a "tick"
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)
    
    pt = robot_lidar.read_lidar_point()
    if pt:
        points.append(pt)
        
robot_motion.set_move(0,0,0)
vrep.simxSynchronousTrigger(clientID)


points = np.array(points).T
# print(points)
plt.plot(points[0], points[1], 'o')
plt.show()

# ======================================================================================================= #
# ======================================== End Simulation =============================================== #
# ======================================================================================================= #

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)
# Close the connection to V-REP
vrep.simxFinish(clientID)
print("==================== ** Simulation Ended ** ====================")
