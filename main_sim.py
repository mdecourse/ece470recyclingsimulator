import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm, logm
import matplotlib.pyplot as plt
from scripts.robot_motion import *
from scripts.arm_motion import *
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
gripper 		= get_handle_blocking('youBotGripperJoint1')

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

robot_lidar = robot_lidar(clientID, prox_sensor, lidar_motor)
robot_lidar.set_lidar_velocity(6)

# Simulation dt is 50ms (0.05s)
dt = 0.05

pos = robot_motion.get_global_position()
print(pos)
theta = 0
rot_v = math.pi / 10
for i in range(200):
    # Trigger a "tick"
    robot_motion.set_move(0.05 * (math.cos(theta) - math.sin(theta)), 0.05 * (math.cos(theta) + math.sin(theta)), rot_v)
    theta += dt * rot_v
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)
    robot_motion.motion_update()
    arm_motion.motion_update()

robot_motion.set_move(0,0,0)
pos = robot_motion.get_global_position()
print(pos)

time.sleep(1)

# print("\nATTEMPTING TO MOVE ROBOT +0.5 IN X AND -0.5 IN Y.\n")
# pos = robot_motion.get_global_position()
# endpos = [pos[0] + 0.5, pos[1] - 0.5, pos[2]]

# robot_motion.set_move_global_position(endpos, 0.01)
# robot_motion.get_global_position()

# print("\nATTEMPTING TO MOVE ARM.\n")
# thetas = [math.pi/2, -math.pi/4, math.pi/4, math.pi/4, math.pi/4]
# prediction = arm_motion.forw_kin(thetas)
# print("FORWARD KINEMATICS PREDICTED GRIPPER POSITION:")
# print(prediction)
# print()
# arm_motion.set_target_arm_angles(thetas)

# while True:
    # # Trigger a "tick"
    # vrep.simxSynchronousTrigger(clientID)
    # vrep.simxGetPingTime(clientID)
    # arm_motion.motion_update()
    # if not robot_motion.motion_update():
        # break

# print("SIMULATOR READ GRIPPER POSITION")
# read_pos = arm_motion.get_any_ref_position(gripper, youBot)
# print()

# robot_motion.set_move(0,0,0)
# arm_motion.hold_position()

# time.sleep(1)

# print("\nATTEMPTING TO READ LIDAR SENSOR.\n")

print("Scanning...")
points = []
for i in range(200):
    # Trigger a "tick"
    vrep.simxSynchronousTrigger(clientID)
    vrep.simxGetPingTime(clientID)

    robot_motion.motion_update()
    arm_motion.motion_update()
    
    pt = robot_lidar.read_lidar_point()
    if pt:
        points.append(pt)


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
