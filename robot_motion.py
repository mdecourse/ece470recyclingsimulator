import vrep
import time
import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm

# ======================================= Helper Functions ============================================== #

# Move at a given velocity
def set_move(forwBackVel, leftRightVel, rotVel):
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[0],-forwBackVel-leftRightVel-rotVel, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[1],-forwBackVel+leftRightVel-rotVel, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[2],-forwBackVel-leftRightVel+rotVel, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(clientID, wheelJoints[3],-forwBackVel+leftRightVel+rotVel, vrep.simx_opmode_blocking)
