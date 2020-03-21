import vrep
import time
import numpy as np
import math
from scipy.linalg import expm,logm

# ======================================= Helper Functions ============================================== #
class robot_motion:
	""" Class for the 2D motion of the entire robot frame. """

	def __init__(self, clientID, wheelJoints):
		""" Initialize a motion object. """
		self.clientID = clientID
		self.wheelJoints = wheelJoints
		# min and max wheel rotation vel. for backward/forward movement
		self.forwBackVelRange = [-240*math.pi/180, 240*math.pi/180]
		# min and max wheel rotation vel. for left/right movement
		self.leftRightVelRange = [-240*math.pi/180, 240*math.pi/180]
		# min and max wheel rotation vel. for left/right rotation movement
		self.rotVelRange = [-240*math.pi/180, 240*math.pi/180]

	def set_move(forwBackVel, leftRightVel, rotVel):
		""" Move at a given velocity. """
		if forwBackVel < self.forwBackVelRange[0] or forwBackVel > self.forwBackVelRange[1]:
			print("SET_MOVE ERROR: Forward Backward Velocity value out of range.\n")
			return
		if leftRightVel < self.leftRightVelRange[0] or leftRightVel > self.leftRightVelRange[1]:
			print("SET_MOVE ERROR: Left Right Velocity value out of range.\n")
			return
		if rotVel < self.rotVelRange[0] or rotVel > self.rotVelRange[1]:
			print("SET_MOVE ERROR: Rotational Velocity value out of range.\n")
			return
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[0],-forwBackVel-leftRightVel-rotVel, vrep.simx_opmode_blocking)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[1],-forwBackVel+leftRightVel-rotVel, vrep.simx_opmode_blocking)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[2],-forwBackVel-leftRightVel+rotVel, vrep.simx_opmode_blocking)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[3],-forwBackVel+leftRightVel+rotVel, vrep.simx_opmode_blocking)
