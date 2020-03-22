import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm,logm

absolute_position = -1

# ======================================= Helper Functions ============================================== #
class robot_motion:
	""" Class for the 2D motion of the entire robot frame. """

	def __init__(self, clientID, youBotRef, wheelJoints, arm):
		""" Initialize a motion object. """
		self.clientID = clientID
		self.wheelJoints = wheelJoints
		self.youBotRef = youBotRef
		self.baseArm = arm
		# min and max wheel rotation vel. for backward/forward movement
		self.forwBackVelRange = [-240*math.pi/180, 240*math.pi/180]
		# min and max wheel rotation vel. for left/right movement
		self.leftRightVelRange = [-240*math.pi/180, 240*math.pi/180]
		# min and max wheel rotation vel. for left/right rotation movement
		self.rotVelRange = [-240*math.pi/180, 240*math.pi/180]

	def get_global_position(self):
		error,position = vrep.simxGetObjectPosition(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_blocking)
		if error:
			print("get_global_position ERROR")
		print("POSITION: x="+str(position[0])+" y="+str(position[1])+" z="+str(position[2])+"\n")
		return position

	def get_any_global_position(self, handle):
		error,position = vrep.simxGetObjectPosition(self.clientID, handle, absolute_position, vrep.simx_opmode_blocking)
		if error:
			print("get_global_position ERROR")
		print("POSITION: x="+str(position[0])+" y="+str(position[1])+" z="+str(position[2])+"\n")
		return position

	def get_global_orientation(self):
		error,euler = vrep.simxGetObjectOrientation(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_blocking)
		if error:
			print("get_global_position ERROR")
		print("EULER ANGLES: x="+str(euler[0])+" y="+str(euler[1])+" z="+str(euler[2])+"\n")
		return euler

	def move_global_position(self, curr_pos, end_pos, tolerance):
		curr_pos = np.array(self.get_global_position())
		curr_pos_arm = np.array(self.get_any_global_position(self.baseArm))

		base_vector = curr_pos_arm - curr_pos
		dest_vector = end_pos - curr_pos

		theta = (base_vector[0]*dest_vector[0]+base_vector[1]*dest_vector[1])
		theta = theta / ((base_vector[0]**2+base_vector[1]**2)**0.5 * (dest_vector[0]**2+dest_vector[1]**2)**0.5)
		theta = np.arccos(theta)

		distance = ((curr_pos[0]-end_pos[0])**2+(curr_pos[1]-end_pos[1])**2)**0.5

		while abs(theta) > tolerance and \
			  ((base_vector[0] != dest_vector[0]) or \
			   (base_vector[1] != dest_vector[1])):
			curr_pos = np.array(self.get_global_position())
			curr_pos_arm = np.array(self.get_any_global_position(self.baseArm))
			base_vector = curr_pos_arm - curr_pos
			dest_vector = end_pos - curr_pos
			theta = (base_vector[0]*dest_vector[0]+base_vector[1]*dest_vector[1])
			theta = theta / ((base_vector[0]**2+base_vector[1]**2)**0.5 * (dest_vector[0]**2+dest_vector[1]**2)**0.5)
			theta = np.arccos(theta)
			print("THETA "+str(theta))
			if theta > 0:
				self.set_move(0, 0, 10*theta/math.pi)
				# time.sleep(0.1)
			else:
				self.set_move(0, 0, -10*theta/math.pi)
		self.set_move(0,0,0)

		prev_dist = distance
		vel = 5*distance/math.pi
		while distance > tolerance*3:
			curr_pos = np.array(self.get_global_position())
			distance = ((curr_pos[0]-end_pos[0])**2+(curr_pos[1]-end_pos[1])**2)**0.5
			print("DISTANCE "+str(distance))
			print("prev: "+str(prev_dist))
			if distance > prev_dist:
				if vel < 0:
					vel = 4*distance/math.pi
				else:
					vel = -4*distance/math.pi
			self.set_move(vel, 0, 0)
			time.sleep(0.1)
			prev_dist = distance
		self.set_move(0,0,0)

	def set_move(self, forwBackVel, leftRightVel, rotVel):
		""" Move at a given velocity. """
		if forwBackVel < self.forwBackVelRange[0]:
			print("set_move TOO LOW: Forward Backward Velocity value out of range.\n")
			forwBackVel = self.forwBackVelRange[0]
		elif forwBackVel > self.forwBackVelRange[1]:
			print("set_move TOO HIGH: Forward Backward Velocity value out of range.\n")
			forwBackVel = self.forwBackVelRange[1]

		if leftRightVel < self.leftRightVelRange[0]:
			print("set_move TOO LOW: Left Right Velocity value out of range.\n")
			leftRightVel = self.leftRightVelRange[0]
		elif leftRightVel > self.leftRightVelRange[1]:
			print("set_move TOO HIGH: Left Right Velocity value out of range.\n")
			leftRightVel = self.leftRightVelRange[1]

		if rotVel < self.rotVelRange[0]:
			print("set_move TOO LOW: Rotational Velocity value out of range.\n")
			rotVel = self.rotVelRange[0]
		elif rotVel > self.rotVelRange[1]:
			print("set_move TOO HIGH: Rotational Velocity value out of range.\n")
			rotVel = self.rotVelRange[1]

		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[0],-forwBackVel-leftRightVel-rotVel, vrep.simx_opmode_blocking)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[1],-forwBackVel+leftRightVel-rotVel, vrep.simx_opmode_blocking)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[2],-forwBackVel-leftRightVel+rotVel, vrep.simx_opmode_blocking)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[3],-forwBackVel+leftRightVel+rotVel, vrep.simx_opmode_blocking)
