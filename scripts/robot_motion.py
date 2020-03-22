import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm, logm

absolute_position = -1

# ======================================= Helper Functions ============================================== #
class robot_motion:
	""" Class for the 2D motion of the entire robot frame. """

	def __init__(self, clientID, youBotRef, wheelJoints):
		""" Initialize a motion object. """
		self.clientID = clientID
		self.wheelJoints = wheelJoints
		self.youBotRef = youBotRef
		# min and max wheel rotation vel. for backward/forward movement
		self.forwBackVelRange = [-240*math.pi/180, 240*math.pi/180]
		# min and max wheel rotation vel. for left/right movement
		self.leftRightVelRange = [-240*math.pi/180, 240*math.pi/180]
		# min and max wheel rotation vel. for left/right rotation movement
		self.rotVelRange = [-240*math.pi/180, 240*math.pi/180]
		vrep.simxGetObjectPosition(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_streaming)

	def get_global_position(self):
		error,position = vrep.simxGetObjectPosition(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_buffer)
		if error:
			print("get_global_position ERROR")
		print("POSITION: x="+str(position[0])+" y="+str(position[1])+" z="+str(position[2])+"\n")
		return position

	def move_global_position(self, curr_pos, end_pos, tolerance):
		# curr_pos = self.get_global_position()
		rotVel = 0
		forwBackVel = 0
		leftRightVel = 0
		print(curr_pos)
		print(end_pos)
		while True:
			# if current position is with tolerated end position, end
			if (curr_pos[0] < end_pos[0] + tolerance and curr_pos[0] > end_pos[0] - tolerance) and \
				  (curr_pos[1] < end_pos[1] + tolerance and curr_pos[1] > end_pos[1] - tolerance):
				  return

			theta = np.arctan2(end_pos[1]-curr_pos[1],end_pos[0]-curr_pos[0])
			print(end_pos[1]-curr_pos[1])
			print(end_pos[0]-curr_pos[0])
			print("THETA "+str(theta))
			if theta < 0 - tolerance:
				rotVel = -math.pi
			elif theta > 0 + tolerance:
				rotVel = math.pi
			self.set_move(forwBackVel, leftRightVel, rotVel)
			time.sleep(theta/rotVel)
			time.sleep(3)
			rotVel = 0
			distance = ((curr_pos[0]-end_pos[0])**2+(curr_pos[1]-end_pos[1])**2)**0.5
			print("DISTANCE "+str(distance))
			if distance > 0:
				forwBackVel = math.pi
			self.set_move(forwBackVel, leftRightVel, rotVel)
			time.sleep(distance/forwBackVel)
			forwBackVel = 0

	def set_move(self, forwBackVel, leftRightVel, rotVel):
		""" Move at a given velocity. """
		if forwBackVel < self.forwBackVelRange[0] or forwBackVel > self.forwBackVelRange[1]:
			print("set_move ERROR: Forward Backward Velocity value out of range.\n")
			return
		if leftRightVel < self.leftRightVelRange[0] or leftRightVel > self.leftRightVelRange[1]:
			print("set_move ERROR: Left Right Velocity value out of range.\n")
			return
		if rotVel < self.rotVelRange[0] or rotVel > self.rotVelRange[1]:
			print("set_move ERROR: Rotational Velocity value out of range.\n")
			return
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[0],-forwBackVel-leftRightVel-rotVel, vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[1],-forwBackVel+leftRightVel-rotVel, vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[2],-forwBackVel-leftRightVel+rotVel, vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[3],-forwBackVel+leftRightVel+rotVel, vrep.simx_opmode_oneshot)
