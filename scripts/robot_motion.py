import support.vrep as vrep
import time
import numpy as np
import numpy.linalg as la
import math
from scipy.linalg import expm, logm

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

        self.update_func = lambda: False

        vrep.simxGetObjectPosition(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_streaming)
        vrep.simxGetObjectOrientation(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_streaming)

    def motion_update(self):
        return self.update_func()

    def get_global_position(self):
        error,position = vrep.simxGetObjectPosition(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_buffer)
        if error:
            print("get_global_position ERROR")
        # print("POSITION: x="+str(position[0])+" y="+str(position[1])+" z="+str(position[2])+"\n")
        return position

    def get_any_global_position(self, handle):
        error,position = vrep.simxGetObjectPosition(self.clientID, handle, absolute_position, vrep.simx_opmode_blocking)
        if error:
            print("get_global_position ERROR")
        # print("POSITION: x="+str(position[0])+" y="+str(position[1])+" z="+str(position[2])+"\n")
        return position

    def get_global_orientation(self):
        error,euler = vrep.simxGetObjectOrientation(self.clientID, self.youBotRef, absolute_position, vrep.simx_opmode_buffer)
        if error:
            print("get_global_position ERROR")
        # print("EULER ANGLES: x="+str(euler[0])+" y="+str(euler[1])+" z="+str(euler[2])+"\n")
        return euler

    def set_move_global_position(self, end_pos, tolerance):
        def rotate_command(end_pos):
            curr_pos = np.array(self.get_global_position())
            curr_pos_arm = np.array(self.get_any_global_position(self.baseArm))

            base_vector = curr_pos_arm - curr_pos
            dest_vector = end_pos - curr_pos

            theta = (base_vector[0]*dest_vector[0]+base_vector[1]*dest_vector[1])
            theta = theta / (la.norm(base_vector[0:2]) * la.norm(dest_vector[0:2]))
            theta = np.arccos(theta)

            if abs(theta) > tolerance and \
                  ((base_vector[0] != dest_vector[0]) or \
                   (base_vector[1] != dest_vector[1])):

                # vrep.simxSynchronousTrigger(self.clientID)
                # vrep.simxGetPingTime(self.clientID)

                # curr_pos = np.array(self.get_global_position())
                # curr_pos_arm = np.array(self.get_any_global_position(self.baseArm))
                # base_vector = curr_pos_arm - curr_pos
                # dest_vector = end_pos - curr_pos
                # theta = (base_vector[0]*dest_vector[0]+base_vector[1]*dest_vector[1])
                # theta = theta / ((base_vector[0]**2+base_vector[1]**2)**0.5 * (dest_vector[0]**2+dest_vector[1]**2)**0.5)
                # theta = np.arccos(theta)
                # print("THETA "+str(theta))
                if theta > 0:
                    self.set_move(0, 0, min(10*theta/math.pi, self.rotVelRange[1]))
                    # time.sleep(0.1)
                else:
                    self.set_move(0, 0, max(-10*theta/math.pi, self.rotVelRange[0]))
            else:
                # print("Done Angles")
                self.set_move(0,0,0)
                def move_command(end_pos):
                    curr_pos = np.array(self.get_global_position())
                    curr_pos_arm = np.array(self.get_any_global_position(self.baseArm))

                    base_vector = curr_pos_arm - curr_pos
                    dest_vector = end_pos - curr_pos
                    # vel = 5*distance/math.pi
                    distance = la.norm(dest_vector)
                    if distance > tolerance*3:

                        # vrep.simxSynchronousTrigger(self.clientID)
                        # vrep.simxGetPingTime(self.clientID)

                        # print("DISTANCE "+str(distance))
                        # print("prev: "+str(prev_dist))
                        if np.dot(base_vector, dest_vector) > 0:
                            vel = 10*distance/math.pi
                        else:
                            vel = -10*distance/math.pi
                        self.set_move(vel, 0, 0)
                    else:
                        # print("Done Position")
                        self.set_move(0,0,0)
                        self.update_func = lambda: False
                    return True
                self.update_func = lambda: move_command(end_pos)
            return True

        self.update_func = lambda: rotate_command(end_pos)

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

        vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[0],-forwBackVel-leftRightVel-rotVel, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[1],-forwBackVel+leftRightVel-rotVel, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[2],-forwBackVel-leftRightVel+rotVel, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(self.clientID, self.wheelJoints[3],-forwBackVel+leftRightVel+rotVel, vrep.simx_opmode_oneshot)
