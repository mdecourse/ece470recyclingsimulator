import support.vrep as vrep
import time
import numpy as np
import numpy.linalg as la
import math
from scipy.linalg import expm, logm
from scripts.utils import *

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
        
        self.velocities = (0, 0, 0)

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

    def set_move_global_position2(self, end_pos, dijkstras_callback, get_pose_callback, tolerance):
        storage = []
        def everything_command(end_pos, dijkstras_callback, get_pose_callback, storage):
            trans_vel = 0.2
            rot_vel = 0.4
            
            pos, angle = decompose_pose2D(get_pose_callback())
            angle = angle + np.pi / 2
            if angle < 0:
                angle = angle + 2 * np.pi
            if la.norm(pos - end_pos) < tolerance:
                storage.append(0)
                if len(storage) >= 60:
                    self.update_func = lambda: False
                    self.set_move(0, 0, 0)
                    return True
            else:
                storage.clear()
            my_gridpos = tuple(int(x*10) for x in pos)
            target_gridpos = tuple(int(x*10) for x in end_pos)
            print("Pathing from {} to {}".format(my_gridpos, target_gridpos))
            turn_angle = dijkstras_callback(my_gridpos, target_gridpos)
            if turn_angle != -1:
                print(turn_angle)
                turn_direction = angle_delta(angle, turn_angle)
                print(turn_direction)
                xyv = trans_vel
                if abs(turn_direction) > 0.15:
                    xyv *= 0.25
                self.set_move(xyv * math.cos(turn_direction), -xyv * math.sin(turn_direction), np.clip(rot_vel*turn_direction, -0.4, 0.4))
                # if abs(turn_direction) > 0.15:
                    # self.set_move(0, 0, np.clip(rot_vel*turn_direction, -2*rot_vel, 2*rot_vel))
                # else:
                    # # TODO turn angle
                    # self.set_move(trans_vel * math.cos(turn_direction), trans_vel * math.sin(turn_direction), 0)
            return True
        self.update_func = lambda: everything_command(end_pos, dijkstras_callback, get_pose_callback, storage)
    
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
                    self.set_move(0, 0, min(400*theta/math.pi, self.rotVelRange[1]))
                    # time.sleep(0.1)
                else:
                    self.set_move(0, 0, max(-400*theta/math.pi, self.rotVelRange[0]))
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
                            vel = min(400*distance/math.pi, self.forwBackVelRange[1])
                        else:
                            vel = max(-400*distance/math.pi, self.forwBackVelRange[0])
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
        """ Move at a given velocity. Velocity is in m/s for translational, and rad/sec for rotational. """
        self.velocities = (forwBackVel, leftRightVel, rotVel)
        forwBackVel *= 20
        leftRightVel *= 20
        rotVel *= -7.77
        
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
