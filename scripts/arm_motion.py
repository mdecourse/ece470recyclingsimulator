import support.vrep as vrep
import time
import numpy as np
import numpy.linalg as la
import math
from scipy.linalg import expm,logm

absolute_position = -1

# ======================================= Helper Functions ============================================== #
class arm_motion:
    """ Class for the 2D motion of the entire robot frame. """

    def __init__(self, clientID, youBotRef, arms, youBot, gripper):
        """ Initialize a motion object. """
        self.clientID = clientID
        self.youBotRef = youBotRef
        self.youBot = youBot
        self.arms = arms
        self.gripper = gripper
        
        for arm in arms:
            vrep.simxGetJointPosition(self.clientID, arm, vrep.simx_opmode_streaming)
        
        self.update_func = lambda: False
        
    def zero(self):

        # forward kinematics for arm joints
        # zero position, relative to youBot frame
        self.w = np.array([[1,0,0],[0,1,0],[0,1,0],[0,1,0],[1,0,0]])
        self.r = []
        self.v = []
        for arm, w in zip(self.arms,self.w):
            r = self.get_any_ref_position(arm, self.youBot)
            self.r.append(r)
            self.v.append(np.cross(-w,r))
            
        pos = self.get_any_ref_position(self.gripper, self.youBot)
        self.M = np.array([[-1,0,0,pos[0]],[0,0,1,pos[1]],[0,1,0,pos[2]],[0,0,0,1]])
        
    def motion_update(self):
        return self.update_func()

    def get_arm_angles(self):
        angles = []
        for arm_joint in self.arms:
            angles.append(vrep.simxGetJointPosition(self.clientID, arm_joint, vrep.simx_opmode_buffer)[1])
        print(angles)
        return np.array(angles)
        
    def set_target_arm_angles(self, target, tolerance=0.01):
        target = np.array(target)
        def set_angle_loop(target, tolerance):
            for arm_joint, theta in zip(self.arms, target):
                vrep.simxSetJointPosition(self.clientID, arm_joint, theta, vrep.simx_opmode_oneshot)
            if la.norm(self.get_arm_angles() - target) < la.norm(np.ones(len(self.arms)) * 0.01):
                self.update_func = lambda: False
                print("Arm target angle reached")
                return True
            return True
        self.update_func = lambda: set_angle_loop(target, tolerance)

    def get_any_ref_position(self, handle, reference):
        error,position = vrep.simxGetObjectPosition(self.clientID, handle, reference, vrep.simx_opmode_blocking)
        if error:
            print("get_global_position ERROR")
        print("POSITION: x="+str(position[0])+" y="+str(position[1])+" z="+str(position[2])+"\n")
        return position

    def forw_kin(self, thetas):
        e_s_theta = []
        for w, v, theta in zip(self.w,self.v,thetas):
            w_skew = np.array([[0,-w[2],w[1]], [w[2],0,-w[0]], [-w[1],w[0],0]])
            temp = np.eye(3) + np.sin(theta)*w_skew + (1-np.cos(theta))*(w_skew@w_skew)
            temp2 = (np.eye(3)*theta + (1-np.cos(theta))*w_skew + \
                    (theta-np.sin(theta))*(w_skew@w_skew))@v

            temp3 = np.zeros((4,4))
            temp3[0:3, 0:3] = temp
            temp3[0:3, 3] = temp2
            temp3[3, 3] = 1
            e_s_theta.append(temp3)

        answer = e_s_theta[0]
        for i in range(1, len(e_s_theta)):
            answer = answer@e_s_theta[i]
        answer = answer@self.M
        return answer

    def SetJointPosition(self, thetas):
        for arm, theta in zip(self.arms, thetas):
            vrep.simxSetJointPosition(self.clientID, arm, theta, vrep.simx_opmode_oneshot)
