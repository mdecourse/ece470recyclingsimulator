import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm, logm

# ======================================= Helper Functions ============================================== #

class robot_lidar:
    """ Class for the lidar and proximity sensor. """

    def __init__(self, clientID, prox_sensor, lidar_motor, noise_stdev = 0.00):
        self.clientID = clientID
        self.prox_sensor = prox_sensor
        self.lidar_motor = lidar_motor
        self.noise_stdev = noise_stdev
        
        vrep.simxGetJointPosition(self.clientID, self.lidar_motor, vrep.simx_opmode_streaming)
        vrep.simxReadProximitySensor(self.clientID, self.prox_sensor, vrep.simx_opmode_streaming)
    
    def set_lidar_velocity(self, v):
        """ Set angular velocity of lidar joint, in rad/sec. """
        vrep.simxSetJointTargetVelocity(self.clientID, self.lidar_motor, v, vrep.simx_opmode_oneshot)
        
    def get_lidar_angle(self):
        """ Get angle of lidar joint, in radians. """
        result, theta = vrep.simxGetJointPosition(self.clientID, self.lidar_motor, vrep.simx_opmode_buffer)
        if result != vrep.simx_return_ok:
            raise Exception('could not get lidar angle')
        return theta
    
    def get_lidar_raw(self):
        """ Gets the lidar distance measurement directly. """
        result = vrep.simxReadProximitySensor(self.clientID, self.prox_sensor, vrep.simx_opmode_buffer)
        if result[1]:
            depth = result[2][2] * np.random.normal(loc=1.0, scale=self.noise_stdev)
            return depth
        return None
    
    def read_lidar_point(self):
        """ Gets an (x, y) of a point the lidar detected, relative to its own center. """
        # 1: True if detected, False if not.
        # 2: 3D vector containing position of detected point.
        # 3: Object handle of the detected object.
        # 4: Surface normal vector of the detected point.
        result = self.get_lidar_raw()
        if result:
            angle = self.get_lidar_angle()
            return (result * np.cos(angle), result * np.sin(angle))
        return None
        