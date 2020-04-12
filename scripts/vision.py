import support.vrep as vrep
import time
import numpy as np
import numpy.linalg as la
import math
from scipy.linalg import expm,logm
from modern_robotics import IKinSpace

class vision_sensor:
    """ Class for reading data from vision sensor. """

    def __init__(self, clientID, sensor):
        """ Initialize a motion object. """
        self.clientID = clientID
        self.sensor = sensor
        vrep.simxReadVisionSensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
        vrep.simxGetVisionSensorImage(self.clientID, self.sensor, 0, vrep.simx_opmode_streaming)

    def read_sensor(self):
        for i in range(5):
            returnCode, detectionState, auxPackets = \
                vrep.simxReadVisionSensor(self.clientID, self.sensor, vrep.simx_opmode_buffer)
            print(returnCode)
            print(detectionState)
            print(auxPackets)
            returnCode, resolution, image = \
                vrep.simxGetVisionSensorImage(self.clientID, self.sensor, 0, vrep.simx_opmode_buffer)
            print(returnCode)
            print(resolution)
            print(image)
