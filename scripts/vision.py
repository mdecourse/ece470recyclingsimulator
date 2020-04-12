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
        vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.sensor, vrep.simx_opmode_streaming)

        self.avg_red = 0.389
        self.max_red = 0.891

    def read_sensor(self):
        # returnCode, detectionState, auxPackets = \
        #     vrep.simxReadVisionSensor(self.clientID, self.sensor, vrep.simx_opmode_buffer)
        returnCode, resolution, image = \
            vrep.simxGetVisionSensorImage(self.clientID, self.sensor, 0, vrep.simx_opmode_buffer)
        returnCode, resolution, depth = \
            vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.sensor, vrep.simx_opmode_buffer)
        num_pixels = 0
        if len(resolution) > 0:
            for x in range(0, resolution[0]):
                for y in range(0, resolution[1]):
                    red = image[(x*resolution[0]+y)*3]& 0b000011111111
                    if red > 250:
                        num_pixels += 1
                        d = depth[x*resolution[0]+y]
                        # print("Cylinder is: "+str(d)+" meters away.")
                        # print("yay red")
            print("Number of red pixels: " + str(num_pixels))
