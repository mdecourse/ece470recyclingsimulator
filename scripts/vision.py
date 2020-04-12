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

        self.avg_red = 0.389
        self.max_red = 0.891

    def read_sensor(self):
        returnCode, detectionState, auxPackets = \
            vrep.simxReadVisionSensor(self.clientID, self.sensor, vrep.simx_opmode_buffer)
        if returnCode == 0 and len(auxPackets) > 0:
            # print(auxPackets)
            avg_red = auxPackets[0][11]
            # print("avg red color: " + str(avg_red))
            # avg_green = auxPackets[0][12]
            # avg_blue = auxPackets[0][13]
            # avg_red = avg_red / (avg_red + avg_green + avg_blue)
            # max_red = auxPackets[0][6]
            if avg_red > 0.39:
                print("avg is BIG RED")
            # if max_red > self.max_red:
            #     print("max red color: " + str(max_red))
        returnCode, resolution, image = \
            vrep.simxGetVisionSensorImage(self.clientID, self.sensor, 0, vrep.simx_opmode_buffer)
        if len(resolution) > 0:
            for x in range(0, resolution[0]*3, 3):
                for y in range(0, resolution[1]*3, 3):
                    red = image[x*resolution[0]+y]& 0b000011111111
                    # print(red)
                    if red > 250:
                        print("yay red")
                    # print(image[x*resolution[0]+y])
                    # print(image[x*resolution[0]+y]& 0b000011111111)
