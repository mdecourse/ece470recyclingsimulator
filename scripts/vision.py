import support.sim as vrep
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
        vrep.simxGetJointPosition(self.clientID, self.sensor, vrep.simx_opmode_streaming)
        vrep.simxReadVisionSensor(self.clientID, self.sensor, vrep.simx_opmode_streaming)
        vrep.simxGetVisionSensorImage(self.clientID, self.sensor, 0, vrep.simx_opmode_streaming)
        vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.sensor, vrep.simx_opmode_streaming)

        self.avg_red = 0.389
        self.max_red = 0.891

    def red_pixel_detection(self):
        # returnCode, detectionState, auxPackets = \
        #     vrep.simxReadVisionSensor(self.clientID, self.sensor, vrep.simx_opmode_buffer)
        returnCode, resolution, image = \
            vrep.simxGetVisionSensorImage(self.clientID, self.sensor, 0, vrep.simx_opmode_buffer)
        returnCode, resolution, depth = \
            vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.sensor, vrep.simx_opmode_buffer)
        num_pixels = 0
        avg_distance = 0
        avg_angle = 0
        #neg_angle = 0
        any_red = False
        if len(resolution) > 0:
            closest_distance = 9999
            for x in range(0, resolution[0]):
                for y in range(0, resolution[1]):
                    red = image[(x*resolution[0]+y)*3]& 0b000011111111
                    if red > 250:
                        closest_distance = min(closest_distance, depth[x*resolution[0]+y])
            # print(resolution) # 64 x 64
            # center = (32,32) ish
            for x in range(0, resolution[0]):
                for y in range(0, resolution[1]):
                    red = image[(x*resolution[0]+y)*3]& 0b000011111111
                    if red > 250:
                        distance = depth[x*resolution[0]+y]
                        if abs(distance - closest_distance) / closest_distance < 0.8: # 20% distance tolerance
                            num_pixels += 1
                            avg_distance += depth[x*resolution[0]+y]
                            # whole thing is 60 degrees across
                            avg_angle += 60 * (32-y)/64
                            # positive right
                            # negative left
                            #if 32 - y >= 0:
                            #    neg_angle += 1
                            #else:
                            #    neg_angle -= 1
        if num_pixels > 0:
            any_red = True
            avg_distance /= num_pixels
            avg_angle /= num_pixels
            #if neg_angle < 0:
            #    avg_angle = -avg_angle
            # print("red: ", num_pixels, " dist: ", avg_distance, " angle: ", avg_angle)
        return avg_distance, avg_angle, any_red
