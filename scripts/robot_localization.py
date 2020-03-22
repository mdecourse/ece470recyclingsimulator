import support.vrep as vrep
import time
import numpy as np
import math
from scipy.linalg import expm, logm
from scripts.utils import *

# ======================================= Helper Functions ============================================== #

lidar_robot_pose = np.array([[1, 0, 0], [0, 1, -0.175], [0, 0, 1]])

class robot_state:
    def __init__(self, pose, lidar_angle, cache_lidar_readings=20):
        self.pose = pose
        self.lidar_angle = lidar_angle
        self.lidar_readings = []
        self.lidar_reading_max = cache_lidar_readings

class particle_filter:
    def __init__(self, nParticles, sensorModel, surroundings_map):
        self.nParticles = nParticles
        self.surroundings_map = surroundings_map
        self.particles = surroundings_map.generate_particles(nParticles)
    
    def iterate(self):
        

class known_map_AABB:
    """ Class for a known map (hard coded by a set of axis-aligned bounding boxes """
    
    def __init__(self, low_corner, high_corner, aabb_list):
        self.min = low_corner
        self.max = high_corner
        self.aabb_list = aabb_list
        self.cache_prev = None
    
    def sensor_model(self, pos, angle):
    
    def generate_particles(self, nParticles):
        particles = []
        for i in range(nParticles):
            angle = np.random.uniform(0.0, np.pi * 2, 1)
            x = np.random.uniform(self.min[0], self.max[0])
            y = np.random.uniform(self.min[1], self.max[1])
            particles.append(robot_state(pose2D((x, y), angle)), 0)
        return particles