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
    def __init__(self, nParticles, surroundings_map):
        self.nParticles = nParticles
        self.surroundings_map = surroundings_map
        self.particles = surroundings_map.generate_particles(nParticles)
    
    def iterate(self):
        pass

class known_map_AABB:
    """ Class for a known map (hard coded by a set of axis-aligned bounding boxes """
    
    def __init__(self, low_corner, high_corner, aabb_list):
        self.min = low_corner
        self.max = high_corner
        self.aabb_edges = [segments_from_aabb(aabb) for aabb in aabb_list]
        self.cache_prev = None
    
    def sensor_model(self, pos, angle):
        """ Get sensor reading (a point x, y), or None. No noise """
        ray = axis2D(angle)
        if self.cache_prev:
            intersection = ray_segment_intersection(pos, ray, self.cache_prev[0], self.cache_prev[1])
            if intersection:
                return intersection
        for segments in self.aabb_edges:
            for start, end in segments:
                intersection = ray_segment_intersection(pos, ray, start, end)
                if intersection:
                    self.cache_prev = intersection
                    return intersection
        self.cache_prev = None
        return None
    
    def generate_particles(self, nParticles):
        particles = []
        for i in range(nParticles):
            angle = np.random.uniform(0.0, np.pi * 2, 1)
            x = np.random.uniform(self.min[0], self.max[0])
            y = np.random.uniform(self.min[1], self.max[1])
            particles.append(robot_state(pose2D((x, y), angle), 0))
        return particles


KNOWN_MAP = known_map_AABB((0, 0), (4, 3), 
                           [[(0, -0.1), (4, 0)], 
                            [(-0.1, 0), (0, 3)], 
                            [(0, 3), (4, 3.1)], 
                            [(4, 0), (4.1, 3)], 
                            [(1.95, 1), (2.05, 3)], 
                            [(2.95, 0), (3.05, 1)]])