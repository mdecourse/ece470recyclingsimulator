import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from scipy.linalg import expm, logm
from scripts.utils import *

# ======================================= Helper Functions ============================================== #

"""
Hardcoded position of the lidar tower relative to the robot center.
"""
lidar_robot_pose = np.array([[1, 0, 0], [0, 1, -0.175], [0, 0, 1]])

"""
Range of the lidar detection beam.
"""
lidar_range = 2

"""
A class representing the robot's current state
and past few lidar readings. These are the particles.
"""
class robot_state:
    def __init__(self, pose, past_readings=[], max_readings=30):
        self.pose = pose
        self.past_readings = past_readings.copy()
        self.max_readings = max_readings
    
    def update(self, v_fb, v_rl, v_t, dt):
        new_pose_relative = pose2D((v_rl * dt, v_fb * dt), v_t * dt)
        self.pose = self.pose @ new_pose_relative
    
    
    def perturb(self, perturb_pos_stdev, perturb_angle_stdev):
        """ Returns a copy of this robot_state, displaced by a random amount in translation and rotation with the same sensor readings. """
        perturb_vec = axis2D_H(np.random.uniform(low=0.0, high=2*np.pi)) * np.random.normal(loc=perturb_pos_stdev, scale=perturb_pos_stdev)
        return robot_state(self.pose @ pose2D(perturb_vec, np.random.normal(loc=0.0, scale=perturb_angle_stdev)), self.past_readings, self.max_readings)

    def add_sensor_input(self, reading):
        if len(self.past_readings) == self.max_readings:
            self.past_readings.pop(0)
        self.past_readings.append(reading)

class particle_filter:
    def __init__(self, nParticles, surroundings_map, perturb_pos_stdev=0.05, perturb_angle_stdev = 0.15,random_fraction=10):
        self.nParticles = nParticles
        self.surroundings_map = surroundings_map
        self.lidar_angle = np.pi / 2
        self.particles = surroundings_map.generate_particles(nParticles)
        self.perturb_pos_stdev = perturb_pos_stdev
        self.perturb_angle_stdev = perturb_angle_stdev
        self.predicted_pose = None
        self.random_fraction = random_fraction
        self.resample_particles = nParticles
    
    def update(self, v_fb, v_rl, v_t, lidar_angle, dt):
        
        if self.predicted_pose is not None:
            predicted_pose_relative = pose2D((v_rl * dt, v_fb * dt), v_t * dt)
            self.predicted_pose = self.predicted_pose @ predicted_pose_relative
        
        self.lidar_angle = lidar_angle
        
        for particle in self.particles:
            particle.update(v_fb, v_rl, v_t, dt)
            particle_sensor = self.surroundings_map.sensor_model(particle.pose, self.lidar_angle)
            particle.add_sensor_input(particle_sensor)
        # print(weights)
    
    def resample(self, readings):
        """
        Resampling method: Take the top quarter of particles, add itself and three perturbed copies each
        except the bottom 1/8 become random particles.
        """
        
        self.predicted_pose = None
        
        measured_readings = np.array(readings)
        weights = []
        for particle in self.particles:
            particle_reading = np.array(particle.past_readings)
            error = particle_reading - measured_readings[:len(particle_reading)]
            if self.surroundings_map.feasible(particle):
                weights.append(np.exp(-la.norm(error, 1)**2))
            else:
                weights.append(0)
        weights = np.array(weights) / la.norm(weights, 1)
        paired = sorted(list(zip(weights, self.particles)), key=lambda x: x[0])
        # Resample
        newParticles = []
        for i in range((self.resample_particles // 4)):
            target_particle = paired[-1-i][1] # Between 0 and 1
            newParticles.append(target_particle)
            for j in range(4-1):
                newParticles.append(target_particle.perturb(self.perturb_pos_stdev, self.perturb_angle_stdev))
        self.nParticles = self.resample_particles
        self.particles = newParticles
        self.particles[-(self.nParticles // self.random_fraction):] = self.surroundings_map.generate_particles(self.nParticles // self.random_fraction)
        
    def get_predicted_pose(self):
        """
        Predicts pose using top 50% of particles averaged.
        """
        if self.predicted_pose is not None:
            return self.predicted_pose
        
        N = self.nParticles // 2
        
        direction_avg = np.zeros(2)
        position_avg = np.zeros(2)
        for i in range(N):
            particle = self.particles[i]
            position_avg += particle.pose[:2, -1]
            direction_avg += particle.pose[:2, 0] # Hack: these entries are cos(theta), sin(theta)
        
        direction_avg /= la.norm(direction_avg)
        position_avg /= N
        
        self.predicted_pose = np.array([[direction_avg[0], -direction_avg[1], position_avg[0]], 
                                        [direction_avg[1], direction_avg[0],  position_avg[1]], 
                                        [0,                0,                 1]])
        return self.predicted_pose
    # def repopulate(self):
        # N = len(self.particles)
        # N_skip = N // 4
        # for i in range(N - N_skip):
            # for j in range(4-1):
                # self.particles.append(self.particles[i].perturb(self.perturb_pos_stdev, self.perturb_angle_stdev))
        # self.particles.extend(self.surroundings_map.generate_particles(N_skip * (4-1), self.lidar_angle))

class known_map_AABB:
    """ Class for a known map (hard coded by a set of axis-aligned bounding boxes """
    
    def __init__(self, low_corner, high_corner, aabb_list):
        self.min = low_corner
        self.max = high_corner
        self.aabb_edges = [segments_from_aabb(aabb) for aabb in aabb_list]
    
    def sensor_model(self, pose, angle):
        """ Get sensor reading (a point x, y), or None. No noise """
        ray = (pose @ axis2D_H(angle))[:2]
        closest_intersection = None
        best_distance = 5
        lidar_position = (pose @ [0, -0.175, 0])[:2] + pose[:2, -1]
        for segments in self.aabb_edges:
            for start, end in segments:
                intersection, distance = ray_segment_intersection(lidar_position, ray, lidar_range, start, end)
                if intersection is not None:
                    if closest_intersection is None or distance < best_distance:
                        closest_intersection = intersection
                        best_distance = distance
        return best_distance
    
    def generate_particles(self, nParticles):
        particles = []
        for i in range(nParticles):
            angle = np.random.uniform(0.0, np.pi * 2, 1)
            x = np.random.uniform(self.min[0], self.max[0])
            y = np.random.uniform(self.min[1], self.max[1])
            particles.append(robot_state(pose2D((x, y), angle)))
        return particles

    def feasible(self, state):
        pos = state.pose[:2, -1]
        return pos[0] >= self.min[0] and pos[1] >= self.min[1] and pos[0] <= self.max[0] and pos[1] <= self.max[1]

    def draw(self):
        for segments in self.aabb_edges:
            lc = mc.LineCollection(segments)
            ax = plt.gca()
            ax.add_collection(lc)


KNOWN_MAP = known_map_AABB((0, 0), (4, 3), 
                           [[(0, -0.1), (4, 0)], 
                            [(-0.1, 0), (0, 3)], 
                            [(0, 3), (4, 3.1)], 
                            [(4, 0), (4.1, 3)], 
                            [(1.95, 1), (2.05, 3)], 
                            [(2.95, 0), (3.05, 1)]])