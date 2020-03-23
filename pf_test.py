import numpy as np
import numpy.linalg as la
import math
import matplotlib.pyplot as plt
from scripts.robot_localization import *

pf = particle_filter(80, KNOWN_MAP)
true_position = robot_state(pose2D((3, 2), 0))

def plot_single(pf, pose):
    pos = pose[:2, -1]
    pointer = pose @ np.array([0, 0.1, 1])
    intersect_distance = pf.surroundings_map.sensor_model(pose, pf.lidar_angle)
    if intersect_distance == 5:
        intersect_distance = 0
    intersect_pos = axis2D_H(pf.lidar_angle) * intersect_distance
    intersect_pos[2] = 1
    intersection = pose @ intersect_pos
    plt.plot(pos[0], pos[1], "o")
    plt.plot(pointer[0], pointer[1], ".")
    plt.plot(intersection[0], intersection[1], ".")
    

def visualize_pf(pf, true_pos = None, n=0):
    points = []
    dirs = []
    intersections = []
    for state in pf.particles:
        intersect_distance = pf.surroundings_map.sensor_model(state.pose, pf.lidar_angle)
        if intersect_distance == 5:
            intersect_distance = 0
        intersect_pos = axis2D_H(pf.lidar_angle) * intersect_distance
        intersect_pos[2] = 1
        intersections.append(state.pose @ intersect_pos)
        points.append(state.pose[:2, -1])
        dirs.append(state.pose @ np.array([0, 0.1, 1]))
    
    points = np.array(points).T
    dirs = np.array(dirs).T
    intersections = np.array(intersections).T
    plt.figure(n)
    plt.clf()
    plt.plot(points[0], points[1], "o")
    plt.plot(dirs[0], dirs[1], ".")
    plt.plot(intersections[0], intersections[1], ".")
    
    if true_pos:
        plot_single(pf, true_pos.pose)
    
    predicted_pose = pf.get_predicted_pose()
    if predicted_pose is not None:
        plot_single(pf, predicted_pose)
    
    plt.xlim(-0.15, 4.15)
    plt.ylim(-0.15, 3.15)
    pf.surroundings_map.draw()

pf.particles[-1] = true_position.perturb(0.01, 0.01)
visualize_pf(pf, true_position)
plt.ion()
plt.show()
plt.pause(0.001)

for i in range(600):
    # true_position.update(0, -0.05, 0.3, 0.05)
    true_position.update(0, 0, 0, 0.05)
    intersect_distance = pf.surroundings_map.sensor_model(true_position.pose, pf.lidar_angle)
    true_position.add_sensor_input(intersect_distance * np.random.normal(loc=1, scale=0.05))
    
    # pf.update(0, -0.05, 0.3, 6, 0.05)
    pf.update(0, 0, 0, 6, 0.05)
    # if True:
    if i % 30 == 29:
        pf.resample(true_position.past_readings)
        visualize_pf(pf, true_position)
        plt.ion()
        plt.show()
        plt.pause(0.001)
        # pf.repopulate()
        # visualize_pf(pf, true_position)
        # plt.show()
