import time
import numpy as np
import numpy.linalg as la
import math
from scipy.linalg import expm, logm

def rot2D(theta):
    """ 2D Rotation matrix. """
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    
def rot2D_H(theta):
    """ 2D Rotation matrix. (homogenous transform) """
    return np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    
def pose2D(pos, theta):
    """ 2D translation and rotation matrix. (homogenous transform) """
    return np.array([[np.cos(theta), -np.sin(theta), pos[0]], [np.sin(theta), np.cos(theta), pos[1]], [0, 0, 1]])
    
def rotToAngle2D(rotm):
    """ Get rotation angle from a 2D rotation matrix. """
    return math.atan2(rotm[1, 0], rotm[0, 0])
    
def cross_prod_matrix(axis):
    """ 3D skew-symmetric (cross-product) matrix. """
    return np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    
def rot3D(axis, angle):
    """ 3D Rotation matrix. """
    return expm(cross_prod_matrix(axis) * angle)
    
def rot3D(axis, angle):
    """ 3D Rotation matrix. (homogenous transform) """
    retval = np.zeros((4, 4))
    retval[:3, :3] = expm(cross_prod_matrix(axis) * angle)
    retval[3, 3] = 1
    return retval

def ray_segment_intersection(start, ray, segment_a, segment_b):
    # This is intentionally the negative of the ray from a to b - to avoid having to take negative later
    ray_2 = segment_a - segment_b
    # "Times" - for intersect, the first time (time of ray) should be greater than zero, and
    #   second time (time of segment) should be between 0 and 1.
    intersect_t = la.solve(np.array([ray, ray_2]).T, segment_a - start)
    if intersect_t[0] >= 0 and (intersect_t[1] >= 0 and intersect_t[1] <= 1):
        return 
    return None
    
def segments_from_aabb(aabb):
    aabb_min = aabb[0]
    aabb_max = aabb[1]
    
def quaternion_to_rotation(quaternion_vec):
    