import numpy as np
from numba import njit

from . import geometry, physics

@njit
def odometry(vel, w_yaw):#, rot_vel, q_center, delta_t, extrinsic=None):
    '''
    measurement = (angle about x, angle about y, angle about z, translation x, translation y, translation z)
    '''
    z_prior = np.array([vel, w_yaw])
    # Compose H
    H = np.zeros((2, 10))
    H[0, 3] = 1
    H[1, 9] = 1
    return z_prior, H

@njit
def static_vec(q_center, vec, extrinsic=None):
    # Rotate the vector backwards
    q_inv = geometry.quat_inv(q_center)
    z_prior = geometry.rotate_vector(vec, q_inv)
    # Extrinsic transform
    if extrinsic is not None:
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3,:3])
        z_prior = rot_extrinsic @ np.ascontiguousarray(z_prior)
    # Compose H
    H = np.zeros((3, 10))
    vec_cross = geometry.vector_to_pseudo_matrix(z_prior)
    if extrinsic is None:
        H[:, 4:7:1] = vec_cross
    else:
        H[:, 4:7:1] = vec_cross @ rot_extrinsic
    return z_prior, H

@njit
def rot_vel_local(w_yaw):
    # Get rot_vel
    z_prior = w_yaw
    # Compose H
    H = np.zeros((1, 10))
    H[0, 9] = 1
    return z_prior, H