import numpy as np
from numba import njit

from . import geometry

@njit
def transition_jac(
    vel,
    rot_vel,
    q_center,
    delta_t: float,
):
    '''
    Jacobian of the transition function of the 12D Extended Kalman filter

    rot_vel (np.array of shape [3]): rotational velocity
    '''
    # Create an empty matrix
    F = np.eye(10)
    rot_mat = geometry.quat_as_matrix(q_center)
    # Translation derivatives
    # dx/dv
    F[:3, 3] = rot_mat[:, 0] * delta_t

    # Find the rotation matrix from w
    rot_q = rot_vel_to_q(rot_vel, delta_t)
    rot_mat_w = geometry.quat_as_matrix(rot_q)

    vel_cross = geometry.vector_to_pseudo_matrix(np.array([vel, 0, 0]))
    # dx/de
    F[:3, 4::2] = -rot_mat @ vel_cross * delta_t

    # Rotation derivatives
    # de/dw
    F[4, 5] = delta_t
    F[6, 7] = delta_t
    F[8, 9] = delta_t
    # de/de
    F[4::2, 4::2] = rot_mat_w.T

    return F

@njit
def rot_vel_to_q(rot_vel, delta_t):
    '''
    input:
        rot_vel (np.array of shape (3)): rotational velocity
        delta_t (float): time step
    output:
        quaternion of the rotation
    '''
    rot_vel_length = np.sqrt((rot_vel**2).sum())
    rot_angle_05 = rot_vel_length * delta_t * 0.5
    w_unit = rot_vel / (rot_vel_length + 1e-12)
    rot_q = np.empty(4)
    rot_q[0] = np.cos(rot_angle_05)
    rot_q[1:] = w_unit * np.sin(rot_angle_05)
    return rot_q