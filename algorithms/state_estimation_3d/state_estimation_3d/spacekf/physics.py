import numpy as np
from numba import njit

from . import geometry


@njit
def transition_function(
    pos, vel, epsilon, rot_vel,
    q_center,
    delta_t: float,
):
    '''
    Transition function of the 12D Kalman filter
    '''
    # Integrate translation
    next_pos = pos + delta_t * vel
    next_vel = vel

    # Find the rotation quaternion from w
    rot_q = geometry.rot_vel_to_q(rot_vel, delta_t)

    # Rotate the current attitude
    next_q_center = geometry.quat_product(q_center, rot_q)
    next_q_center = next_q_center / np.sqrt(np.sum(next_q_center**2))

    # Quaternion will rotate, but epsilon will stay zero
    next_epsilon = epsilon
    # Rotation speed does not change
    next_rot_vel = rot_vel

    return next_pos, next_vel, next_epsilon, next_rot_vel, next_q_center


@njit
def transition_jac(
    rot_vel,
    delta_t: float,
):
    '''
    Jacobian of the transition function of the 12D Extended Kalman filter

    rot_vel (np.array of shape [3]): rotational velocity
    '''
    # Create an empty matrix
    F = np.eye(12)

    # Translation derivatives
    # dx/dv
    F[0, 3] = delta_t
    F[1, 4] = delta_t
    F[2, 5] = delta_t

    # Find the rotation matrix from w
    rot_q = geometry.rot_vel_to_q(rot_vel, delta_t)
    rot_mat_w = geometry.quat_as_matrix(rot_q)

    # Rotation derivatives
    # de/dw
    F[6, 9] = delta_t
    F[7, 10] = delta_t
    F[8, 11] = delta_t
    # de/de
    F[6:9, 6:9] = rot_mat_w.T

    return F
