import numpy as np
from numba import njit

from . import geometry


@njit
def transition_function12(
    state,
    q_center,
    delta_t: float,
):
    '''
    Transition function of the 12D Kalman filter
    '''
    # Get state variables (all are 3-vectors)
    pos = state[0:5:2]
    vel = state[1:6:2]
    epsilon = state[6::2]
    rot_vel = state[7::2]

    # Integrate translation
    next_pos = pos + delta_t * vel
    next_vel = vel

    # Find the rotation quaternion from w
    rot_q = rot_vel_to_q(rot_vel, delta_t)

    # Rotate the current attitude
    next_q_center = geometry.quat_product(q_center, rot_q)
    next_q_center = next_q_center / np.sqrt(np.sum(next_q_center**2))

    # Quaternion will rotate, but epsilon will stay zero
    next_epsilon = epsilon
    # Rotation speed does not change
    next_rot_vel = rot_vel

    # Combine the next state
    next_state = np.empty(12)
    next_state[0:5:2] = next_pos
    next_state[1:6:2] = next_vel
    next_state[6::2] = next_epsilon
    next_state[7::2] = next_rot_vel

    return next_state, next_q_center


@njit
def transition_function15(
    state,
    q_center,
    delta_t: float,
):
    '''
    Transition function of the 15D Kalman filter
    '''
    # Get state variables (all are 3-vectors)
    pos = state[0:7:3]
    vel = state[1:8:3]
    acc = state[2:9:3]
    epsilon = state[9::2]
    rot_vel = state[10::2]

    # Integrate translation
    next_pos = pos + delta_t * vel + (0.5 * delta_t * delta_t) * acc
    next_vel = vel + delta_t * acc
    next_acc = acc

    # Find the rotation quaternion from w
    rot_q = rot_vel_to_q(rot_vel, delta_t)

    # Rotate the current attitude
    next_q_center = geometry.quat_product(q_center, rot_q)
    next_q_center = next_q_center / np.sqrt(np.sum(next_q_center**2))

    # Quaternion will rotate, but epsilon will stay zero
    next_epsilon = epsilon
    # Rotation speed does not change
    next_rot_vel = rot_vel

    # Combine the next state
    next_state = np.empty(15)
    next_state[0:7:3] = next_pos
    next_state[1:8:3] = next_vel
    next_state[2:9:3] = next_acc
    next_state[9::2] = next_epsilon
    next_state[10::2] = next_rot_vel

    return next_state, next_q_center


@njit
def transition_jac12(
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
    F[0, 1] = delta_t
    F[2, 3] = delta_t
    F[4, 5] = delta_t

    # Find the rotation matrix from w
    rot_q = rot_vel_to_q(rot_vel, delta_t)
    rot_mat_w = geometry.quat_as_matrix(rot_q)

    # Rotation derivatives
    # de/dw
    F[6, 7] = delta_t
    F[8, 9] = delta_t
    F[10, 11] = delta_t
    # de/de
    F[6::2, 6::2] = rot_mat_w.T

    return F


@njit
def transition_jac15(
    rot_vel,
    delta_t: float,
):
    '''
    Jacobian of the transition function of the 15D Extended Kalman filter

    rot_vel (np.array of shape [3]): rotational velocity
    '''
    # Create an empty matrix
    F = np.eye(15)

    # Translation derivatives
    # dx/dv
    F[0, 1] = delta_t
    F[3, 4] = delta_t
    F[6, 7] = delta_t
    # dx/da
    dxda = 0.5 * delta_t**2
    F[0, 2] = dxda
    F[3, 5] = dxda
    F[6, 8] = dxda
    # dv / da
    F[1, 2] = delta_t
    F[4, 5] = delta_t
    F[7, 8] = delta_t

    # Find the rotation matrix from w
    rot_q = rot_vel_to_q(rot_vel, delta_t)
    rot_mat_w = geometry.quat_as_matrix(rot_q)

    # Rotation derivatives
    # de/dw
    F[9, 10] = delta_t
    F[11, 12] = delta_t
    F[13, 14] = delta_t
    # de/de
    F[9::2, 9::2] = rot_mat_w.T

    return F
