import numpy as np
from numba import njit

from . import geometry


@njit
def transition_function(
    state,
    q_center,
    delta_t: float,
):
    '''
    Transition function of the Kalman filter
    '''
    # Get state variables (all are 3-vectors)
    pos = state[0:7:3]
    vel = state[1:8:3]
    acc = state[2:9:3]
    epsilon = state[9::2]
    w = state[10::2]

    # Integrate translation
    next_pos = pos + delta_t * vel + (0.5 * delta_t * delta_t) * acc
    next_vel = vel + delta_t * acc
    next_acc = acc

    # Find the rotation quaternion from w
    w = state[10::2]
    w_length = np.sqrt((w**2).sum())
    rot_angle_05 = w_length * delta_t * 0.5
    w_unit = w / (w_length + 1e-12)
    rot_q = np.empty(4)
    rot_q[0] = np.cos(rot_angle_05)
    rot_q[1:] = w_unit * np.sin(rot_angle_05)

    # Rotate the current attitude
    next_q_center = geometry.quat_product(q_center, rot_q)
    next_q_center = next_q_center / np.sqrt(np.sum(next_q_center**2))

    # Quaternion will rotate, but epsilon will stay zero
    next_epsilon = epsilon
    # Rotation speed does not change
    next_w = w

    # Combine the next state
    next_state = np.empty(15)
    next_state[0:7:3] = next_pos
    next_state[1:8:3] = next_vel
    next_state[2:9:3] = next_acc
    next_state[9::2] = next_epsilon
    next_state[10::2] = next_w

    return next_state, next_q_center

@njit
def transition_jac(
    rot_vel,
    delta_t: float,
    order: int,
):
    '''
    Jacobian of the transition function of the Extended Kalman filter

    rot_vel (np.array of shape [3]): rotational velocity
    order: 1 or 2. If 2, state vector includes acceleration.
    '''
    # Create an empty matrix
    if order == 1:
        F = np.eye(12)
    elif order == 2:
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
    rot_vel_length = np.sqrt((rot_vel**2).sum())
    rot_angle_05 = rot_vel_length * delta_t * 0.5
    w_unit = rot_vel / (rot_vel_length + 1e-12)
    rot_q = np.empty(4)
    rot_q[0] = np.cos(rot_angle_05)
    rot_q[1:] = w_unit * np.sin(rot_angle_05)
    rot_mat_w = geometry.quat_as_matrix(rot_q)

    # Rotation derivatives
    # de/dw
    F[9, 10] = delta_t
    F[11, 12] = delta_t
    F[13, 14] = delta_t
    # de/de
    F[9::2, 9::2] = rot_mat_w.T

    return F


def rotation_vel_to_q(rot_vel, delta_t):
    rot_vel_length = np.sqrt((rot_vel**2).sum())
    rot_angle_05 = rot_vel_length * delta_t * 0.5
    w_unit = rot_vel / (rot_vel_length + 1e-12)
    rot_q = np.empty(4)
    rot_q[0] = np.cos(rot_angle_05)
    rot_q[1:] = w_unit * np.sin(rot_angle_05)
    return rot_q