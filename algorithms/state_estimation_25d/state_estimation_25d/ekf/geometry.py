import numpy as np
from numba import njit

@njit
def quat_product(q1, q2):
    res = np.empty(4)
    res[0] = q1[0] * q2[0] - np.dot(q1[1:], q2[1:])
    res[1:] = q1[0] * q2[1:] + q2[0] * q1[1:] + np.cross(q1[1:], q2[1:])
    return res

@njit
def quat_inv(q):
    q_inv = np.empty_like(q)
    q_inv[0] = q[0]
    q_inv[1:] = -q[1:]
    return q_inv

@njit
def chart_inv(epsilon, q0=None):
    delta = np.empty(4)
    e_norm_squared = (epsilon**2).sum()
    scale = 1 / np.sqrt(4. + e_norm_squared)
    delta[0] = scale * 2
    delta[1:] = scale * epsilon

    if q0 is not None:
        q = quat_product(q0, delta)
    else:
        q = delta
    return q

@njit
def rotate_vector(p, q):
    p_quat = np.empty(4)
    p_quat[0] = 0
    p_quat[1:] = p
    res1 = quat_product(q, p_quat)
    q_inv = quat_inv(q)
    res2 = quat_product(res1, q_inv)
    return res2[1:]

@njit
def reset_manifold(epsilon, q_center):
    next_center = chart_inv(epsilon, q_center)
    next_epsilon = np.zeros_like(epsilon)
    return next_epsilon, next_center

@njit
def vector_to_pseudo_matrix(vec):
    mat = np.empty(shape=(3,3))
    mat[0, 0] = 0
    mat[0, 1] = -vec[2]
    mat[0, 2] = vec[1]
    mat[1, 0] = vec[2]
    mat[1, 1] = 0
    mat[1, 2] = -vec[0]
    mat[2, 0] = -vec[1]
    mat[2, 1] = vec[0]
    mat[2, 2] = 0
    return mat