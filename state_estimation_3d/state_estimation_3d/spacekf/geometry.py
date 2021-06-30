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
def quat_as_matrix(q):
    '''
    Transforms a quaternion to a rotation matrix
    '''
    mat = np.empty(shape=(3,3))
    mat[0, 0] = 1 - 2 * q[2]*q[2] - 2 * q[3] * q[3]
    mat[0, 1] = 2 * (q[1] * q[2] - q[3] * q[0])
    mat[0, 2] = 2 * (q[1] * q[3] + q[2] * q[0])
    mat[1, 0] = 2 * (q[1] * q[2] + q[3] * q[0])
    mat[1, 1] = 1 - 2 * q[1]*q[1] - 2 * q[3] * q[3]
    mat[1, 2] = 2 * (q[2] * q[3] - q[1] * q[0])
    mat[2, 0] = 2 * (q[1] * q[3] - q[2] * q[0])
    mat[2, 1] = 2 * (q[2] * q[3] + q[1] * q[0])
    mat[2, 2] = 1 - 2 * q[1]*q[1] - 2 * q[2] * q[2]
    return mat

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

@njit
def chart(q, q0=None):
    '''
    Chart function phi.
    A homeomorphism which maps the quaternion manifold to a 3d Euclidian space.
    Formula taken from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6339217/ .
    Representation used is MRP (Modified Rodrigues Parameters).
    '''
    if q0 is not None:
        delta = quat_product(quat_inv(q0), q)
    else:
        delta = q
    # RP:
    epsilon = delta[1:] * (2 / (delta[0] + 1e-12))
    # MRP:
    # epsilon = delta[1:] * (4 / (1 + delta[0] + 1e-12))
    return epsilon

@njit
def chart_inv(epsilon, q0=None):
    '''
    Inverse of the phi function.
    '''
    delta = np.empty(4)
    e_norm_squared = (epsilon**2).sum()
    # RP:
    scale = 1 / np.sqrt(4. + e_norm_squared)
    delta[0] = scale * 2
    delta[1:] = scale * epsilon
    # MRP:
    # scale = 1 / (16. + e_norm_squared)
    # delta[0] = (16. - e_norm_squared) * scale
    # delta[1:] = (8 * scale) * epsilon
    if q0 is not None:
        q = quat_product(q0, delta)
    else:
        q = delta
    return q

@njit
def reset_manifold(epsilon, q_center):
    # Rotate the center
    next_center = chart_inv(epsilon, q_center)
    # Clear epsilon
    next_epsilon = np.zeros_like(epsilon)
    return next_epsilon, next_center