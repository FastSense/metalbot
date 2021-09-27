import numpy as np
from numba import njit


@njit
def quat_product(q1, q2):
    res = np.empty(4)
    res[:3] = q1[3] * q2[:3] + q2[3] * q1[:3] + np.cross(q1[:3], q2[:3])
    res[3] = q1[3] * q2[3] - np.dot(q1[:3], q2[:3])
    return res

@njit
def quat_inv(q):
    q_inv = np.empty_like(q)
    q_inv[:3] = -q[:3]
    q_inv[3] = q[3]
    return q_inv

@njit
def quat_as_matrix(q):
    '''
    Transforms a quaternion to a rotation matrix
    '''
    mat = np.empty(shape=(3,3))
    mat[0, 0] = 1 - 2 * q[1]*q[1] - 2 * q[2] * q[2]
    mat[0, 1] = 2 * (q[0] * q[1] - q[2] * q[3])
    mat[0, 2] = 2 * (q[0] * q[2] + q[1] * q[3])
    mat[1, 0] = 2 * (q[0] * q[1] + q[2] * q[3])
    mat[1, 1] = 1 - 2 * q[0]*q[0] - 2 * q[2] * q[2]
    mat[1, 2] = 2 * (q[1] * q[2] - q[0] * q[3])
    mat[2, 0] = 2 * (q[0] * q[2] - q[1] * q[3])
    mat[2, 1] = 2 * (q[1] * q[2] + q[0] * q[3])
    mat[2, 2] = 1 - 2 * q[0]*q[0] - 2 * q[1] * q[1]
    return mat

@njit
def rotate_vector(p, q):
    p_quat = np.empty(4)
    p_quat[:3] = p
    p_quat[3] = 0
    res1 = quat_product(q, p_quat)
    q_inv = quat_inv(q)
    res2 = quat_product(res1, q_inv)
    return res2[:3]

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
    delta[:3] = scale * epsilon
    delta[3] = scale * 2
    # MRP:
    # scale = 1 / (16. + e_norm_squared)
    # delta[:3] = (8 * scale) * epsilon
    # delta[3] = (16. - e_norm_squared) * scale
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
    rot_q[:3] = w_unit * np.sin(rot_angle_05)
    rot_q[3] = np.cos(rot_angle_05)
    return rot_q
