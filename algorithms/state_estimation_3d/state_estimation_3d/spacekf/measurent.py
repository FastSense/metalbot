import numpy as np
from numba import njit

from . import geometry, physics


@njit
def rot_vel_local(rot_vel, dim, extrinsic=None):
    # Get rot_vel
    z_prior = rot_vel

    # Extrinsic transform
    if extrinsic is not None:
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3,:3])
        z_prior = rot_extrinsic @ np.ascontiguousarray(z_prior)

    # Compose H
    H = np.zeros((3, dim))
    if extrinsic is None:
        H[0, dim - 5] = 1
        H[1, dim - 3] = 1
        H[2, dim - 1] = 1
    else:
        H[:, dim - 5::2] = rot_extrinsic
    return z_prior, H

@njit
def static_vec(q_center, vec, dim, extrinsic=None):
    # Rotate the vector backwards
    q_inv = geometry.quat_inv(q_center)
    z_prior = geometry.rotate_vector(vec, q_inv)

    # Extrinsic transform
    if extrinsic is not None:
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3,:3])  # TODO: maybe .T ???
        z_prior = rot_extrinsic @ np.ascontiguousarray(z_prior)

    # Compose H
    H = np.zeros((3, dim))
    vec_cross = geometry.vector_to_pseudo_matrix(z_prior)
    if extrinsic is None:
        H[:, dim - 6::2] = vec_cross
    else:
        H[:, dim - 6::2] = vec_cross @ rot_extrinsic
    return z_prior, H

@njit
def flow_odom12(vel, rot_vel, q_center, delta_t, depths, pixels, camera_matrix, camera_matrix_inv, extrinsic=None):
    z_full = np.empty(3 * len(depths))
    H_full = np.empty((3 * len(depths), 12))

    for i in range(len(depths)):
        depth = depths[i]
        pixel = pixels[i]
        z, H = _flow_odom12_single(vel, rot_vel, q_center, delta_t, depth, pixel, camera_matrix, camera_matrix_inv, extrinsic)
        z_full[i * 3: (i + 1) * 3] = z
        H_full[i * 3: (i + 1) * 3] = H

    return np.ascontiguousarray(z_full), np.ascontiguousarray(H_full)

@njit
def _flow_odom12_single(vel, rot_vel, q_center, delta_t, depth, pixel, camera_matrix, camera_matrix_inv, extrinsic=None):
    '''
    Function for single measurement

    depth (float): depth
    pixel (int array [2]): pixel coordinates
    camera_matrix (float array [3, 3]): intrinsic parameters
    extrinsic (float array [3, 4]): extrinsic parameters
    '''
    # Target hyperbolic coordinates
    p = np.empty(3)
    p[:2] = pixel
    p[2] = 1
    target_m = camera_matrix_inv @ np.ascontiguousarray(p) # [3]
    # Target point coordinates
    target_x = target_m * depth # x,y,z shape: [3]

    # Rotate the vector backwards
    q_inv = geometry.quat_inv(q_center)
    rot_matrix_inv = geometry.quat_as_matrix(q_inv)

    # Extrinsic transform
    vel = np.ascontiguousarray(vel)
    rot_vel = np.ascontiguousarray(rot_vel)
    if extrinsic is not None:
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3, :3])
    else:
        rot_extrinsic = np.eye(3)

    # 3: Velocity of a point in camera coordinates
    vel_local = geometry.rotate_vector(vel, q_inv)
    point_vel = -rot_extrinsic @ vel_local + np.cross(target_x, rot_extrinsic @ rot_vel)
    # 3: Jacobian [3, 9]
    jacobian_3x9 = np.concatenate((
        -rot_extrinsic @ rot_matrix_inv,                                # velocity from velocity
        -rot_extrinsic @ geometry.vector_to_pseudo_matrix(vel_local),   # velocity from angle
        geometry.vector_to_pseudo_matrix(target_x) @ rot_extrinsic,     # velocity from rotation velocity
    ), 1)

    # 2: Source hyperbolic coordinates
    source_x = target_x - point_vel * delta_t
    source_m = source_x / source_x[2]
    # 2: Jacobian [2, 3]
    jacobian_2x3 = np.zeros((2, 3))
    jacobian_2x3[0, 0] = -delta_t / target_x[2] # TODO: check if target_x is better
    jacobian_2x3[1, 1] = jacobian_2x3[0, 0]
    jacobian_2x3[0, 2] = delta_t * target_x[0] / target_x[2]**2
    jacobian_2x3[1, 2] = delta_t * target_x[1] / target_x[2]**2

    # 1: Compute optical flow
    source_p = np.ascontiguousarray(camera_matrix[:2]) @ source_m
    z_prior = np.empty(3)
    z_prior[:2] = source_p - pixel
    z_prior[2] = -point_vel[2] * delta_t
    # 1: Jacobian [2, 2]
    jacobian_2x2 = np.ascontiguousarray(camera_matrix[:2, :2])

    # Compute the whole jacobian
    jac = jacobian_2x2 @ jacobian_2x3 @ jacobian_3x9 # [2, 9]
    H = np.zeros((3, 12))
    # Flow
    H[:2, 1:6:2] = jac[:, :3] # velocity
    H[:2, 6::2] = jac[:, 3:6] # angle
    H[:2, 7::2] = jac[:, 6:]  # rotation velocity
    # Delta depth
    H[2, 1:6:2] = -delta_t * jacobian_3x9[2, :3]
    H[2, 6::2] = -delta_t * jacobian_3x9[2, 3:6]
    H[2, 7::2] = -delta_t * jacobian_3x9[2, 6:]

    return z_prior, H
