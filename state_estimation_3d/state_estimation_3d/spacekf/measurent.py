import numpy as np
from numba import njit

from . import geometry, physics


@njit
def acc_local(state, q_center, gravity, extrinsic=None):
    # Get acceleration
    acc = state[2:9:3] - gravity
    # Get rotation matrix
    rot_matrix_inv = geometry.quat_as_matrix(q_center).T
    if extrinsic is not None:
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3,:3])
        rot_matrix_inv = rot_extrinsic @ rot_matrix_inv
    # Rotate the vector backwards
    q_inv = geometry.quat_inv(q_center)
    acc_lin = geometry.rotate_vector(acc, q_inv)
    if extrinsic is None:
        z_prior = acc_lin
    else:
        # Add rotational acceleration
        w = state[10::2]
        rxw = np.cross(extrinsic[:3,3], w)
        acc_rot = np.cross(rxw, w)
        z_prior = acc_lin# + acc_rot
    
    # Compose H
    H = np.zeros((3,15))
    H[:, 2:9:3] = rot_matrix_inv
    H[:, 9::2] = geometry.vector_to_pseudo_matrix(acc)
    # Add rotational acceleration
    # if extrinsic is not None:
    #     H[:, 10::2] = geometry.vector_to_pseudo_matrix(rxw)
    return z_prior, H

@njit
def rot_vel_local(state, extrinsic=None):
    w = state[10::2]
    if extrinsic is None:
        z_prior = w
    else:
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3,:3])
        z_prior = rot_extrinsic @ np.ascontiguousarray(w)

    # Compose H
    H = np.zeros((3,15))
    if extrinsic is None:
        H[0, 10] = 1
        H[1, 12] = 1
        H[2, 14] = 1
    else:
        H[:, 10::2] = rot_extrinsic
    return z_prior, H

@njit
def static_vec(q_center, vec):
    # Rotate the vector backwards
    q_inv = geometry.quat_inv(q_center)
    return geometry.rotate_vector(vec, q_inv)
@njit
def static_vec_jac(q_center, vec):
    vec_cross = geometry.vector_to_pseudo_matrix(vec)
    # Compose H
    H = np.zeros((3,15))
    H[:, 9::2] = vec_cross
    return H

@njit
def odometry_jac(state, q_center, dt_btw_frames, dt_since_last_frame, extrinsic=None):
    # Go back in time
    state_1, q_1 = physics.transition_function(
        state,
        q_center,
        -(dt_btw_frames + dt_since_last_frame),
    )
    state_2, _ = physics.transition_function(
        state,
        q_center,
        -dt_since_last_frame,
    )
    q_1_inv = geometry.quat_inv(q_1)
    # Compute positions difference
    translation = state_2[0:7:3] - state_1[0:7:3]
    translation_loc = geometry.rotate_vector(translation, q_1_inv)
    if extrinsic is not None:
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3,:3])
        translation_loc = rot_extrinsic @ translation_loc

    # Compose H
    H = np.zeros((6,15))
    translation_cross = geometry.vector_to_pseudo_matrix(translation_loc)
    rot_matrix_inv = geometry.quat_as_matrix(q_1_inv)
    if extrinsic is not None:
        rot_matrix_inv = rot_extrinsic @ rot_matrix_inv
    # Rotation
    H[0, 10] = dt_btw_frames
    H[1, 12] = dt_btw_frames
    H[2, 14] = dt_btw_frames
    # Translation
    H[3:, 1:8:3] = rot_matrix_inv * dt_btw_frames
    H[3:, 2:9:3] = rot_matrix_inv * (0.5 * (dt_btw_frames**2 - dt_since_last_frame**2))
    H[3:, 9::2] = translation_cross
    if extrinsic is not None:
        extrinsic_cross = extrinsic[:3,3]
        H[3:, 10::2] = extrinsic_cross.T * dt_btw_frames

    return H
