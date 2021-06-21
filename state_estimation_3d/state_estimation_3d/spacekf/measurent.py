import numpy as np
from numba import njit

from . import geometry, physics


@njit
def acc_local15(state, q_center, gravity, extrinsic=None):
    pass
    # TODO
    #return z_prior, H

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
        rot_extrinsic = np.ascontiguousarray(extrinsic[:3,:3])
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
def odometry_jac12(state, q_center, dt_btw_frames, extrinsic=None):
    # Compose H
    H = np.zeros((6, 12))

    return H

@njit
def odometry_jac15(state, q_center, dt_btw_frames, dt_since_last_frame, extrinsic=None):
    # Go back in time
    state_1, q_1 = physics.transition_function15(
        state,
        q_center,
        -(dt_btw_frames + dt_since_last_frame),
    )
    state_2, _ = physics.transition_function15(
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
