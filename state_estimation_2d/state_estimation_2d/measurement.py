import numpy as np

def get_jacobian_odom():
    J_odom = np.zeros((3, 8))
    J_odom[0, 2] = 1
    J_odom[1, 3] = 1
    J_odom[2, 7] = 1
    return J_odom

def get_jacobian_imu():
    J_imu = np.zeros((1, 8))
    J_imu[0, 7] = 1
    return J_imu