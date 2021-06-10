import numpy as np

def get_jacobian_odom():
    J_odom = np.zeros((6, 8))
    J_odom[0, 0] = 1
    J_odom[1, 1] = 1
    J_odom[2, 2] = 1
    J_odom[3, 3] = 1
    J_odom[4, 6] = 1
    J_odom[5, 7] = 1
    return J_odom

def get_jacobian_imu():
    J_imu = np.zeros((1, 8))
    J_imu[0, 7] = 1
    return J_imu