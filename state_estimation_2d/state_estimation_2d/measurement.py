import numpy as np

def get_jacobian_odom(x):
    H = np.zeros((3, 6))
    # J_odom[0, 2] = 1 / (np.cos(x[4]) + 0.00001)
    # H[0, 2] = np.cos(x[4])
    # H[0, 3] = np.sin(x[4])
    # H[0, 4] = -x[2] * np.sin(x[4]) + x[3] * np.cos(x[4])
    # H[1, 2] = -np.sin(x[4])
    # H[1, 3] =  np.cos(x[4])
    # H[1, 4] = -x[2] * np.cos(x[4]) - x[3] * np.sin(x[4])
    H[0, 2] = 1 
    H[1, 3] = 1
    H[2, 5] = 1  


    # H[0, 2] = np.cos(x[4])
    # H[0, 4] = -x[2] * np.sin(x[4])
    # H[1, 3] = np.sin(x[4])
    # H[1, 4] = x[3] * np.cos(x[4])
    # J_odom[0, 4] = -x[2] * np.sin(x[4]) + x[3] * np.cos(x[4])
    # H[0, 4] = - 2 * x[2] * np.sin(x[4]) * np.cos(x[4]) + x[3] * np.cos(x[4]) * np.cos(x[4]) - x[3] * np.sin(x[4]) * np.sin(x[4])
    # J_odom[1, 3] = 1
    # J_odom[0, 3] = 1 / (np.sin(x[4]) + 0.00001)
    # J_odom[0, 3] = np.sin(x[4])
    # J_odom[2, 5] = 1
    H[2, 5] = 1
    return H

def odom(x):
    z = np.zeros(3)
    # z[0] = x[2] * np.cos(x[4])
    # z[1] = x[3] * np.sin(x[4])
    # z[2] = x[5]

    z[0] = x[2] * np.cos(x[4]) + x[3] * np.sin(x[4])
    z[1] = -x[2] * np.sin(x[4]) + x[3] * np.cos(x[4])
    z[2] = x[5]
    # z[0] = x[2] * np.cos(x[4]) + x[3] * np.sin(x[4])
    # z[1] = -x[2] * np.sin(x[4]) + x[3] * np.cos(x[4])
    # z[2] = x[5] 
    # print("cos:")
    # print(np.cos(x[4]))
    # print("sin:")
    # print(np.sin(x[4]))
    return z

def get_jacobian_imu(x):
    H = np.zeros((2, 6))
    H[0, 2] = -x[5]
    H[0, 5] = -x[2]
    H[1, 5] = 1
    return H

def imu(x):
    z = np.zeros(2)
    z[0] = -x[2] * x[5]
    z[1] = x[5]
    return z