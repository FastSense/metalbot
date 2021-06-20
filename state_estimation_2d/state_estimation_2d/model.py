import numpy as np
    
def transform_jacobian(x, dt):
    J_f = np.eye(6)
    # J_f[0, 2] = dt
    # J_f[1, 3] = dt
    # J_f[4, 5] = dt
    J_f[0, 2] = np.cos(x[4]) * dt
    J_f[1, 2] = np.sin(x[4]) * dt
    J_f[0, 4] = -x[2] * np.sin(x[4]) * dt
    J_f[1, 4] = x[2] * np.cos(x[4]) * dt
    J_f[4, 5] = dt
    return J_f