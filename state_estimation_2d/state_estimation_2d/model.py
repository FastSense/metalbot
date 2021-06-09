import numpy as np
    
def transform_jacobian(dt):
    J_f = np.eye(8)
    J_f[0, 2] = dt
    J_f[1, 3] = dt
    J_f[2, 4] = dt
    J_f[3, 5] = dt
    J_f[6, 7] = dt
    J_f[0, 4] = 0.5 * dt * dt
    J_f[1, 5] = 0.5 * dt * dt
    return J_f