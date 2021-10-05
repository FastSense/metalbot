import numpy as np
import scipy
from numba import njit
from scipy.spatial.transform import Rotation
from filterpy.common import Q_discrete_white_noise

from . import physics, measurement
from .. import geometry


GRAVITY = np.array([0, 0, 9.8])


class SpaceKF12:
    '''
    12-dimensional Extended Kalman Filter

    State of this filter is called `x` and consists of:
    0. x
    1. x_vel
    2. y
    3. y_vel
    4. z
    5. z_vel
    6. e_x
    7. w_x
    8. e_y
    9. w_y
    10. e_z
    11. w_z

    Here e_xyz is the Euclidian representation of the delta from the center quaternion.
    w_xyz is the rotation speed.
    Additionaly, this filter stores `q` - the center quaternion around which the neighborhood e is built.
    More about this here: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6339217/
    Normally, `e_xyz==0` after the predict step. This means that `q` gives us the current attitude.

    You can get current state either directly from `x` and `q` attributes or by using these properties:
    `pos`, `vel`, `rot_vel`.
    You can get yaw, pitch, roll by using property `euler`.
    You can get rotation matrix by using property `rot_matrix`.
    '''

    DIM = 12
    ACCEL_STD = 5.0

    def __init__(
        self,
        dt,
        velocity_std,
        rot_vel_std,
    ):
        self.x = np.zeros(12)
        self.P = np.eye(12)
        # Idk why there are minuses , but it works:
        Q_e_w = np.array([
            [0.333 * dt**3, -0.5 * dt**2],
            [ -0.5 * dt**2,           dt],
        ]) * rot_vel_std**2
        self.Q = scipy.linalg.block_diag(
            Q_discrete_white_noise(dim=2, dt=dt, var=velocity_std**2, block_size=3),
            Q_e_w, Q_e_w, Q_e_w,
        )
        self.q = np.array([1., 0, 0, 0])
        self.dt = dt
        # Transition function
        self.transition_function = physics.transition_function12
        self.transition_jac = physics.transition_jac12

    def predict(self, dt=None):
        # Reset manifold before each predict
        self.reset_manifold()
        # Prediction step
        dt = dt or self.dt
        F = self.transition_jac(self.rot_vel, dt)
        self.x, self.q = self.transition_function(self.x, self.q, dt)
        self.P = F @ self.P @ F.T + self.Q

    def update_linear(self, H, z, R):
        y = z - H @ self.x
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_acc(self, z, R, gravity=None, extrinsic=None):
        '''
        Update state by a measurement coming from on-board accelerometer.
        '''
        R_new = R + np.eye(3) * self.ACCEL_STD**2
        self.update_static_vec(z, R_new, vec=gravity or GRAVITY, extrinsic=extrinsic)

    def update_rot_vel(self, z, R, extrinsic=None):
        '''
        Update state by a measurement coming from on-board gyroscope.
        '''
        z_prior, H = measurement.rot_vel_local(self.rot_vel, self.DIM, extrinsic=extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_static_vec(self, z, R, vec, extrinsic=None):
        '''
        Update state by a measurement of some vector which should be constant in the world coordinates.
        For example, magnetic field.
        '''
        z_prior, H = measurement.static_vec(self.q, vec, self.DIM, extrinsic=extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_odometry(self, z, R, delta_t, extrinsic=None):
        '''
        Update state by a visual odometry measurement coming from a neural network.

        Parameters
        ----------
            z (np.array of shape [6]): Output of a neural network. Contains rotations and translations: `[rot_x, rot_y, rot_z, dx, dy, dz]`.
        '''
        z_prior, H = measurement.odometry12(self.vel, self.rot_vel, self.q, delta_t, extrinsic=extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_flow(self, flows, delta_t, depths, pixels, R, camera_matrix, camera_matrix_inv, extrinsic=None):
        '''
        Update state by a visual odometry measurement coming from a neural network.

        Parameters
        ----------
            flows (np.array of shape [N, 3]): Optical flow measurements at N given points
            delta_t (float): time step
            depths (np.array of shape [N]): Depth at N given points
            pixels (np.array of shape [N, 2]): x and y coordinates of image points, in pixel scale
            R (np.array of shape [3, 3]): error of measurements
            camera_matrix (np.array of shape [3, 3]): camera intrinsic matrix
            extrinsic (None or np.array of shape [3, 4]): optional. Camera pose relative to filter
        '''
        z = flows.flatten()
        z_prior, H = measurement.flow_odom12(self.vel, self.rot_vel, self.q, delta_t, depths, pixels, camera_matrix, camera_matrix_inv, extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def reset_manifold(self):
        '''
        Reset the linearization center

        Called automatically before every predict step.
        '''
        self._epsilon, self.q = geometry.reset_manifold(self._epsilon, self.q)

    @property
    def pos(self):
        return self.x[0:5:2]
    @pos.setter
    def pos(self, value):
        self.x[0:5:2] = value

    @property
    def vel(self):
        return self.x[1:6:2]
    @vel.setter
    def vel(self, value):
        self.x[1:6:2] = value

    @property
    def _epsilon(self):
        return self.x[6::2]
    @_epsilon.setter
    def _epsilon(self, value):
        self.x[6::2] = value

    @property
    def rot_vel(self):
        return self.x[7::2]
    @rot_vel.setter
    def rot_vel(self, value):
        self.x[7::2] = value

    @property
    def euler(self):
        return Rotation.from_quat(self.q[[1,2,3,0]]).as_euler('xyz', degrees=False)

    @property
    def rot_matrix(self):
        return geometry.quat_as_matrix(self.q)

    def get_pose_covariance(self):
        '''
        Returns 6x6 covariance matrix of pose in the form a list with length 36:
        (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        '''
        indeces = [0, 2, 4, 6, 8, 10]
        mat = self.P[indeces][:, indeces]
        return list(mat.flatten())

    def get_twist_covariance(self):
        '''
        Returns 6x6 covariance matrix of velocity in the form a list with length 36:
        (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        '''
        indeces = [1, 3, 5, 7, 9, 11]
        mat = self.P[indeces][:, indeces]
        return list(mat.flatten())


class SpaceKF15(SpaceKF12):
    '''
    15-dimensional Extended Kalman Filter
    State of this filter is called `x` and consists of:
    0. x
    1. x_vel
    2. x_acc
    3. y
    4. y_vel
    5. y_acc
    6. z
    7. z_vel
    8. z_acc
    9. e_x
    10. w_x
    11. e_y
    12. w_y
    13. e_z
    14. w_z

    Here e_xyz is the Euclidian representation of the delta from the center quaternion.
    w_xyz is the rotation speed.
    Additionaly, this filter stores `q` - the center quaternion around which the neighborhood e is built.
    More about this here: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6339217/
    Normally, `e_xyz==0` after the predict step. This means that `q` gives us the current attitude.

    You can get current state either directly from `x` and `q` attributes or by using these properties:
    `pos`, `vel`, `acc`, `rot_vel`.
    You can get yaw, pitch, roll by using property `euler`.
    You can get rotation matrix by using property `rot_matrix`.
    '''

    DIM = 15

    def __init__(
        self,
        dt,
        velocity_std,
        rot_vel_std,
    ):
        self.x = np.zeros(15)
        self.P = np.eye(15)
        # Idk why there are minuses , but it works:
        Q_e_w = np.array([
            [0.333 * dt**3, -0.5 * dt**2],
            [ -0.5 * dt**2,           dt],
        ]) * rot_vel_std**2
        self.Q = scipy.linalg.block_diag(
            Q_discrete_white_noise(dim=3, dt=dt, var=velocity_std**2, block_size=3),
            Q_e_w, Q_e_w, Q_e_w,
        )
        self.q = np.array([1., 0, 0, 0])
        self.dt = dt
        # Transition function
        self.transition_function = physics.transition_function15
        self.transition_jac = physics.transition_jac15

    def update_acc(self, z, R, gravity=None, extrinsic=None):
        '''
        Update state by a measurement coming from on-board accelerometer.
        '''
        z_prior, H = measurement.acc_local15(self.x, self.q, gravity=gravity or GRAVITY, extrinsic=extrinsic)
        y = z - z_prior
        # q_mat = geometry.quat_as_matrix(self.q)
        # R_glob = q_mat.T @ R @ q_mat
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    @property
    def pos(self):
        return self.x[0:7:3]
    @pos.setter
    def pos(self, value):
        self.x[0:7:3] = value

    @property
    def vel(self):
        return self.x[1:8:3]
    @vel.setter
    def vel(self, value):
        self.x[1:8:3] = value

    @property
    def acc(self):
        return self.x[2:9:3]
    @acc.setter
    def acc(self, value):
        self.x[2:9:3] = value

    @property
    def _epsilon(self):
        return self.x[9::2]
    @_epsilon.setter
    def _epsilon(self, value):
        self.x[9::2] = value

    @property
    def rot_vel(self):
        return self.x[10::2]
    @rot_vel.setter
    def rot_vel(self, value):
        self.x[10::2] = value


@njit
def kalman_update(x, P, H, R, y):
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    new_x = x + K @ y
    new_P = (np.eye(x.shape[0]) - K @ H) @ P
    return new_x, new_P