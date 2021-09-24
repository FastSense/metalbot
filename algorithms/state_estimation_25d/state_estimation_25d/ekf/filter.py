import numpy as np
import scipy
from numba import njit
from scipy.spatial.transform import Rotation
from filterpy.common import Q_discrete_white_noise

from . import physics, geometry, measurement

class Filter:
    '''
    10-dimensional Extended Kalman Filter

    State of this filter is called `x` and consists of:
    0. x
    1. y
    2. z
    3. v_parallel
    4. e_x
    5. w_x
    6. e_y
    7. w_y
    8. e_z
    9. w_z

    Here e_xyz is the Euclidian representation of the delta from the center quaternion.
    w_xyz is the rotation speed.
    Additionaly, this filter stores `q` - the center quaternion around which the neighborhood e is built.
    More about this here: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6339217/
    Normally, `e_xyz==0` after the predict step. This means that `q` gives us the current attitude.

    You can get current state either directly from `x` and `q` attributes or by using these properties:
    `pos`, `vel`, `rot_vel`.
    ----------------------------
    Attributes:
    dt: float
        Time step
    x: np.array
        10-dim state vector
    P: np.array
        Computed covariance matrix for state vector
    q: np.array
        Orientation quaternion
    Q: np.array
        Model noise covariance matrix
    transition_jac: function
        Jacobian of nonlinear transition function
    '''
    ACCEL_STD = 5.0
    GRAVITY = np.array([0, 0, 9.8])
    
    def __init__(self, dt, vel_std, rot_vel_std):
        self.dt = dt
        self.x = np.zeros(10)
        self.P = np.eye(10)
        self.q = np.array([1., 0, 0, 0])
        Q_e_w = np.array([
            [0.333 * dt**3, -0.5 * dt**2],
            [ -0.5 * dt**2,           dt],
        ]) * rot_vel_std**2
        Q_vel = np.array([vel_std ** 2])
        self.Q = scipy.linalg.block_diag(
            Q_discrete_white_noise(dim=3, dt=dt, var=vel_std**2, block_size=1),
            Q_vel,
            Q_e_w, Q_e_w, Q_e_w,
        )

        self.transition_jac = physics.transition_jac
    
    def predict_by_nn_model(self, model, control):
        """
        Kalman filter predict step
        @ parameters
        model: 
            Pretrained NN control model
        @ return 
        x_opt:
            State vector after predict step
        P_opt:
            Covariance matrix after predict step
        """
        model_input = np.array([[self.v, self.w_yaw, control[0], control[1], self.dt]], 
                                dtype=np.float32)
        model_output = model(model_input)
        # Predicted velocity control
        self.v, self.w_yaw = float(model_output[0][0]), float(model_output[0][1])
        self.predict()

    def predict(self):
        self.reset_manifold()
        F = self.transition_jac(self.v, self.rot_vel, self.q, self.dt)
        self.predict_state()
        self.P = F @ self.P @ F.T + self.Q

    def predict_state(self):
        self.pos = self.predict_coords()
        self.epsilon, self.q = self.predict_quaternion()

    def predict_coords(self):
        """
        Predict robot coordinates in global frame
        """
        odom_extrinsic = geometry.quat_as_matrix(self.q)
        velocity_local = np.array([self.v, 0, 0])
        self.pos += odom_extrinsic @ velocity_local * self.dt
        return self.pos
    
    def predict_quaternion(self):
        """
        Predict robot orientation
        """
        rot_q = rot_vel_to_q(self.rot_vel, self.dt)
        # Rotate the current attitude
        next_q_center = geometry.quat_product(self.q, rot_q)
        next_q_center = next_q_center / np.sqrt(np.sum(next_q_center**2))
        # Quaternion will rotate, but epsilon will stay zero
        next_epsilon = self.epsilon
        return next_epsilon, next_q_center

    def reset_manifold(self):
        '''
        Reset the linearization center

        Called automatically before every predict step.
        '''
        self.epsilon, self.q = geometry.reset_manifold(self.epsilon, self.q)
    
    def update_odom(self, z, R):
        """
        Update state vector using odometry measurements
        @ parameters
        z: 
            Odometry measurement
        R:
            Odometry noise covariance matrix
        """
        z_prior, H = measurement.odometry(self.v, self.w_yaw)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_odom_flow(self, msg, stereo, extrinsic):
        '''
        Update filter state using flow odometry message
        '''
        if stereo is None:
            print('waiting for camera parameters...')
            return

        z = np.vstack([msg.flow_x, msg.flow_y, msg.delta_depth]).transpose() # [N, 3]
        pixels = np.vstack([msg.x, msg.y]).transpose() # [N, 2]
        R = np.diag(msg.covariance_diag)

        self.update_flow(
            z,
            self.dt,
            msg.depth,
            pixels,
            R,
            stereo.M1,
            stereo.M1_inv,
            extrinsic=extrinsic,
        )

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
        z_prior, H = measurement.flow_odom12(self.v, self.rot_vel, self.q, delta_t, depths, pixels, camera_matrix, camera_matrix_inv, extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_imu(self, 
                   z_acc, R_acc, acc_extrinsic,
                   z_gyro, R_gyro):
        """
        Update state vector using odometry measurements
        """
        self.update_acc(z_acc, R_acc, acc_extrinsic)
        self.update_gyro(z_gyro, R_gyro)

    def update_acc(self, z, R, extrinsic=None):
        '''
        Update state by a measurement coming from on-board accelerometer.
        '''
        R_new = R + np.eye(3) * self.ACCEL_STD**2
        self.update_static_vec(z, R_new, vec=np.array([0, 0, 9.8]), extrinsic=extrinsic)

    def update_static_vec(self, z, R, vec, extrinsic=None):
        '''
        Update state by a measurement of some vector which should be constant in the world coordinates.
        For example, magnetic field.
        '''
        z_prior, H = measurement.static_vec(self.q, vec, extrinsic=extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_gyro(self, z, R):
        '''
        Update state by a measurement coming from on-board gyroscope.
        '''
        z_prior, H = measurement.rot_vel_local(self.w_yaw)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def get_pose_covariance(self):
        '''
        Returns 6x6 covariance matrix of pose in the form a list with length 36:
        (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        '''
        indeces = [0, 1, 2, 4, 6, 8]
        mat = self.P[indeces][:, indeces]
        return list(mat.flatten())

    def get_twist_covariance(self):
        '''
        Returns 6x6 covariance matrix of velocity in the form a list with length 36:
        (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        '''
        rot_mat = geometry.quat_as_matrix(self.q)
        P_v_local = np.array([[self.P[3][3], 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]])
        P_v_global = rot_mat @ P_v_local @ rot_mat.T
        indeces = [5, 7, 9]
        mat = self.P[indeces][:, indeces]
        mat = scipy.linalg.block_diag(
            P_v_global,
            mat
        )
        return list(mat.flatten())

    @property
    def pos(self):
        return self.x[:3]
    @pos.setter
    def pos(self, value):
        self.x[:3] = value

    @property
    def v(self):
        return self.x[3]
    @v.setter
    def v(self, value):
        self.x[3] = value

    @property
    def rot_vel(self):
        return self.x[5::2]
    @rot_vel.setter
    def rot_vel(self, value):
        self.x[5::2] = value
    
    @property
    def w_yaw(self):
        return self.x[9]
    @w_yaw.setter
    def w_yaw(self, value):
        self.x[9] = value

    @property
    def epsilon(self):
        return self.x[4::2]
    @epsilon.setter
    def epsilon(self, value):
        self.x[4::2] = value

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
    rot_q[0] = np.cos(rot_angle_05)
    rot_q[1:] = w_unit * np.sin(rot_angle_05)
    return rot_q

@njit
def kalman_update(x, P, H, R, y):
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    new_x = x + K @ y
    new_P = (np.eye(x.shape[0]) - K @ H) @ P
    return new_x, new_P
