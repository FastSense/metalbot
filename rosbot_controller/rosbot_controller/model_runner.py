from scipy.spatial.transform import Rotation
import rclpy
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Quaternion
import nnio
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rosbot_controller.rosbot_2D import Rosbot, RobotState, RobotControl


class ModelRunner(Node):
    """
    Predict state of a kinematic or NN model.

    Принимать type из командной строки

    """

    def __init__(self):
        rclpy.init()
        super().__init__("model runner")
        self.declare_and_get_parametrs()
        # Node name = model_type

        # self.model_type = type
        # self.control_topic = '/cmd_vel'
        # self.parent_frame = "odom"
        # self.cmd_freq = 60.0  # Hz

        self.child_frame_id = None
        self.robot = Rosbot()
        self.model_state = RobotState()

        self.dt = 1.0 / self.cmd_freq

        self.tf_br = TransformBroadcaster(self)

        self.control_vector = RobotControl()
        self.cmd_sub = self.create_subscription(
            Twist, self.control_topic, self.command_callback, 10)

        self.nn_model = None

    def declare_and_get_parametrs(self):
        """
        Declare and get node parameters

        """
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("model_type", "kinematic")
        self.declare_parameter("cmd_freq", 60.0)
        self.declare_parameter("nn_model_path", "")

        self.nn_model_path = self.get_parameter(
            'nn_model_path').get_parameter_value().string_value
        self.model_type = self.get_parameter(
            'model_type').get_parameter_value().string_value
        self.control_topic = self.get_parameter(
            'control_topic').get_parameter_value().string_value
        self.cmd_freq = self.get_parameter(
            'cmd_freq').get_parameter_value().double_value
        self.parent_frame = self.get_parameter(
            'parent_frame').get_parameter_value().string_value

    def command_callback(self, msg: Twist):
        """
        Save msg to contro_vector (new current control).

        """
        self.control_vector = RobotControl(msg.linear.x, msg.angular.z)

    def broadcast_model_tf(self, state: RobotState):
        """
        Broadcast our predicted position

        """
        pose = Vector3()
        pose.x, pose.y, pose.z = state.x, state.y, 0.0
        goal_quat = Rotation.from_euler(
            'z', state.yaw, degrees=False
        ).as_quat()

        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = goal_quat[0], goal_quat[1], goal_quat[2], goal_quat[3]

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame_id
        msg.transform.translation = pose
        msg.transform.rotation = quat
        self.tf_br.sendTransform(msg)

    def _kinematic_model(self):
        """
        Kinematic model runner

        """
        self.model_state = self.robot.update_state_by_model(
            self.control_vector, self.dt)

        self.broadcast_model_tf(self.model_state)

    def _nn_model(self):
        """
        Neural network model runner.

        """
        self.model_state = self.robot.update_state_by_nn_model(
            self.nn_model, self.control_vector, self.dt
        )

        self.broadcast_model_tf(self.model_state)

    def run(self):
        """
        Main method.

        """
        if self.model_type == "kinematic":
            self.get_logger().info("Start kinematic model")
            self.child_frame_id = "kinematic_model_link"

            self.create_timer(self.dt, self._kinematic_model)
            rclpy.spin(self)
        elif self.model_type == "nn":
            if self.nn_model_path is None or self.nn_model_path == "":
                self.get_logger().error("Wrong path to neural network model")
                return
            self.get_logger().info("Start neural network  model")
            self.child_frame_id = "nn_model_link"

            #  load NN model (.onnx) from given path, using nnio module
            self.nn_model = nnio.ONNXModel(self.nn_model_path)
            self.create_timer(self.dt, self._nn_model)
            rclpy.spin(self)
        else:
            self.get_logger().error("Error type")
            return

        rclpy.spin(self)


def main():
    runner = ModelRunner()
    runner.run()


if __name__ == '__main__':
    main()
