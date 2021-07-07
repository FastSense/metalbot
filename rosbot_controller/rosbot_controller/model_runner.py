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

    def __init__(self, node_name, type):
        rclpy.init()
        super().__init__(node_name)
        self.type = type
        self.cmd_topic = '/cmd_vel'

        self.robot = Rosbot()
        self.model_state = RobotState()

        self.cmd_freq = 60.0  # Hz
        self.dt = 1.0 / self.cmd_freq

        self.tf_br = TransformBroadcaster(self)
        
        self.control_vector = RobotControl()
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self.command_callback, 10)

        self.last_timestamp = self.get_clock().now().to_msg().sec

    def command_callback(self, msg: Twist):
        """


        """
        self.control_vector = RobotControl(msg.linear.x, msg.angular.z)

    def broadcast_model_tf(self, state: RobotState):
        """

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
        msg.header.frame_id = "odom"
        msg.child_frame_id = "model_link"
        msg.transform.translation = pose
        msg.transform.rotation = quat
        print(msg)
        self.tf_br.sendTransform(msg)

    def _kinematic_model(self):
        """
        Kinematic model runner
        """
        current_timestamp = self.get_clock().now().to_msg().sec
        dt = current_timestamp - self.last_timestamp

        self.model_state = self.robot.update_state_by_model(
            self.control_vector, dt)

        self.broadcast_model_tf(self.model_state)

        self.last_timestamp = current_timestamp

    def _nn_model(self):
        """
        NN model runner
        """
        pass

    def run(self):

        if self.type == "kinematic":
            self.create_timer(self.dt, self._kinematic_model)
            rclpy.spin(self)
        elif self.type == "nn":
            self.create_timer(self.dt, self._nn_model)
            rclpy.spin(self)
        else:
            print("Error type")
            return

        rclpy.spin(self)


def main():
    runner = ModelRunner(node_name="kinematic", type="kinematic")
    runner.run()


if __name__ == '__main__':
    main()
