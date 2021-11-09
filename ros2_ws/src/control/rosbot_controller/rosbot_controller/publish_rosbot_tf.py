"""
spawn_rosbot.py

Dummy script used to publish TF transforms of the rosbot
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

class ROSbotTFPublisher(Node):
    """
    """
    def __init__(self):
        rclpy.init()
        super().__init__("ROSbotTFPublisher")
        self.pub_freq_in_hz = 10

        self.dt = 1.0 / self.pub_freq_in_hz 

        self.tf_br = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.broadcast_all_tf)
        rclpy.spin(self)


    def broadcast_tf(self, pose_:Vector3(), rotation_, parent_frame, child_frame_id):
        """
        """
        
        goal_quat = Rotation.from_euler('zyx', [rotation_[2], rotation_[1], rotation_[0]], degrees=False).as_quat()
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = goal_quat[0], goal_quat[1], goal_quat[2], goal_quat[3]

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame_id
        msg.transform.translation = pose_
        msg.transform.rotation = quat
        self.tf_br.sendTransform(msg)

    def broadcast_all_tf(self):
        """
        """
        pose = Vector3()
        # ['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_depth_frame']
        pose.x = 0.0
        pose.y = 0.0
        pose.z = 0.0
        self.broadcast_tf(pose, [-1.5707, 0, -1.5707], 'camera_link', 'camera_depth_frame')
        
        # ['-0.03', '0', '0.11', '0', '0', '0', 'base_link', 'camera_link']
        # pose.x = -0.03
        # pose.y = 0.0
        # pose.z = 0.11
        # self.broadcast_tf(pose, [0, 0, 0], 'base_link', 'camera_link')
        
        # ['0.0', '0.0', '0.08', '-3.141593', '0.0', '0.0', 'base_link', 'laser']
        pose.x = 0.0
        pose.y = 0.0
        pose.z = 0.08
        self.broadcast_tf(pose, [-3.141593, 0, 0], 'base_link', 'laser')
        
        # ['0.08', '0.1', '0', '0', '0', '0', 'base_link', 'front_left_wheel']
        pose.x = 0.08
        pose.y = 0.1
        pose.z = 0.0
        self.broadcast_tf(pose, [0, 0, 0], 'base_link', 'front_left_wheel')
        
        # arguments=['0.08', '-0.1', '0', '0', '0', '0', 'base_link', 'front_right_wheel']
        pose.x = 0.08
        pose.y = -0.1
        pose.z = 0.0
        self.broadcast_tf(pose, [0, 0, 0], 'base_link', 'front_right_wheel')
        
        # arguments=['-0.08', '0.1', '0', '0', '0', '0', 'base_link', 'rear_left_wheel']
        pose.x = -0.08
        pose.y = 0.1
        pose.z = 0.0
        self.broadcast_tf(pose, [0, 0, 0], 'base_link', 'rear_left_wheel')
        
        # arguments=['-0.08', '-0.1', '0', '0', '0', '0', 'base_link', 'rear_right_wheel']
        pose.x = -0.08
        pose.y = -0.1
        pose.z = 0.0
        self.broadcast_tf(pose, [0, 0, 0], 'base_link', 'rear_right_wheel')
        
        # arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'top']
        pose.x = 0.0
        pose.y = 0.0
        pose.z = 0.0
        self.broadcast_tf(pose, [0, 0, 0], 'base_link', 'top')
        
        # arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link']
        pose.x = 0.0
        pose.y = 0.0
        pose.z = 0.18
        self.broadcast_tf(pose, [0, 0, 0], 'base_link', 'camera_link')
       


def main():
    node = ROSbotTFPublisher()


if __name__ == '__main__':
    main()
