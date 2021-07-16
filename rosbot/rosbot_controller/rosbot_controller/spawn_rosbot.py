"""
spawn_rosbot.py

Script used to spawn a rosbot
"""
import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xml.etree.ElementTree as ET
from rclpy.node import Node


class RosbotSpawner(Node):
    """
    Simple node for RosBot spawn
    The node parses the RosBot model, changes the frequency of 
    publication to the topic of odometry ('/odom'),
    and causes a service for a spawn robot in simulation.
    :Attributes:
        :update_rate: (int) - odometry publishing frequence
    """

    def __init__(self):
        """
        """
        super().__init__('spawn_rosbot')
        self.init_parameters()
        self.get_node_parametes()

    def init_parameters(self):
        """
        Inits node parameters
        """
        self.declare_parameter('rosbot_update_rate', "10")

    def get_node_parametes(self):
        """
        Gets node parameters
        """
        self.update_rate = self.get_parameter(
            'rosbot_update_rate').get_parameter_value().integer_value

    def run(self):
        """
        """
        rosbot_description_dir = get_package_share_directory('rosbot_description')
        # Original RosBot model
        original_model = os.path.join(
            rosbot_description_dir,
            "models",
            "rosbot.sdf"
        )
        # RosBot model with modified parameters
        new_model = os.path.join(
            rosbot_description_dir,
            'models/rosbot_fs.sdf'
        )

        # parse original rosbot model
        tree = ET.parse(original_model)
        rosbot_update_rate_value = tree.getroot().find('model/plugin/update_rate')
        # change
        rosbot_update_rate_value.text = str(self.update_rate)
        tree.write(new_model)

        self.get_logger().info('Creating Service client to connect to `/spawn_entity`')
        client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Connecting to `/spawn_entity` service...")

        if not client.service_is_ready():
            client.wait_for_service()
            self.get_logger().info("...connected!")

        request = SpawnEntity.Request()
        request.name = "rosbot"
        request.xml = open(new_model, 'r').read()
        request.robot_namespace = ""
        request.initial_pose.position.x = float(0)
        request.initial_pose.position.y = float(0)
        request.initial_pose.position.z = float(0.03)

        self.get_logger().info("Sending service request to `/spawn_entity`")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())

        self.get_logger().info("Done! Shutting down node.")
        self.destroy_node()
        rclpy.shutdown()


def main():
    """
    """
    rclpy.init()
    spawner = RosbotSpawner()
    spawner.run()


if __name__ == "__main__":
    main()
