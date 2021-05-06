import rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage




class Logger(Node):
    """

    """
    def __init__(self):
        """

        """
        super().__init__('logger')
      
        self.declare_logger_parameters()
        self.get_logger_parametes()

        self.subscription = self.create_subscription(
            TFMessage,
            self.tf_topic,
            self.listener_callback,
            1
        )
        
        self.subscription  # prevent unused variable warning

    def declare_logger_parameters(self):
        """

        """
        self.declare_parameter('output_path', "")
        self.declare_parameter('output_folder', 'output_data')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('kinetic_model_frame', 'model_link')

    def get_logger_parametes(self):
        """

        """
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        self.tf_topic = self.get_parameter('tf_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.kinetic_model_frame = self.get_parameter('kinetic_model_frame').get_parameter_value().string_value


    def listener_callback(self, msg):
        """
        
        """
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.parent_frame and 
                transform.child_frame_id == self.robot_frame
            ):  
                print("Translation")   
                print(transform.transform.translation)
                print("Rotation")
                print(transform.transform.rotation)



def main():
    """

    """
    rclpy.init()

    logger = Logger()

    rclpy.spin(logger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



