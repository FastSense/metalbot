import rclpy
from rclpy.node import Node
import numpy as np

class ControlGenerator(Node):
    """
    """
    def __init__(self):
        """

        """
        super().__init__('control_generator')

        self.wait_for_subs()
        self.init_parameters()
        self.get_parametes()

    def wait_for_subs():
        """
        """
        pass

    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameter('mode', "periodic")


    def get_parametes(self):
        """
        Gets node parameters
        """
        pass

    

def main():
    """

    """
    rclpy.init()

    control_gen = ControlGenerator()

    try:
        rclpy.spin(control_gen)
    except:
        pass

    control_gen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()