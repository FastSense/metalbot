import rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

class Logger(Node):

    def __init__(self):
        super().__init__('logger')

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            1
        )

        # #self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        
        """
        for item in msg.transforms:
            if (
                item.header.frame_id == 'odom' and 
                item.child_frame_id == 'base_link'
            ):
                # tf_transform = self.tf_buffer.lookup_transform(
                #     'odom', 
                #     'base_link',
                #     self.get_clock().now(),
                #     0.1
                # )

                # trans_vec = tf_transform.transform.translation
                # rot_quat = tf_transform.transform.rotation
                # print(trans_vec)


def main(args=None):
    """

    """
    rclpy.init(args=args)

    logger = Logger()

    rclpy.spin(logger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



