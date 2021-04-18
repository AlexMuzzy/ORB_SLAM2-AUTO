import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from rclpy.qos import qos_profile_sensor_data


class RobotControl(Node):

    def __init__(self):
        super().__init__('robot_control')

        # Initialise all given contants for robot control.
        self.image_depth_sub_topic = '/camera/depth/image_raw'
        self.geometry_move_pub_topic = '/cmd_vel'
        self.default_angular_speed = 0.5
        self.default_linear_speed = 0.5
        self.show_optical_image = True
        self.show_depth_image = True
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, self.image_depth_sub_topic, self.image_callback, qos_profile=qos_profile_sensor_data)

        self.control_publisher = self.create_publisher(
            Twist, self.geometry_move_pub_topic, qos_profile=qos_profile_sensor_data)

    def optical_image_callback(self, message: Image) -> None:

    def image_callback(self, message: Image) -> None:
        """
        Objectives for callback method:
         - Show optical and depth data in their own frames.
         - 
        """
        message = Twist()

    def ros_image_to_cv_frame(self, image: Image, window_title: str) -> None:
        """
        Convert the ROS2 optical image stream to OpenCV image format and display given frame.
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                image, desired_encoding='passthrough')

        except CvBridgeError as e:
            self.get_logger().error(e)

        cv2.imshow(window_title, cv_image)


def main(args=None):
    rclpy.init(args=args)

    robot_control = RobotControl()

    rclpy.spin(robot_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
