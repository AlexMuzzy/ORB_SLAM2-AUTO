from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError


class RobotControl(Node):

    def __init__(self):
        super().__init__('robot_control')

        # Initialise all given contants for robot control.
        self.image_depth_sub_topic: str = '/camera/depth/image_raw'
        self.image_optical_sub_topic: str = '/camera/image_raw'
        self.geometry_move_pub_topic: str = '/cmd_vel'
        self.default_angular_speed: int = 0.5
        self.default_linear_speed: int = 0.5
        self.show_image: bool = True
        self.bridge: object = CvBridge()
        self.logger = self.get_logger()
        self.angular_strengths = {
            'far-left': 1.0,
            'left': 0.5,
            'straight': 0,
            'right': -0.5,
            'far-right': -1.0
        }

        self.logger.info(
            f'Initialising subscription for {self.image_depth_sub_topic}')

        self.depth_image_sub = self.create_subscription(
            Image, self.image_depth_sub_topic, self.depth_image_callback, qos_profile=qos_profile_sensor_data)

        self.logger.info(
            f'Initialising subscription for {self.image_optical_sub_topic}')

        self.optical_image_sub = self.create_subscription(
            Image, self.image_optical_sub_topic, self.optical_image_callback, qos_profile=qos_profile_sensor_data)

        self.logger.info(
            f'Initialising publisher for {self.geometry_move_pub_topic}')

        self.control_publisher = self.create_publisher(
            Twist, self.geometry_move_pub_topic, 10)

    def optical_image_callback(self, message: Image) -> None:
        cv_image = self.convert_ros_imgmsg_to_cv_frame(message)

        if self.show_image:
            self.display_cv_image(cv_image, 'Optical Image')

    def depth_image_callback(self, message: Image) -> None:
        cv_image = self.convert_ros_imgmsg_to_cv_frame(message)
        test = self.convert_depth_to_usable_np_frame(cv_image)

        if self.show_image:
            self.display_cv_image(test, 'Depth Image')

        self.logger.info('Average Depth image value: %d' %
                         np.average(cv_image).astype(np.uint8))

        robot_direction = Twist()
        robot_direction.linear.x = 0.3
        self.control_publisher.publish(robot_direction)

    def display_cv_image(self, image: np.ndarray, window_title: str) -> None:
        self.logger.debug('Displaying %s with dimensions: %s' %
                          (window_title, ' '.join(map(str, image.shape))))

        cv2.imshow(window_title, image)
        cv2.waitKey(1)

    def convert_depth_to_usable_np_frame(self, frame: np.ndarray):
        # equate unusable values to 0.
        frame[np.isinf(frame)] = 0
        # Normalise frame.
        return (frame - frame.min()) / (frame.max() - frame.min())

    def convert_ros_imgmsg_to_cv_frame(self, image, depth_image=False):
        try:
            if depth_image:
                return self.bridge.imgmsg_to_cv2(image)
            else:
                return cv2.cvtColor(self.bridge.imgmsg_to_cv2(image), cv2.COLOR_BGR2RGB)

        except CvBridgeError as e:
            raise e


def main(args=None):
    rclpy.init(args=args)

    robot_control = RobotControl()

    rclpy.spin(robot_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    stop_movement = Twist()
    stop_movement.linear.x = 0
    robot_control.control_publisher.publish(stop_movement)
    robot_control.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
