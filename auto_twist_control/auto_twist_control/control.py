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

        # Initialise all given constants for robot control.
        self.image_depth_sub_topic: str = '/camera/depth/image_raw'
        self.image_optical_sub_topic: str = '/camera/image_raw'
        self.geometry_move_pub_topic: str = '/cmd_vel'
        self.default_angular_speed: float = 0.5
        self.default_linear_speed: float = 0.5
        self.close_distance: float = 0.75
        self.show_image: bool = True
        self.bridge: object = CvBridge()
        self.logger = self.get_logger()

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

        if self.show_image:
            self.display_cv_image(
                self.convert_depth_to_usable_np_frame(cv_image), 'Depth Image')

        # Initialise movement object and NP array of depth data.
        robot_direction = self.calculate_movement(np.array(cv_image))
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

    def calculate_movement(self, depth_array):
        depth_array = depth_array[:, :, 0]
        close_distance_mask = (
            depth_array < self.close_distance) & (depth_array > 0)
        robot_direction = Twist()
        # Checks if any single pixel is greater than the declared close distance.
        if depth_array[close_distance_mask].sum() > 0:
            close_distance_indices = np.where(close_distance_mask)[1]
            x_coord_midpoint = round(depth_array.shape[1] / 2)
            close_distance_indices[(
                close_distance_indices < x_coord_midpoint)] = -1
            close_distance_indices[(
                close_distance_indices >= x_coord_midpoint)] = 1
            # If robot is still moving, stop the robot.
            robot_direction.linear.x = 0.0
            if close_distance_indices.sum() > 0:
                direction = 'left'
                robot_direction.angular.z = self.default_angular_speed
            else:
                direction = 'right'
                robot_direction.angular.z = -self.default_angular_speed
            self.logger.info('Obstacle detected. Turning %s at %f metres.' %
                             (direction, self.default_angular_speed))
        else:
            self.logger.info('Moving at %f metres forward.' %
                             self.default_linear_speed)
            # If robot is still turning, stop turning.
            robot_direction.angular.z = 0.0
            robot_direction.linear.x = self.default_linear_speed
        return robot_direction


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
