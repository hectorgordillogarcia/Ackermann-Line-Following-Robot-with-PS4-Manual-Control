#! /usr/bin/env python3

"""
ackermann_node.py
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2

from ros_follow_line.controller_pid import PID
from ros_follow_line.color_filter import red_filter, get_centroid


class AckermannNode(Node):
    """Ackermann Node
    """

    def __init__(self, max_speed=4) -> None:
        super().__init__('ackermann_node')
        self.max_speed = max_speed

        self.linear_controller = PID(k_p=0.5, k_d=0.1)
        self.linear_controller.setpoint = 0.0
        self.angular_controller = PID(k_p=0.25, k_d=0.05)
        self.angular_controller.setpoint = 0.0

        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self._img_sub = self.create_subscription(
            Image, '/cam/image_raw', self.img_callback, 10)
        self.cv_bridge = CvBridge()

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_callback(self, msg: Odometry) -> None:
        """Odometry callback
        """
        pass

    def img_callback(self, msg: Image) -> None:
        """Image callback
        """
        frame = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = red_filter(frame)
        centroid = get_centroid(img)

        height, width, _ = frame.shape
        center = (width // 2, height // 2)
        angular_error = (center[0] - centroid[0]) / 100
        self.send_cmd_vel(angular_error)

        # # Debug
        cv2.circle(frame, centroid, 1, (0, 255, 0), 10)
        cv2.imshow("window", img)
        cv2.imshow("window2", frame)
        cv2.waitKey(1)

    def send_cmd_vel(self, error):
        """Velocity commander
        """
        linear_vel = self.linear_controller.update(-error)
        angular_vel = self.angular_controller.update(-error)

        print(f"{error=}")
        print(f"Linear Velocity: {self.max_speed - abs(linear_vel)}  Angular Velocity: {angular_vel}")

        twist = Twist()
        twist.linear.x = self.max_speed - abs(linear_vel)
        twist.angular.z = angular_vel
        self._cmd_vel_pub.publish(twist)


def main(args=None):
    """Main method
    """
    rclpy.init(args=args)

    node = AckermannNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
