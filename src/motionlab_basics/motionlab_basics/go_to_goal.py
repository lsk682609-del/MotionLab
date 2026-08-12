import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class GoToGoalController(Node):

    def __init__(self):
        super().__init__('go_to_goal')

        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('k_linear', 1.0)
        self.declare_parameter('k_angular', 4.0)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 4.0)
        self.declare_parameter('goal_tolerance', 0.1)

        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.k_angular = float(self.get_parameter('k_angular').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance').value
        )

        self.pose = None
        self.goal_reached = False
        self.control_count = 0

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10,
        )

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Goal set to ({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    def pose_callback(self, message):
        self.pose = message

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def limit(value, maximum):
        return max(-maximum, min(maximum, value))

    def publish_stop(self):
        self.publisher.publish(Twist())

    def control_loop(self):
        if self.pose is None:
            return

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance_error = math.hypot(dx, dy)

        if distance_error < self.goal_tolerance:
            self.publish_stop()

            if not self.goal_reached:
                self.get_logger().info(
                    f'Goal reached: distance={distance_error:.3f}'
                )
                self.goal_reached = True

            return

        self.goal_reached = False

        desired_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(
            desired_heading - self.pose.theta
        )

        angular_speed = self.limit(
            self.k_angular * heading_error,
            self.max_angular_speed,
        )

        if abs(heading_error) > 0.5:
            linear_speed = 0.0
        else:
            linear_speed = min(
                self.k_linear * distance_error,
                self.max_linear_speed,
            )

        command = Twist()
        command.linear.x = linear_speed
        command.angular.z = angular_speed
        self.publisher.publish(command)

        self.control_count += 1

        if self.control_count % 20 == 0:
            self.get_logger().info(
                f'distance={distance_error:.2f}, '
                f'heading_error={heading_error:.2f}, '
                f'v={linear_speed:.2f}, '
                f'omega={angular_speed:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = GoToGoalController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
