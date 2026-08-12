import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')

        self.latest_pose = None

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
        )

        self.timer = self.create_timer(0.5, self.report_pose)
        self.get_logger().info('Pose monitor started')

    def pose_callback(self, message):
        self.latest_pose = message

    def report_pose(self):
        if self.latest_pose is None:
            self.get_logger().warning('Waiting for /turtle1/pose')
            return

        pose = self.latest_pose

        self.get_logger().info(
            f'x={pose.x:.2f}, '
            f'y={pose.y:.2f}, '
            f'theta={pose.theta:.2f}, '
            f'v={pose.linear_velocity:.2f}, '
            f'omega={pose.angular_velocity:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = PoseMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
