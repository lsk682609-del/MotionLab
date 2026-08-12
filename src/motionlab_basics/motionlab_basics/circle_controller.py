from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CircleController(Node):
    def __init__(self):
        super().__init__('circle_controller')
        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 1.0)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10,
        )

        self.timer = self.create_timer(0.1, self.publish_command)
        self.get_logger().info('Circle controller started')

    def publish_command(self):
        command = Twist()
        command.linear.x = self.linear_speed
        command.angular.z = self.angular_speed
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)

    node = CircleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
