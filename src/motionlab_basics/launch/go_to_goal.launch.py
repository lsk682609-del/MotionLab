import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('motionlab_basics')

    parameter_file = os.path.join(
        package_share,
        'config',
        'go_to_goal.yaml',
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    controller_node = Node(
        package='motionlab_basics',
        executable='go_to_goal',
        name='go_to_goal',
        output='screen',
        parameters=[parameter_file],
    )

    return LaunchDescription([
        turtlesim_node,
        controller_node,
    ])
