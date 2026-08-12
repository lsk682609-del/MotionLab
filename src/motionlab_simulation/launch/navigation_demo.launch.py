import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    stop_distance = LaunchConfiguration('stop_distance')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz2 visualization'
    )
    declare_lookahead_distance = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.6',
        description='Pure Pursuit lookahead distance in meters'
    )

    declare_stop_distance = DeclareLaunchArgument('stop_distance', default_value='0.4', description='Emergency stop distance in meters')

    simulation_share = get_package_share_directory(
        'motionlab_simulation'
    )

    robot_share = get_package_share_directory(
        'motionlab_robot_description'
    )

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                'launch',
                'simulation.launch.py'
            )
        )
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ]
    )

    planner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='motionlab_control_cpp',
                executable='astar_planner',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': True,
                        'start_x': 0.5,
                        'start_y': 0.5,
                        'goal_x': 9.5,
                        'goal_y': 5.5
                    }
                ]
            )
        ]
    )

    controller = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='motionlab_control_cpp',
                executable='pure_pursuit_controller',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': True,
                        'lookahead_distance': lookahead_distance,
                        'nominal_speed': 0.25,
                        'max_angular_speed': 1.0,
                        'goal_tolerance': 0.15
                    }
                ]
            )
        ]
    )

    safety_stop = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="motionlab_control_cpp",
                executable="lidar_safety_stop",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "stop_distance": stop_distance,
                    "front_angle_deg": 30.0
                }]
            )
        ]
    )
    rviz_config = os.path.join(
        robot_share,
        'rviz',
        'motionlab_navigation.rviz'
    )

    rviz = TimerAction(
        period=4.0,
        actions=[
            Node(
                condition=IfCondition(use_rviz),
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
                additional_env={
                    'LIBGL_ALWAYS_SOFTWARE': '1'
                }
            )
        ]
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_lookahead_distance,
        declare_stop_distance,
        simulation_launch,
        map_to_odom,
        planner,
        rviz,
        safety_stop,
        controller
    ])
