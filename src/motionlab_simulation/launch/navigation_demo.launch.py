import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ---------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------

    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz2 visualization'
    )

    # ---------------------------------------------------------
    # Package directories
    # ---------------------------------------------------------

    simulation_share = get_package_share_directory(
        'motionlab_simulation'
    )

    robot_share = get_package_share_directory(
        'motionlab_robot_description'
    )


    # ---------------------------------------------------------
    # Files
    # ---------------------------------------------------------

    params_file = os.path.join(
        simulation_share,
        'config',
        'navigation_params.yaml'
    )

    map_yaml = os.path.join(
        simulation_share,
        'maps',
        'motionlab_map.yaml'
    )

    rviz_config = os.path.join(
        robot_share,
        'rviz',
        'motionlab_navigation.rviz'
    )

    # ---------------------------------------------------------
    # Gazebo simulation
    # ---------------------------------------------------------

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                simulation_share,
                'launch',
                'simulation.launch.py'
            )
        )
    )

    # ---------------------------------------------------------
    # Static localization transform
    #
    # Temporary development solution:
    #
    # map -> odom = identity
    #
    # This will later be replaced by AMCL.
    # ---------------------------------------------------------

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
        output='screen'
    )

    # ---------------------------------------------------------
    # Static map server
    # ---------------------------------------------------------

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml,
                'use_sim_time': True
            }
        ]
    )

    # ---------------------------------------------------------
    # Automatically configure + activate map_server
    # ---------------------------------------------------------

    lifecycle_manager = TimerAction(
    period=8.0,
    actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server']
                }
            ]
        )
    ]
    )

    # ---------------------------------------------------------
    # A* planner
    # ---------------------------------------------------------

    planner = Node(
        package='motionlab_control_cpp',
        executable='astar_planner',
        name='astar_planner',
        output='screen',
        parameters=[
            params_file
        ]
    )

    # ---------------------------------------------------------
    # LiDAR safety layer
    # ---------------------------------------------------------

    safety_stop = Node(
        package='motionlab_control_cpp',
        executable='lidar_safety_stop',
        name='lidar_safety_stop',
        output='screen',
        parameters=[
            params_file
        ]
    )

    # ---------------------------------------------------------
    # Pure Pursuit controller
    # ---------------------------------------------------------

    controller = Node(
        package='motionlab_control_cpp',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            params_file
        ]
    )

    # ---------------------------------------------------------
    # RViz
    # ---------------------------------------------------------

    rviz = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=[
        '-d',
        rviz_config
    ],
    parameters=[
        {
            'use_sim_time': True
        }
    ],
    output='screen',
    additional_env={
        'QT_QPA_PLATFORM': 'xcb',
        'GALLIUM_DRIVER': 'd3d12'
    }
  )

    # ---------------------------------------------------------
    # Complete system
    # ---------------------------------------------------------

    return LaunchDescription([
        declare_use_rviz,

        simulation_launch,

        map_to_odom,

        map_server,
        

        planner,
        safety_stop,
        controller,

        rviz,
        lifecycle_manager,
    ])