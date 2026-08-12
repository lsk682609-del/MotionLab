import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory(
        'motionlab_robot_description'
    )
    simulation_share = get_package_share_directory(
        'motionlab_simulation'
    )
    ros_gz_sim_share = get_package_share_directory(
        'ros_gz_sim'
    )

    xacro_file = os.path.join(
        description_share,
        'urdf',
        'motionlab_car.urdf.xacro',
    )

    world_file = os.path.join(
        simulation_share,
        'worlds',
        'motionlab_world.sdf',
    )

    robot_description = {
        'robot_description': Command([
            'xacro ',
            xacro_file,
        ])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_share,
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': '-r -s --headless-rendering -v 2 ' + world_file,
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'motionlab_car',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.11',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
