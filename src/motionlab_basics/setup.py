from setuptools import find_packages, setup

package_name = 'motionlab_basics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/go_to_goal.launch.py']),
        ('share/' + package_name + '/config', ['config/go_to_goal.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'circle_controller = motionlab_basics.circle_controller:main',       
        'pose_monitor = motionlab_basics.pose_monitor:main',
        'go_to_goal = motionlab_basics.go_to_goal:main',      ],
    },
)
