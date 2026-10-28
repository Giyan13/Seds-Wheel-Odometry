from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_odometry',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
        ),
    ])
