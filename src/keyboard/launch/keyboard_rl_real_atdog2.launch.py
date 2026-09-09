from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="keyboard",
            executable="keyboard_node",
            name="keyboard_node",
            output="screen",
        ),
        Node(
            package="rl_sar",
            executable="rl_real_atdog2",
            name="rl_real_atdog2",
            output="screen",
        ),
    ])
