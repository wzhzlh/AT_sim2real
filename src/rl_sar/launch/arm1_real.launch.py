from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/ttyACM0"),
        Node(
            package="rl_sar",
            executable="rl_real_arm1",
            name="arm1_rl_deployment",
            output="screen",
            parameters=[{"device": LaunchConfiguration("device")}],
            emulate_tty=True,
        ),
    ])
