"""兼容启动入口：只启动 Tag odom 重定位，不再启动 AMCL/地图服务器。

雷达 odom 与机器人静态外参需事先启动；通过 localizer_params 指定配置。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("apriltag_localization"), "config", "localizer.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("localizer_params", default_value=default_params),
        Node(
            package="apriltag_localization",
            executable="apriltag_localizer",
            name="apriltag_localizer",
            output="screen",
            parameters=[LaunchConfiguration("localizer_params")],
        ),
    ])
