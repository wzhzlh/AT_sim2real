"""启动 Tag 对齐现实雷达 odom 到理想地图的节点；雷达及外参 TF 需单独启动。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("apriltag_localization")
    default_params = PathJoinSubstitution([pkg_share, "config", "localizer.yaml"])
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="节点参数 YAML；现场部署前必须修改 tag_poses / 外参 / 帧名",
        ),
        Node(
            package="apriltag_localization",
            executable="apriltag_localizer",
            name="apriltag_localizer",
            output="screen",
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
