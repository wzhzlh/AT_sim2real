"""只启动 AprilTag 视觉重定位节点。

适用于：地图服务器、AMCL、雷达驱动、机器人 TF 已经由别的 launch 启动的场景。
若希望一条命令把「地图 + AMCL + 视觉初值」整套拉起，用 full_localization.launch.py。
"""

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
