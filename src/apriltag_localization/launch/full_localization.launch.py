"""完整链路：地图服务器 + AMCL + AprilTag 视觉初值注入。

    D435/AprilTag -> /initialpose -> AMCL(激光 scan 与 map 匹配) -> map->odom->base_link

运行示例：
    ros2 launch apriltag_localization full_localization.launch.py \
        map_yaml:=/abs/path/to/field_map.yaml

前提：
  * 激光雷达驱动已发布 /scan（frame_id 需与 localizer.yaml 的 lidar_frame 一致）
  * 机器人已有 odom -> base_link 的 TF 和 base_link -> lidar/camera 的静态 TF
  * 已按 teach.md 标定 tag 的 map 位姿与相机外参
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("apriltag_localization")
    default_localizer_params = PathJoinSubstitution(
        [pkg_share, "config", "localizer.yaml"])
    default_amcl_params = PathJoinSubstitution(
        [pkg_share, "config", "amcl.yaml"])

    map_yaml = LaunchConfiguration("map_yaml")
    localizer_params = LaunchConfiguration("localizer_params")
    amcl_params = LaunchConfiguration("amcl_params")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "yaml_filename": map_yaml,
            "topic_name": "map",
            "frame_id": "map",
        }],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[amcl_params],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
            "bond_timeout": 4.0,
        }],
    )

    localizer = Node(
        package="apriltag_localization",
        executable="apriltag_localizer",
        name="apriltag_localizer",
        output="screen",
        emulate_tty=True,
        parameters=[localizer_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_yaml",
            description="Nav2 地图 .yaml 的绝对路径（必填）",
        ),
        DeclareLaunchArgument(
            "localizer_params",
            default_value=default_localizer_params,
            description="AprilTag 节点参数 YAML",
        ),
        DeclareLaunchArgument(
            "amcl_params",
            default_value=default_amcl_params,
            description="AMCL 参数 YAML",
        ),
        map_server,
        amcl,
        lifecycle_manager,
        # 延后 3 秒再启动视觉节点，确保 AMCL 已 configure/activate 并订阅
        # /initialpose；否则一次性初值消息可能丢失。
        TimerAction(period=3.0, actions=[localizer]),
    ])
