"""实机 Tag 趋近：机器人底层控制 + 相机检测和预览 GUI。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("apriltag_localization"), "config", "approach.yaml"])

    controller = Node(
        package="rl_sar",
        executable=["rl_real_", LaunchConfiguration("robot")],
        output="screen",
        emulate_tty=True,
    )
    approach = Node(
        package="apriltag_localization",
        executable="tag_approach",
        name="tag_approach",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("params_file"), {
            "image_source": LaunchConfiguration("image_source"),
            "show_image": True,
            "cmd_output": "robot_move_cmd",
            "cmd_topic": "/robot_move_cmd",
            "auto_start": ParameterValue(
                LaunchConfiguration("auto_start"), value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot", default_value="atdog2", choices=["atdog2", "atdog3"],
            description="实机型号"),
        DeclareLaunchArgument(
            "params_file", default_value=params_file,
            description="Tag、相机内外参与目标站位配置"),
        DeclareLaunchArgument(
            "image_source", default_value="realsense",
            choices=["realsense", "uvc", "auto"],
            description="D435 使用 realsense，普通 USB 相机使用 uvc"),
        DeclareLaunchArgument(
            "auto_start", default_value="false", choices=["true", "false"],
            description="默认待命，通过 /tag_approach/start 开始趋近"),
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason="Robot controller exited"))])),
        RegisterEventHandler(OnProcessExit(
            target_action=approach,
            on_exit=[EmitEvent(event=Shutdown(reason="Tag approach exited"))])),
        controller,
        approach,
    ])
