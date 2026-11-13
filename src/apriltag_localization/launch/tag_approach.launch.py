"""无雷达纯视觉测试：让机器人走到 tag 码前面的固定位姿。

不启动 map_server / AMCL，也不查 TF 树；所有常数位姿从 4x4 矩阵参数读取。

    ros2 launch apriltag_localization tag_approach.launch.py
    # 干跑（不发速度，只看日志）：
    ros2 launch apriltag_localization tag_approach.launch.py \
        params_file:=/abs/path/to/dry_run.yaml
    # MuJoCo 仿真（ID 5，无需实体相机）：
    ros2 launch apriltag_localization tag_approach.launch.py use_mujoco:=true
    # 不开相机预览窗口（远程 / 无桌面时）：
    ros2 launch apriltag_localization tag_approach.launch.py show_image:=false
"""

from launch import LaunchDescription
from pathlib import Path

from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_nodes(context):
    use_mujoco = LaunchConfiguration("use_mujoco").perform(context).lower() == "true"
    parameters = [LaunchConfiguration("params_file")]
    actions = []
    if use_mujoco:
        parameters.append(PathJoinSubstitution([
            FindPackageShare("apriltag_localization"), "config", "approach_mujoco.yaml"]))
        simulator = ExecuteProcess(
            cmd=[LaunchConfiguration("mujoco_executable"),
                 LaunchConfiguration("robot"), "scene_tag"],
            additional_env={"AT_MUJOCO_ROS2": "1"}, output="screen",
            emulate_tty=True)
        actions += [simulator, RegisterEventHandler(OnProcessExit(
            target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo exited"))]))]
    parameters.append({"show_image": ParameterValue(
        LaunchConfiguration("show_image"), value_type=bool)})
    detector = Node(
        package="apriltag_localization", executable="tag_approach",
        name="tag_approach", output="screen", emulate_tty=True,
        parameters=parameters)
    actions.append(detector)
    if use_mujoco:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=detector,
            on_exit=[EmitEvent(event=Shutdown(reason="Tag detector exited"))])))
    return actions


def generate_launch_description():
    workspace = next((parent for parent in Path(__file__).resolve().parents
                      if (parent / "src/rl_sar").is_dir()), Path.cwd())
    pkg_share = FindPackageShare("apriltag_localization")
    default_params = PathJoinSubstitution([pkg_share, "config", "approach.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="测试参数 YAML；4x4 常数位姿矩阵就写在这里",
        ),
        # 相机预览窗口由节点自身用 OpenCV 创建（画面带 tag 检测框与坐标轴），
        # 不依赖 realsense2_camera / usb_cam，所以也不需要 rqt_image_view。
        # 需要图形界面（本机桌面或 ssh -X）；无 DISPLAY 时节点打印 WARN 并自动禁用。
        DeclareLaunchArgument(
            "show_image",
            default_value="true",
            description="启动相机预览窗口；无桌面时用 show_image:=false",
        ),
        DeclareLaunchArgument("use_mujoco", default_value="false",
                              choices=["true", "false"],
                              description="启动 MuJoCo、ID 5 标签场景和仿真相机"),
        DeclareLaunchArgument("robot", default_value="atdog2",
                              choices=["atdog2", "atdog3"]),
        DeclareLaunchArgument(
            "mujoco_executable",
            default_value=str(workspace / "cmake_build/bin/rl_sim_mujoco"),
            description="以 -DMUJOCO_ROS2=ON 编译的 rl_sim_mujoco 绝对路径"),
        OpaqueFunction(function=launch_nodes),
    ])
