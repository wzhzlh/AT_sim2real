from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DOG_NAME = "dog3"
PATH_GROUPS = ("main", "stairs", "brige_a", "brige_b")


def _trajectory_dir():
    candidates = [Path.cwd(), Path(__file__).resolve()]
    for candidate in candidates:
        for parent in (candidate, *candidate.parents):
            path = parent / "trajectory"
            if path.is_dir():
                return path
    return Path.cwd() / "trajectory"


def _launch_setup(context, *args, **kwargs):
    side = LaunchConfiguration("side").perform(context).lower()
    profile = "2" if side == "p2" else "1"

    trajectory_dir = _trajectory_dir()
    scene_paths = []
    for group in PATH_GROUPS:
        yaml_path = trajectory_dir / f"{group}_{DOG_NAME}_{profile}.yaml"
        if yaml_path.is_file():
            scene_paths.append(str(yaml_path))

    robot_control_node = Node(
        package="obstacle_game",
        executable="robot_control",
        output="screen",
        parameters=[{
            "scene_path": ";".join(scene_paths),
        }],
    )

    return [robot_control_node]


def generate_launch_description():

    remote_node = Node(
        package="remote_node",
        executable="remote_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="p1", choices=["p1", "p2"]),
        OpaqueFunction(function=_launch_setup),
        remote_node,

    ])
