from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_control_node = Node(
        package="obstacle_game",
        executable="robot_control",
        output="screen",
        parameters=[{
            "autopilot_available": False,
        }],
    )

    remote_node = Node(
        package="remote_node",
        executable="remote_node",
        output="screen",
    )

    return LaunchDescription([
        robot_control_node,
        remote_node,
    ])
