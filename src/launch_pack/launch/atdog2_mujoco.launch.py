"""Launch the MuJoCo simulator with the atdog2 default scene."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    """Start rl_sim_mujoco using atdog2_description/mjcf/scene.xml."""
    return LaunchDescription(
        [
            ExecuteProcess(
                # Explicitly bind stdin to the controlling terminal.  ROS 2's
                # launch subprocess otherwise may receive a pipe, while the
                # simulator reads raw key events directly from stdin.
                cmd=[
                    "bash",
                    "-lc",
                    "exec ros2 run rl_sar rl_sim_mujoco atdog2 scene </dev/tty",
                ],
                output="screen",
                emulate_tty=True,
            )
        ]
    )
