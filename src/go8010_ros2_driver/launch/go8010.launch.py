from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    return LaunchDescription([Node(package='go8010_ros2_driver', executable='go8010_driver_node', name='go8010_driver', parameters=[os.path.join(get_package_share_directory('go8010_ros2_driver'),'config','go8010.yaml')], output='screen')])
