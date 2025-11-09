import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_robot_teleop = get_package_share_directory('robot_teleop')
    
    f710_config_file = os.path.join(
        pkg_robot_teleop,
        'config',
        'f710_config.yaml'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen' 
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            f710_config_file],
        output='screen'
    )

    return LaunchDescription([
        joy_node,    
        teleop_node    
    ])