import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Find package share directory
    pkg_robot_teleop = get_package_share_directory('robot_teleop')
    
    # Gamepad config file path
    f710_config_file = os.path.join(
        pkg_robot_teleop,
        'config',
        'f710_config.yaml'
    )

    # Declare launch arguments
    publish_stamped_arg = DeclareLaunchArgument(
        'publish_stamped',
        default_value='true',
        description='Whether to publish stamped twist messages (true/false)'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/diff_drive_base_controller/cmd_vel',
        description='Topic to remap /cmd_vel to'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (true/false)'
    )


    # Get launch configuration values
    publish_stamped = LaunchConfiguration('publish_stamped')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gamepad joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Teleop twist joy node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            f710_config_file,
            {'publish_stamped_twist': publish_stamped},
            {'use_sim_time': use_sim_time}
            ],
        remappings=[
            ('/cmd_vel', cmd_vel_topic)
        ],
        output='screen'
    )

    return LaunchDescription([
        publish_stamped_arg,
        cmd_vel_topic_arg,
        use_sim_time_arg,
        joy_node,    
        teleop_node    
    ])