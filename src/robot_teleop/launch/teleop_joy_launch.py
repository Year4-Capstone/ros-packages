# This launch file starts two nodes:
# 1. joy_node: Interfaces with the physical gamepad and publishes joy messages
# 2. teleop_node: Converts joy messages to velocity commands (Twist/TwistStamped)


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

    # Launch arguments
    # Controls whether to publish TwistStamped (true) or Twist (false) messages
    publish_stamped_arg = DeclareLaunchArgument(
        'publish_stamped',
        default_value='true',
        description='Whether to publish stamped twist messages (true/false)'
    )

    # Specifies the topic name where velocity commands will be published
    # Must match the topic your controller expects (check controller config)
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel_gamepad_stamped',
        description='Topic to remap /cmd_vel to'
    )

    # When true, nodes use simulation time from /clock topic (for Gazebo)
    # When false, nodes use system time (for physical robots)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (true/false)'
    )


    # These retrieve the actual values of arguments (either default or user-provided)
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
            {'use_sim_time': use_sim_time} # Sync with sim/real time
        ]
    )

    # Teleop twist joy node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            f710_config_file,                           # Load F710 configuration
            {'publish_stamped_twist': publish_stamped}, # Control message type
            {'use_sim_time': use_sim_time}              # Sync with sim/real time
            ],
        remappings=[
            ('/cmd_vel', cmd_vel_topic) # Remap default /cmd_vel topic to user-specified topic
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        publish_stamped_arg,
        cmd_vel_topic_arg,
        use_sim_time_arg,

        # Nodes
        joy_node,    
        teleop_node    
    ])