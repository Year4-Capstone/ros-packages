from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Package Name ---
    # Ensure this matches your actual package name (e.g., 'robot_description' or 'robot_bringup')
    pkg_robot_description = FindPackageShare('robot_description')
    pkg_robot_bringup = FindPackageShare('robot_bringup')
    # --- Paths ---
    
    # Path to the controller configuration YAML file
    controller_config_file = PathJoinSubstitution(
        [pkg_robot_bringup, 'config', 'diff_drive_controller.yaml']
    )
    
    xacro_file = PathJoinSubstitution(
        [pkg_robot_description, 'urdf', 'ibex.urdf.xacro']
    )
    


    rviz_config_path = PathJoinSubstitution(
        [pkg_robot_description, 'rviz', 'ibex.rviz']
    )

    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file]
    )
    
    # Wrap the content to ensure it is treated as a string, not YAML
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config_file],
        output="screen",
    )

    # 2. Launch Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # --- ROS 2 Control Spawners ---
    # Launched immediately without event handlers. 
    # They might error initially while waiting for Gazebo to start, but will retry.

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    flipper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['flipper_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )


    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=controller_manager_node,
    #         on_start=[diff_drive_base_controller_spawner],
    #     )
    # )

    delayed_flipper_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[flipper_controller_spawner],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        node_robot_state_publisher,
        controller_manager_node,
        rviz_node,
        # delayed_diff_drive_spawner,
        delayed_flipper_spawner,
        delayed_joint_broad_spawner,
    ])