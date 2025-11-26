import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Package Name ---
    # Ensure this matches your actual package name (e.g., 'robot_description' or 'robot_bringup')
    pkg_robot_description = FindPackageShare('robot_sim_description')
    pkg_robot_bringup = FindPackageShare('robot_sim_bringup')
    # --- Paths ---
    
    # Path to the controller configuration YAML file
    controller_config_file = PathJoinSubstitution(
        [pkg_robot_bringup, 'config', 'diff_drive_controller.yaml']
    )
    
    # Paths to your existing launch files
    gazebo_launch_path = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'view_robot_gazebo.launch.py']
    )
    rviz_config_path = PathJoinSubstitution(
        [pkg_robot_description, 'rviz', 'ibex.rviz']
    )

    # --- Include Existing Launch Files ---

    # 1. Launch Gazebo (which spawns the robot)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Launch Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'odom': '/diff_drive_base_controller/odom'
        }.items()
    )

    # --- ROS 2 Control Spawners ---
    # Launched immediately without event handlers. 
    # They might error initially while waiting for Gazebo to start, but will retry.

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_base_controller', '--param-file', controller_config_file],
        output='screen',
    )

    return LaunchDescription([
        gazebo_sim,
        slam_launch,
        rviz_node,
        joint_state_broadcaster_spawner,
        diff_drive_base_controller_spawner,
    ])