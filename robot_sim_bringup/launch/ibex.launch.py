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
        launch_arguments={
            'world': PathJoinSubstitution([
                pkg_robot_description,
                'world',
                'small_house.world'
            ])
        }.items()
    )

    # 2. Launch Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         FindPackageShare('slam_toolbox'),
    #         '/launch/online_async_launch.py'
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'params_file': PathJoinSubstitution([
    #             pkg_robot_bringup, 'config', 'slam_params.yaml'
    #         ])
    #     }.items()
    # )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': '/home/luke/Projects/capstone/search-support-robots/ros-packages/robot_sim_bringup/maps/small_house/map.yaml',
            'params_file': PathJoinSubstitution([pkg_robot_bringup, 'config', 'nav2_params.yaml']),
            'autostart': 'true',
        }.items()
    )

    

    # nav2_map_server = Node(
    #     package="nav2_map_server",
    #     executable="map_server",
    #     name="map_server",
    #     output="screen",
    #     parameters=[
    #         os.path.join(
    #             bumperbot_navigation_pkg,
    #             "config",
    #             "nav2_params.yaml"
    #         ),
    #         {"use_sim_time": use_sim_time}
    #     ],
    # )

    # nav2_amcl = Node(
    #     package="nav2_amcl",
    #     executable="amcl",
    #     name="amcl",
    #     output="screen",
    #     parameters=[
    #         os.path.join(
    #             bumperbot_navigation_pkg,
    #             "config",
    #             "nav2_params.yaml"
    #         ),
    #         {"use_sim_time": use_sim_time}
    #     ],
    # )

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

    # Ground truth of robot pose
    gz_gt_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
        ],
        output='screen'
    )

    gt_extractor_node = Node(
        package='robot_sim_bringup',
        executable='gt_robot_pose.py',
        name='gt_robot_pose',
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        # slam_launch,
        nav2_launch,
        gz_gt_bridge,
        gt_extractor_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        diff_drive_base_controller_spawner,
    ])