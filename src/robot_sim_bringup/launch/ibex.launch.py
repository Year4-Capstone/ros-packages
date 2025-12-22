import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap


def generate_launch_description():

    # =========================
    # Package paths
    # =========================
    pkg_robot_description = FindPackageShare('robot_sim_description')
    pkg_robot_bringup = FindPackageShare('robot_sim_bringup')

    def nav2_cfg(name):
        return PathJoinSubstitution([
            pkg_robot_bringup,
            'config',
            'nav2',
            name
        ])

    controller_config_file = PathJoinSubstitution(
        [pkg_robot_bringup, 'config', 'diff_drive_controller.yaml']
    )

    gazebo_launch_path = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'view_robot_gazebo.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [pkg_robot_description, 'rviz', 'ibex.rviz']
    )

    twist_mux_topics = PathJoinSubstitution(
        [pkg_robot_bringup, 'config', 'twist_mux', 'twist_mux_topics.yaml'
    ])


    # =========================
    # Gazebo
    # =========================
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

    # =========================
    # RViz
    # =========================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # =========================
    # ROS 2 Control
    # =========================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            controller_config_file
        ],
        output='screen',
    )

    # =========================
    # Ground truth pose (Gazebo)
    # =========================
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

    # =========================
    # Twist Mux Node
    # =========================
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_topics,
            {'use_sim_time': True},
        ],
        remappings=[
            ('cmd_vel_out', '/diff_drive_base_controller/cmd_vel')
        ]
    )

    # =========================
    # Nav2 (MANUAL bringup)
    # =========================
    nav2_launch = GroupAction(actions=[

        # --- Map ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_cfg('map_server.yaml')],
        ),

        # --- AMCL ---
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_cfg('amcl.yaml')],
        ),

        # --- Planner ---
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[
                nav2_cfg('planner_server.yaml'),
                nav2_cfg('global_costmap.yaml'),
            ],
        ),

        # --- Controller ---
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                nav2_cfg('controller_server.yaml'),
                nav2_cfg('local_costmap.yaml'),
            ],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav_stamped')
            ],
        ),

        # --- Behaviors ---
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_cfg('behavior_server.yaml')],
        ),

        # --- BT Navigator ---
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_cfg('bt_navigator.yaml')],
        ),

        # --- Lifecycle Manager ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),
    ])

    # =========================
    # Launch description
    # =========================
    return LaunchDescription([
        gazebo_sim,
        gz_gt_bridge,
        gt_extractor_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        diff_drive_base_controller_spawner,
        twist_mux_node,
        nav2_launch,
    ])
