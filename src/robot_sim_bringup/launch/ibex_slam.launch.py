import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap


def generate_launch_description():
    pkg_robot_description = FindPackageShare('robot_sim_description')
    pkg_robot_bringup = FindPackageShare('robot_sim_bringup')

    controller_config_file = PathJoinSubstitution(
        [pkg_robot_bringup, 'config', 'diff_drive_controller.yaml']
    )

    gazebo_launch_path = PathJoinSubstitution(
        [pkg_robot_description, 'launch', 'view_robot_gazebo.launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [pkg_robot_description, 'rviz', 'ibex.rviz']
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                pkg_robot_bringup, 'config', 'slam_params.yaml'
            ])
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

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
        slam_launch,
        gazebo_sim,
        gz_gt_bridge,
        gt_extractor_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        diff_drive_base_controller_spawner
    ])
