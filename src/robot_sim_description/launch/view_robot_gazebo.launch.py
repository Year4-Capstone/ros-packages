import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import AppendEnvironmentVariable
from launch.actions import GroupAction

def generate_launch_description():
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='cave_world.world',
        description='World name (must match worlds/<name>.world and models/<name>/)'
    )

    world_name = LaunchConfiguration('world_name')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([
        pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'
    ])

    pkg_desc = get_package_share_directory('robot_sim_description')

    world_path = PathJoinSubstitution([
        pkg_desc,
        'worlds',
        world_name
    ])

    urdf_path = os.path.join(
        pkg_desc, 'urdf', 'cad_urdf.urdf.xacro'
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([
            FindPackageShare('robot_sim_description'),
            'models'
        ])
    )

    robot_description = {
        'robot_description': Command([
            'xacro ', urdf_path, ' use_gazebo:=true'
        ])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_path]
        }.items()
    )

    gazebo_group = GroupAction([
        set_gz_resource_path,
        gazebo,
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ibex_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_name_arg,
        gazebo_group,
        robot_state_publisher_node,
        spawn_node,
        bridge_node,
    ])
