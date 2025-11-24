import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    world_path = os.path.join(get_package_share_path('robot_sim_description'), 'world', 'empty_world.sdf')
    urdf_path = os.path.join(get_package_share_path('robot_sim_description'), 'urdf', 'ibex.urdf.xacro')

    robot_description_content = Command(
        [
            'xacro ', urdf_path,
            ' use_gazebo:=true' 
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={
                    "gz_args": ['-r -v 4 ', world_path]
                }.items()
             )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    
    # Spawns the robot model into the running Gazebo simulation
    gazebo_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ibex_robot',
            '-topic', 'robot_description', # Reads the URDF from the parameter set above
            '-x', '0', '-y', '0', '-z', '0' # Initial pose
        ]
    )
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ],
        output='screen'
    )

    # Create the final launch description
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        gazebo_spawn_entity_node,
        bridge_node
    ])