import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_path
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_robot_sim_description = get_package_share_directory('robot_sim_description')    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # world argument    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='small_house',
        description='World to load: small_house, cave_world, or empty_world'
    )
    world_name = LaunchConfiguration('world')

    # Construct the path to the world file
    world_path = PathJoinSubstitution([
        pkg_robot_sim_description,
        'world',
        world_name
    ])

    # Get the package share directory for meshes
    robot_sim_description_parent_path = os.path.dirname(pkg_robot_sim_description)
    world_dir = os.path.join(pkg_robot_sim_description, 'world/models')
    combined_resource_path = f"{robot_sim_description_parent_path}:{world_dir}"
    
    default_paths = [
        os.path.expanduser("~/.gz/models"),
        "/usr/share/gz/models",
    ]

    combined = ":".join(default_paths + [combined_resource_path])

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=combined
    )

    urdf_path = os.path.join(get_package_share_path('robot_sim_description'), 'urdf', 'ibex.urdf.xacro')

    robot_description_content = Command(
        ['xacro ', urdf_path, ' use_gazebo:=true']
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
        parameters=[
            robot_description, 
            {'use_sim_time': True}
        ]
    )

    
    # Spawns the robot model into the running Gazebo simulation
    gazebo_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ibex_robot',
            '-topic', 'robot_description', # Reads the URDF from the parameter set above
            '-x', '0', '-y', '0', '-z', '0.5' # Initial pose
        ],
        parameters=[{'use_sim_time': True}] 
    )
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' 
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Create the final launch description
    return LaunchDescription([
        world_arg,
        set_gz_resource_path, 
        gazebo,
        robot_state_publisher_node,
        gazebo_spawn_entity_node,
        bridge_node
    ])