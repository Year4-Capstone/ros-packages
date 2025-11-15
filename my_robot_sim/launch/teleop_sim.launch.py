import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. Find the packages
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 2. Define the path to the F710 config file
    f710_config_file = os.path.join(
        get_package_share_directory('my_robot_sim'),
        'config',
        'F710_sim.yaml'
    )

    # 3. Define the path to your SDF world/robot file
    # Assuming 'building_robot.sdf' is the world file to load
    sdf_file = os.path.join(
        get_package_share_directory('my_robot_sim'),
        'gazebo',
        'building_robot.sdf'
    )

    # 4. Launch Gazebo (using its launch file)
    # We pass the '-r' flag to start the simulation immediately
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Pass the SDF file as an argument to Gazebo
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items()
    )

    # 5. Launch the joy_node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # 6. Launch the teleop_twist_joy node
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[f710_config_file], # Load the controller config
        output='screen'
    )

    # 7. Launch the ROS-Gazebo bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        # Argument bridges ROS /cmd_vel to Gazebo /cmd_vel
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    # Create the final launch description
    return LaunchDescription([
        gz_sim_launch,
        joy_node,
        teleop_node,
        bridge_node
    ])