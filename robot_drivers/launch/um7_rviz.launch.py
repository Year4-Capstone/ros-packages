from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_drivers')
    rviz_config = os.path.join(pkg_share, 'rviz', 'um7_display.rviz')

    return LaunchDescription([
        # UM7 driver
        Node(
            package='robot_drivers',
            executable='um7_node',
            name='um7_node',
            output='screen'
        ),

        # Static TF so imu_link exists
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='imu_tf',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'imu_link'],
        # ),

        # RVIZ2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
