from setuptools import find_packages, setup

package_name = 'robot_drivers'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/um7_rviz.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/um7_display.rviz']),
    ],
    install_requires=['setuptools', 'rsl_comm_py'],
    zip_safe=True,
    maintainer='luke',
    maintainer_email='email@example.com',
    description='Drivers for robot sensors including UM7 IMU',
    entry_points={
        'console_scripts': [
            'um7_node = robot_drivers.um7_node:main',
            'real_time_imu_plot = robot_drivers.real_time_imu_plot:main',
            'scan_to_3d = robot_drivers.lidar_to_3d:main',
        ],
    },
)
