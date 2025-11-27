# ros-packages

ros2 control extra bit articulated robots
change wheels to speheres to remove firction?

# Installs
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
sudo apt install -y libphidget22


# Build and Run
rm -rf build install log
colcon build
source install/setup.bash
ros2 launch robot_bringup ibex.launch.py
or
ros2 launch robot_sim_bringup ibex.launch.py

# For slam too
Only on system not from launch file rn and only with odom and lidar
ros2 launch slam_toolbox online_async_launch.py

SLAM ON ROBOT
Lidar
ros2 run urg_node urg_node_driver --ros-args   -p serial_port:=/dev/ttyACM0 -p serial_baud:=115200 -p frame_id:=laser

Slam
ros2 launch slam_toolbox online_async_launch.py \
    odom:=/diff_drive_base_controller/odom

Control
ros2 launch robot_teleop teleop_joy_launch.py

Robot
ros2 launch robot_bringup ibex.launch.py



## For sim
colcon build --packages-select robot_sim_description robot_sim_bringup


Running controller in other terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel -p stamped:=true


## For repeated motion
ros2 bag record /cmd_vel
ros2 bag play my_motion.bag --loop

Record this for analysis
ros2 bag record /scan /odom /map /tf /tf_static /slam_toolbox/pose /ground_truth/pose

Extracting ground truth
pip install git+https://github.com/MichaelGrupp/evo.git
evo_traj bag2 lidar_in_center_results /pose --save_as_tum

Need to run bridge
ros2 run ros_gz_bridge parameter_bridge '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'


This is ground truth
ros2 topic echo /world/default/dynamic_pose/info --once



## SLAM

## Install and setup
sudo apt install ros-jazzy-urg-node
sudo usermod -a -G dialout $USER

1. LOG OUT AND LOG BACK IN
2. Power lidar
3. Plug in the lidar

### Terminal 1
ros2 run urg_node urg_node_driver --ros-args   -p serial_port:=/dev/ttyACM0 -p serial_baud:=115200 -p frame_id:=laser

### Terminal 2
ros2 run robot_drivers scan_to_3d

### Terminal 3
ros2 launch robot_drivers um7_rviz.launch.py

### Terminal Each
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser


## FOR SLAM
NOT USED
sudo apt install ros-jazzy-slam-toolbox
