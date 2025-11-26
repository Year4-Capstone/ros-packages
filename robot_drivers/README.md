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

