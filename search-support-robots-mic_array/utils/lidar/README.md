## Install and setup
sudo apt install ros-jazzy-urg-node
sudo usermod -a -G dialout $USER

1. LOG OUT AND LOG BACK IN
2. Power lidar
3. Plug in the lidar

### Terminal 1
ros2 run urg_node urg_node_driver --ros-args   -p serial_port:=/dev/ttyACM0 -p serial_baud:=115200 -p frame_id:=laser

### Terminal 2
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map laser

### Terminal 3
ros2 run rviz2 rviz2
IN RVIZ
Fixed Frame - laser
Add --> by topic --> LaserScan

