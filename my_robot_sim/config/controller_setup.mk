ros2 run joy joy_node

ros2 run teleop_twist_joy teleop_node --ros-args --params-file /home/callum/capstone/ros2_test_ws/src/config/8bitdo.config.yaml

ros2 topic echo /cmd_vel

ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

gz sim building_robot.sdf building_robot.sdf 


need ros_gz_bridge

Phidgets setup
sudo apt-get install ros-<ros_distro>-phidgets-drivers


ros2 run teleop_twist_joy teleop_node --ros-args --params-file /home/callum/capstone/ros2_test_ws/src/config/8bitdo.config.yaml -r /cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true  