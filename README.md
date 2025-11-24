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

## For sim
colcon build --packages-select robot_sim_description robot_sim_bringup

Running controller in other terminal:
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_base_controller/cmd_vel -p stamped:=true
