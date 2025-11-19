Commands

# Use defaults (stamped=true, sim_time=true, topic=/diff_drive_controller/cmd_vel)
ros2 launch robot_teleop your_launch_file.launch.py

# All custom options
ros2 launch robot_teleop your_launch_file.launch.py \
    publish_stamped:=false \
    use_sim_time:=false \
    cmd_vel_topic:=/mobile_base/cmd_vel