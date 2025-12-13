git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd ~/Projects/search-support-robots/ros-packages
colcon build

colcon build --allow-overriding behaviortree_cpp
colcon build --symlink-install --allow-overriding behaviortree_cpp --packages-select behavior-tree 
colcon build --symlink-install --cmake-clean-cache --packages-select behavior-tree--cmake-clean-cache
colcon build --symlink-install --cmake-clean-cache --packages-select behavior-tree --cmake-args -DCMAKE_BUILD_TYPE=Debug


To avoid seg fault
rm -rf build install log
colcon build --symlink-install --cmake-clean-cache --packages-select behavior-tree
source install/setup.bash
ros2 run behavior-tree run_tree

# TO RUN
colcon build --packages-select behavior-tree
source install/setup.bash
ros2 run behavior-tree search_support_bt_runner

### Testing in Gazebo with RViz
Setup lifecycle manager
ros2 lifecycle set /bt_navigator configure
ros2 lifecycle set /bt_navigator activate

ros2 lifecycle set /planner_server configure
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
ros2 lifecycle set /recoveries_server configure
ros2 lifecycle set /recoveries_server activate

Ad this to Nav2 parameters

lifecycle_manager_navigation:
  ros__parameters:
    autostart: true
    node_names:
      - map_saver
      - behavior_server
      - planner_server
      - controller_server
      - smoother_server
      - recoveries_server
      - bt_navigator
      - docking_server

Or run this in terminal

for node in /behavior_server /smoother_server /docking_server; do 
  ros2 lifecycle set $node configure; 
  ros2 lifecycle set $node activate; 
done

Then run this

ros2 lifecycle set /bt_navigator activate



First terminal (This command seems sufficent alone)
export TURTLEBOT3_MODEL=burger
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True headless:=False

In second terminal (to fix sync)
source /opt/ros/jazzy/setup.bash
ros2 param set /rviz use_sim_time true
ros2 param set /slam_toolbox use_sim_time true
ros2 param set /bt_navigator use_sim_time true
ros2 param set /controller_server use_sim_time true
ros2 param set /planner_server use_sim_time true
ros2 param set /behavior_server use_sim_time true
ros2 param set /waypoint_follower use_sim_time true
ros2 param set /velocity_smoother use_sim_time true
