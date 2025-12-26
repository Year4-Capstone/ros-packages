This should maybe be in its own package later. But for now, this is the most effective place.

Command to save a new slam map after running slam launch
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
"{name: {data: '/home/luke/Projects/capstone/search-support-robots/ros-packages/robot_sim_bringup/maps/cave_world/map'}}"

