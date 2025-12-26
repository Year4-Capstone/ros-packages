# To get Gazebo qithout models working

## Good place for models
git clone https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps
git clone https://github.com/osrf/gazebo_models ~/.gazebo/models

## Example of getting small house working
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world ~/aws_models

Test the models
export GZ_SIM_RESOURCE_PATH=\
~/aws_models/models:\
$GZ_SIM_RESOURCE_PATH

Test world in gazebo
gz sim /home/luke/Projects/capstone/search-support-robots/ros-packages/robot_sim_description/world/small_house.world

Make permanant
echo 'export GZ_SIM_RESOURCE_PATH=~/aws_models/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
