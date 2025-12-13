cmake_minimum_required(VERSION 3.8)
project(behavior-tree)

# -------------------------------------------------
# C++ standard
# -------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------------------------
# Dependencies
# -------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_executable(test_rclcpp_node src/test_rclcpp_node.cpp)
ament_target_dependencies(test_rclcpp_node
  rclcpp
)
install(TARGETS test_rclcpp_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(search_support_bt_runner src/search_and_support_nodes.cpp)

ament_target_dependencies(search_support_bt_runner
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  behaviortree_cpp
  ament_index_cpp
)

install(TARGETS search_support_bt_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  behavior_trees/search_and_support.xml
  behavior_trees/SearchAndSupport.btproj
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_package()
