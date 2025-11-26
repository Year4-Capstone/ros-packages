cmake_minimum_required(VERSION 3.8)
project(phidgets_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(PHIDGET22_LIBRARY phidget22)

# Generate actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/HomingSequence.action"
  DEPENDENCIES builtin_interfaces
)

# Obtain the generated typesupport target so we can add it as a dependency
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")


# NOTE: generated action/message headers are created in
# ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp. We add per-target include
# directories and add_dependencies on the generated interfaces target so the
# headers are available when compiling the node and client.


include_directories(include)

add_executable(test_phidgets_motors src/test_phidgets_motor_driver.cpp)
ament_target_dependencies(test_phidgets_motors rclcpp)

target_link_libraries(test_phidgets_motors
  ${PHIDGET22_LIBRARY}
)

add_executable(test_bldc_motors src/test_bldc_motors.cpp)
ament_target_dependencies(test_bldc_motors rclcpp)

add_executable(motor_sub src/motor_sub.cpp)
ament_target_dependencies(motor_sub rclcpp geometry_msgs)

add_executable(actuation_sub src/actuation_sub.cpp)
ament_target_dependencies(actuation_sub rclcpp sensor_msgs)

add_executable(limit_switch_test src/test_limit_switch.cpp)
ament_target_dependencies(limit_switch_test rclcpp)
target_link_libraries(limit_switch_test
  ${PHIDGET22_LIBRARY}
)

add_executable(actuation_homing_node src/actuation_homing_node.cpp)
ament_target_dependencies(actuation_homing_node 
  rclcpp 
  rclcpp_action)
add_dependencies(actuation_homing_node ${cpp_typesupport_target})
target_include_directories(actuation_homing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_link_libraries(actuation_homing_node
  ${cpp_typesupport_target}
)

add_executable(homing_client src/homing_client.cpp)
ament_target_dependencies(homing_client
  rclcpp
  rclcpp_action)
add_dependencies(homing_client ${cpp_typesupport_target})
target_include_directories(homing_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_link_libraries(homing_client
  ${cpp_typesupport_target}
)

target_link_libraries(test_bldc_motors
  ${PHIDGET22_LIBRARY}
)

target_link_libraries(actuation_homing_node
  ${PHIDGET22_LIBRARY}
)

target_link_libraries(motor_sub
  ${PHIDGET22_LIBRARY}
)

target_link_libraries(actuation_sub
  ${PHIDGET22_LIBRARY}
)

install(PROGRAMS
  src/plot_homing_positions.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  test_bldc_motors
  motor_sub
  actuation_sub
  test_phidgets_motors
  actuation_homing_node
  homing_client
  limit_switch_test
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()
