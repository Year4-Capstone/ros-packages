cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(phidgets_test REQUIRED)

find_library(PHIDGET22_LIBRARY phidget22)
if(NOT PHIDGET22_LIBRARY)
  message(FATAL_ERROR "Phidget22 library not found")
endif()

add_library(ibex_control SHARED
  hardware/diffbot_system.cpp
)

ament_target_dependencies(ibex_control
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  phidgets_test
)

target_link_libraries(ibex_control
  ${PHIDGET22_LIBRARY}
)

pluginlib_export_plugin_description_file(hardware_interface ibex_control.xml)

install(TARGETS ibex_control
  DESTINATION lib
)

install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install the library and headers
install(
  TARGETS ibex_control
  EXPORT export_ibex_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES ibex_control.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_ibex_control)
ament_export_dependencies(
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  phidgets_test
)

ament_package()
