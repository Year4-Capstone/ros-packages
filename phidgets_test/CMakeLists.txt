cmake_minimum_required(VERSION 3.8)
project(phidgets_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_library(PHIDGET22_LIBRARY phidget22)


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

target_link_libraries(test_bldc_motors
  ${PHIDGET22_LIBRARY}
)

target_link_libraries(motor_sub
  ${PHIDGET22_LIBRARY}
)

target_link_libraries(actuation_sub
  ${PHIDGET22_LIBRARY}
)


install(TARGETS
  test_bldc_motors
  motor_sub
  actuation_sub
  test_phidgets_motors
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
