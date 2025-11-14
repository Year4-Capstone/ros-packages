// actuation_system.hpp
#ifndef ROS2_CONTROL_DEMO_EXAMPLE_2__ACTUATION_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_EXAMPLE_2__ACTUATION_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "phidgets_test/phidgets_motor_driver.hpp"

namespace ros2_control_demo_example_2
{
class ActuationSystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // 4 actuation motors
  std::unique_ptr<PhidgetMotorController> motor_act_0_;
  std::unique_ptr<PhidgetMotorController> motor_act_1_;
  std::unique_ptr<PhidgetMotorController> motor_act_2_;
  std::unique_ptr<PhidgetMotorController> motor_act_3_;
};

}  // namespace ros2_control_demo_example_2
#endif