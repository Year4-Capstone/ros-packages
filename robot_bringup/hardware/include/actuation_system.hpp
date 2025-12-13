// Copyright 2024 Your Name/Organization
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ACTUATION_SYSTEM_HPP_
#define ACTUATION_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "phidgets_motor_driver.hpp"
#include "phidgets_limit_switch.hpp"
#include "std_msgs/msg/bool.hpp" 

namespace ibex_control
{
class ActuationSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ActuationSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

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
  // Command and state storage
  std::vector<double> hw_commands_;      // Target positions (radians)
  std::vector<double> hw_positions_;     // Current positions (radians)
  std::vector<double> hw_velocities_;    // Current velocities (rad/s)
  
  // Last position for velocity calculation
  std::array<double, 4> last_pos_rads_ {{0.0, 0.0, 0.0, 0.0}};
  
  // Per-motor homing flags - true if motor has been homed and is ready for use
  std::array<bool, 4> is_homed_ {{false, false, false, false}};
  // Global homing flag - true only when all motors are homed
  bool all_homed_ = false;

  std::array<bool, 4> limit_triggered_ {{false, false, false, false}};
  
  // Phidget Motor Controllers for actuators
  std::unique_ptr<PhidgetMotorController> motor_fl_;  // Front Left
  std::unique_ptr<PhidgetMotorController> motor_fr_;  // Front Right
  std::unique_ptr<PhidgetMotorController> motor_rl_;  // Rear Left
  std::unique_ptr<PhidgetMotorController> motor_rr_;  // Rear Right

  // Limit switches (optional - for safety monitoring)
  std::unique_ptr<PhidgetLimitSwitch> limit_fl_;
  std::unique_ptr<PhidgetLimitSwitch> limit_fr_;
  std::unique_ptr<PhidgetLimitSwitch> limit_rl_;
  std::unique_ptr<PhidgetLimitSwitch> limit_rr_;

  // Joint indices
  size_t idx_fl_ = 0;  // Front left flipper index
  size_t idx_fr_ = 0;  // Front right flipper index
  size_t idx_rl_ = 0;  // Rear left flipper index
  size_t idx_rr_ = 0;  // Rear right flipper index

  // Position control parameters
  const double Kp_ = 2.0;                    // Proportional gain for position control
  const double POSITION_DEADBAND_ = 0.01;    // Deadband in radians (~0.57 degrees)
  const double MAX_VELOCITY_ = 0.5;          // Maximum velocity in rad/s
  const double MIN_POSITION_ = 0.0;          // Minimum position limit (radians)
  const double MAX_POSITION_ = 3.14159;      // Maximum position limit (radians)
  
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr homing_complete_sub_;
};

}  // namespace ibex_control

#endif  // ACTUATION_SYSTEM_HPP_