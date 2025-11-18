// Copyright 2021 ros2_control Development Team
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

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "ros2_control_demo_example_2/diffbot_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_2
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // We expect 4 hardware parameters for the serial numbers and ports of the motors
  // These will be defined in the URDF xacro file.
  const int fl_serial = std::stoi(info_.hardware_parameters["fl_serial_num"]);
  const int fl_port = std::stoi(info_.hardware_parameters["fl_port_num"]);
  const int bl_serial = std::stoi(info_.hardware_parameters["bl_serial_num"]);
  const int bl_port = std::stoi(info_.hardware_parameters["bl_port_num"]);
  const int fr_serial = std::stoi(info_.hardware_parameters["fr_serial_num"]);
  const int fr_port = std::stoi(info_.hardware_parameters["fr_port_num"]);
  const int br_serial = std::stoi(info_.hardware_parameters["br_serial_num"]);
  const int br_port = std::stoi(info_.hardware_parameters["br_port_num"]);

  motor_fl_ = std::make_unique<PhidgetMotorController>(fl_serial, fl_port, MotorType::Drive, 1);
  motor_bl_ = std::make_unique<PhidgetMotorController>(bl_serial, bl_port, MotorType::Drive, 1);
  motor_fr_ = std::make_unique<PhidgetMotorController>(fr_serial, fr_port, MotorType::Drive, -1);
  motor_br_ = std::make_unique<PhidgetMotorController>(br_serial, br_port, MotorType::Drive, -1);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Phidgets motors...");
  try {
    motor_fl_->init();
    motor_bl_->init();
    motor_fr_->init();
    motor_br_->init();
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize motors: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Phidgets motors...");

  // command and state should be equal when starting
  for (uint i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  
  last_pos_rads_.fill(0.0);

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Phidgets motors...");

  // Stop the motors
  motor_fl_->setVelocityRads(0.0);
  motor_bl_->setVelocityRads(0.0);
  motor_fr_->setVelocityRads(0.0);
  motor_br_->setVelocityRads(0.0);

  // Cleanup resources
  motor_fl_->cleanup();
  motor_bl_->cleanup();
  motor_fr_->cleanup();
  motor_br_->cleanup();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// diffbot_system.cpp

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // 1. Get current position from all 4 physical motors
  std::array<double, 4> current_pos_rads;
  current_pos_rads[0] = motor_fl_->getPositionRads(); // FL
  current_pos_rads[1] = motor_bl_->getPositionRads(); // BL
  current_pos_rads[2] = motor_fr_->getPositionRads(); // FR
  current_pos_rads[3] = motor_br_->getPositionRads(); // BR

  double dt = period.seconds();

  // 2. Calculate and assign the aggregated state for the two ROS 2 Control joints
  
  // --- LEFT WHEEL JOINT (Index 0) ---
  
  // Calculate change in position for FL and BL motors
  double delta_pos_fl = current_pos_rads[0] - last_pos_rads_[0];
  double delta_pos_bl = current_pos_rads[1] - last_pos_rads_[1];
  
  // Calculate current velocity for FL and BL motors
  double vel_fl = delta_pos_fl / dt;
  double vel_bl = delta_pos_bl / dt;
  
  // Aggregate position and velocity for the left joint
  hw_positions_[0] = (current_pos_rads[0] + current_pos_rads[1]) / 2.0; // Average position
  hw_velocities_[0] = (vel_fl + vel_bl) / 2.0;                         // Average velocity

  
  // --- RIGHT WHEEL JOINT (Index 1) ---
  
  // Calculate change in position for FR and BR motors
  double delta_pos_fr = current_pos_rads[2] - last_pos_rads_[2];
  double delta_pos_br = current_pos_rads[3] - last_pos_rads_[3];
  
  // Calculate current velocity for FR and BR motors
  double vel_fr = delta_pos_fr / dt;
  double vel_br = delta_pos_br / dt;

  // Aggregate position and velocity for the right joint
  hw_positions_[1] = (current_pos_rads[2] + current_pos_rads[3]) / 2.0; // Average position
  hw_velocities_[1] = (vel_fr + vel_br) / 2.0;                         // Average velocity


  // 3. Store current positions as 'last' for the next cycle's velocity calculation
  last_pos_rads_[0] = current_pos_rads[0];
  last_pos_rads_[1] = current_pos_rads[1];
  last_pos_rads_[2] = current_pos_rads[2];
  last_pos_rads_[3] = current_pos_rads[3];


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_2 ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // The diff_drive_controller will provide velocity commands for 'left_wheel_joint' and 'right_wheel_joint'.
  // We need to send these commands to the four motors.
  // We assume joint 0 is left and joint 1 is right.
  const double left_vel_cmd = hw_commands_[0];
  const double right_vel_cmd = hw_commands_[1];

  motor_fl_->setVelocityRads(left_vel_cmd);
  motor_bl_->setVelocityRads(left_vel_cmd);
  motor_fr_->setVelocityRads(right_vel_cmd);
  motor_br_->setVelocityRads(right_vel_cmd);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::DiffBotSystemHardware, hardware_interface::SystemInterface)
