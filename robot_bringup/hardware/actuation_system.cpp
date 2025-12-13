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
#include <algorithm>

#include "include/actuation_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ibex_control
{
hardware_interface::CallbackReturn ActuationSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "Initializing Actuation System Hardware...");

  // Parse hardware parameters from URDF
  try {
    const int fl_serial = std::stoi(info_.hardware_parameters["fl_serial_num"]);
    const int fl_port = std::stoi(info_.hardware_parameters["fl_port_num"]);
    const int fr_serial = std::stoi(info_.hardware_parameters["fr_serial_num"]);
    const int fr_port = std::stoi(info_.hardware_parameters["fr_port_num"]);
    const int rl_serial = std::stoi(info_.hardware_parameters["rl_serial_num"]);
    const int rl_port = std::stoi(info_.hardware_parameters["rl_port_num"]);
    const int rr_serial = std::stoi(info_.hardware_parameters["rr_serial_num"]);
    const int rr_port = std::stoi(info_.hardware_parameters["rr_port_num"]);

    // Create motor controller instances with Actuation motor type
    // Polarity set to +1 - adjust if needed based on your mechanism
    motor_fl_ = std::make_unique<PhidgetMotorController>(fl_serial, fl_port, MotorType::Actuation, +1);
    motor_fr_ = std::make_unique<PhidgetMotorController>(fr_serial, fr_port, MotorType::Actuation, +1);
    motor_rl_ = std::make_unique<PhidgetMotorController>(rl_serial, rl_port, MotorType::Actuation, +1);
    motor_rr_ = std::make_unique<PhidgetMotorController>(rr_serial, rr_port, MotorType::Actuation, +1);

    RCLCPP_INFO(get_logger(), "Motor controllers created");

    // Optional: Initialize limit switches if parameters provided
    if (info_.hardware_parameters.find("fl_limit_serial_num") != info_.hardware_parameters.end()) {
      const int fl_limit_serial = std::stoi(info_.hardware_parameters["fl_limit_serial_num"]);
      const int fl_limit_port = std::stoi(info_.hardware_parameters["fl_limit_port_num"]);
      const int fr_limit_serial = std::stoi(info_.hardware_parameters["fr_limit_serial_num"]);
      const int fr_limit_port = std::stoi(info_.hardware_parameters["fr_limit_port_num"]);
      const int rl_limit_serial = std::stoi(info_.hardware_parameters["rl_limit_serial_num"]);
      const int rl_limit_port = std::stoi(info_.hardware_parameters["rl_limit_port_num"]);
      const int rr_limit_serial = std::stoi(info_.hardware_parameters["rr_limit_serial_num"]);
      const int rr_limit_port = std::stoi(info_.hardware_parameters["rr_limit_port_num"]);

      limit_fl_ = std::make_unique<PhidgetLimitSwitch>(fl_limit_serial, fl_limit_port);
      limit_fr_ = std::make_unique<PhidgetLimitSwitch>(fr_limit_serial, fr_limit_port);
      limit_rl_ = std::make_unique<PhidgetLimitSwitch>(rl_limit_serial, rl_limit_port);
      limit_rr_ = std::make_unique<PhidgetLimitSwitch>(rr_limit_serial, rr_limit_port);

      RCLCPP_INFO(get_logger(), "Limit switches created");
    }

  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to parse hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // ActuationSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
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

  // Map joint names to indices
  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::string name = info_.joints[i].name;
    if (name == "front_left_flipper_flipper_joint") {
      idx_fl_ = i;
      RCLCPP_INFO(get_logger(), "Front left flipper joint at index %zu", i);
    }
    else if (name == "front_right_flipper_flipper_joint") {
      idx_fr_ = i;
      RCLCPP_INFO(get_logger(), "Front right flipper joint at index %zu", i);
    }
    else if (name == "rear_left_flipper_flipper_joint") {
      idx_bl_ = i;
      RCLCPP_INFO(get_logger(), "Rear left flipper joint at index %zu", i);
    }
    else if (name == "rear_right_flipper_flipper_joint") {
      idx_br_ = i;
      RCLCPP_INFO(get_logger(), "Rear right flipper joint at index %zu", i);
    }
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(get_logger(), "Actuation System Hardware initialized successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuationBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Phidgets motors...");
  try {
    motor_fl_->init();
    motor_bl_->init();
    motor_fr_->init();
    motor_br_->init();
    RCLCPP_INFO(get_logger(), "All actuator motors initialized");

    limit_fl_->init();
    limit_fr_->init();
    limit_rl_->init();
    limit_rr_->init();
    RCLCPP_INFO(get_logger(), "All limit switches initialized");
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize motors/switches: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  is_homed_.fill(false);
  RCLCPP_INFO(get_logger(), "Actuation System Hardware configured successfully");

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ActuationSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ActuationSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn ActuationSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Phidgets motors...");

  RCLCPP_INFO(get_logger(), "Waiting for motors to stabilize...");
  rclcpp::sleep_for(std::chrono::milliseconds(500));
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

hardware_interface::CallbackReturn ActuationSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Actuation System Hardware...");
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

  limit_fl_->cleanup();
  limit_fr_->cleanup();
  limit_rl_->cleanup();
  limit_rr_->cleanup();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// actuationbot_system.cpp

hardware_interface::return_type ActuationSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read current positions from motors
  double pos_fl = motor_fl_->getPositionRads();
  double pos_bl = motor_bl_->getPositionRads();
  double pos_fr = motor_fr_->getPositionRads();
  double pos_br = motor_br_->getPositionRads();
  
  // Store positions in hardware interface arrays
  hw_positions_[idx_fl_] = pos_fl;
  hw_positions_[idx_bl_] = pos_bl;
  hw_positions_[idx_fr_] = pos_fr;
  hw_positions_[idx_br_] = pos_br;
  
  // Calculate velocities using finite difference: velocity = (current_pos - last_pos) / dt
  double dt = period.seconds();
  if (dt > 0.0) {
    hw_velocities_[idx_fl_] = (pos_fl - last_pos_rads_[0]) / dt;
    hw_velocities_[idx_bl_] = (pos_bl - last_pos_rads_[1]) / dt;
    hw_velocities_[idx_fr_] = (pos_fr - last_pos_rads_[2]) / dt;
    hw_velocities_[idx_br_] = (pos_br - last_pos_rads_[3]) / dt;
  }
  
  // Update last position for next iteration
  last_pos_rads_[0] = pos_fl;
  last_pos_rads_[1] = pos_bl;
  last_pos_rads_[2] = pos_fr;
  last_pos_rads_[3] = pos_br;

  if (limit_fl_->read()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
      "Front left limit switch triggered!");
  }
  if (limit_fr_->read()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
      "Front right limit switch triggered!");
  }
  if (limit_rl_->read()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
      "Rear left limit switch triggered!");
  }
  if (limit_rr_->read()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
      "Rear right limit switch triggered!");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ibex_control ::ActuationSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Implement simple position control for each motor
  
  // Helper lambda for position control
  auto position_control = [this](size_t idx, PhidgetMotorController* motor) {
    // Clamp commanded position to limits
    double target_pos = std::clamp(hw_commands_[idx], MIN_POSITION_, MAX_POSITION_);
    double current_pos = hw_positions_[idx];
    
    // Calculate position error
    double position_error = target_pos - current_pos;
    
    // Apply deadband - stop if close enough
    double velocity_cmd = 0.0;
    if (std::abs(position_error) > POSITION_DEADBAND_) {
      // Proportional control
      velocity_cmd = Kp_ * position_error;
      
      // Clamp velocity to limits
      velocity_cmd = std::clamp(velocity_cmd, -MAX_VELOCITY_, MAX_VELOCITY_);
    }
    
    // Send velocity command to motor
    motor->setVelocityRads(velocity_cmd);
  };

  // Apply position control to all motors
  position_control(idx_fl_, motor_fl_.get());
  position_control(idx_fr_, motor_fr_.get());
  position_control(idx_rl_, motor_rl_.get());
  position_control(idx_rr_, motor_rr_.get());

  return hardware_interface::return_type::OK;
}

}  // namespace ibex_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ibex_control::ActuationSystemHardware, hardware_interface::SystemInterface)
