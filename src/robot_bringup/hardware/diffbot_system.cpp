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

#include "include/diffbot_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ibex_control
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

  for (size_t i = 0; i < info_.joints.size(); i++) {
    std::string name = info_.joints[i].name;
    if (name == "front_left_wheel_joint") idx_fl_ = i;
    else if (name == "rear_left_wheel_joint") idx_rl_ = i;
    else if (name == "front_right_wheel_joint") idx_fr_ = i;
    else if (name == "rear_right_wheel_joint") idx_rr_ = i;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

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

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
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
  // Read current positions from motors
  double pos_fl = motor_fl_->getPositionRads();
  double pos_bl = motor_bl_->getPositionRads();
  double pos_fr = -1 * motor_fr_->getPositionRads();
  double pos_br = -1 * motor_br_->getPositionRads();
  
  // Store positions in hardware interface arrays
  hw_positions_[idx_fl_] = pos_fl;
  hw_positions_[idx_rl_] = pos_bl;
  hw_positions_[idx_fr_] = pos_fr;
  hw_positions_[idx_rr_] = pos_br;
  
  // Calculate velocities using finite difference: velocity = (current_pos - last_pos) / dt
  double dt = period.seconds();
  if (dt > 0.0) {
    hw_velocities_[idx_fl_] = (pos_fl - last_pos_rads_[0]) / dt;
    hw_velocities_[idx_rl_] = (pos_bl - last_pos_rads_[1]) / dt;
    hw_velocities_[idx_fr_] = (pos_fr - last_pos_rads_[2]) / dt;
    hw_velocities_[idx_rr_] = (pos_br - last_pos_rads_[3]) / dt;
  }
  
  // Update last position for next iteration
  last_pos_rads_[0] = pos_fl;
  last_pos_rads_[1] = pos_bl;
  last_pos_rads_[2] = pos_fr;
  last_pos_rads_[3] = pos_br;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ibex_control ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // The diff_drive_controller will provide velocity commands for 'left_wheel_joint' and 'right_wheel_joint'.
  // We need to send these commands to the four motors.
  // We assume joint 0 is left and joint 1 is right.
  motor_fl_->setVelocityRads(hw_commands_[idx_fl_]);
  motor_bl_->setVelocityRads(hw_commands_[idx_rl_]);
  motor_fr_->setVelocityRads(hw_commands_[idx_fr_]);
  motor_br_->setVelocityRads(hw_commands_[idx_rr_]);

  return hardware_interface::return_type::OK;
}

}  // namespace ibex_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ibex_control::DiffBotSystemHardware, hardware_interface::SystemInterface)
