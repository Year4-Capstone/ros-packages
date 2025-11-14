// actuation_system.cpp
#include "ros2_control_demo_example_2/actuation_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_example_2
{

hardware_interface::CallbackReturn ActuationSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get hardware parameters for 4 actuation motors
  const int act0_serial = std::stoi(info_.hardware_parameters["act0_serial_num"]);
  const int act0_port = std::stoi(info_.hardware_parameters["act0_port_num"]);
  const int act1_serial = std::stoi(info_.hardware_parameters["act1_serial_num"]);
  const int act1_port = std::stoi(info_.hardware_parameters["act1_port_num"]);
  const int act2_serial = std::stoi(info_.hardware_parameters["act2_serial_num"]);
  const int act2_port = std::stoi(info_.hardware_parameters["act2_port_num"]);
  const int act3_serial = std::stoi(info_.hardware_parameters["act3_serial_num"]);
  const int act3_port = std::stoi(info_.hardware_parameters["act3_port_num"]);

  motor_act_0_ = std::make_unique<PhidgetMotorController>(act0_serial, act0_port, MotorType::Actuation, 1);
  motor_act_1_ = std::make_unique<PhidgetMotorController>(act1_serial, act1_port, MotorType::Actuation, 1);
  motor_act_2_ = std::make_unique<PhidgetMotorController>(act2_serial, act2_port, MotorType::Actuation, 1);
  motor_act_3_ = std::make_unique<PhidgetMotorController>(act3_serial, act3_port, MotorType::Actuation, 1);

  // Validate joints (expect 4 actuation joints)
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has %zu state interfaces. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' first state interface is '%s'. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' second state interface is '%s'. '%s' expected.",
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

hardware_interface::CallbackReturn ActuationSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring actuation motors...");
  
  try {
    motor_act_0_->init();
    motor_act_1_->init();
    motor_act_2_->init();
    motor_act_3_->init();
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize actuation motors: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset values
  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(get_logger(), "Actuation motors configured successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuationSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating actuation motors...");

  for (size_t i = 0; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(get_logger(), "Actuation motors activated successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ActuationSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating actuation motors...");

  // Stop all motors
  motor_act_0_->setVelocityRads(0.0);
  motor_act_1_->setVelocityRads(0.0);
  motor_act_2_->setVelocityRads(0.0);
  motor_act_3_->setVelocityRads(0.0);

  // Cleanup
  motor_act_0_->cleanup();
  motor_act_1_->cleanup();
  motor_act_2_->cleanup();
  motor_act_3_->cleanup();

  RCLCPP_INFO(get_logger(), "Actuation motors deactivated successfully!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ActuationSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Simulate position based on commanded velocity
  // TODO: Replace with actual encoder readings when available
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_velocities_[i] = hw_commands_[i];
    hw_positions_[i] += period.seconds() * hw_velocities_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ActuationSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send velocity commands to each actuation motor
  motor_act_0_->setVelocityRads(hw_commands_[0]);
  motor_act_1_->setVelocityRads(hw_commands_[1]);
  motor_act_2_->setVelocityRads(hw_commands_[2]);
  motor_act_3_->setVelocityRads(hw_commands_[3]);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_2::ActuationSystemHardware, 
  hardware_interface::SystemInterface)