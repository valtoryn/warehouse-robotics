#include "valtoryn_diff_drive/diff_drive_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace valtoryn_diff_drive
{

CallbackReturn DiffDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read hardware parameters from URDF <ros2_control> tag
  auto mock_it = info_.hardware_parameters.find("mock_hardware");
  if (mock_it != info_.hardware_parameters.end())
  {
    mock_hardware_ = (mock_it->second == "true" || mock_it->second == "1");
  }
  else
  {
    mock_hardware_ = true;  // Default to mock mode for safe simulation / bringup
  }

  auto can_it = info_.hardware_parameters.find("can_interface");
  if (can_it != info_.hardware_parameters.end())
  {
    can_interface_ = can_it->second;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints)
  {
    // Validate command interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveHardware"),
        "Joint '%s' command interface '%s' found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    // Validate state interfaces (position + velocity)
    if (joint.state_interfaces.size() < 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveHardware"),
        "Joint '%s' has %zu state interfaces found. At least 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
  }

  hw_commands_.resize(joint_names_.size(), 0.0);
  hw_positions_.resize(joint_names_.size(), 0.0);
  hw_velocities_.resize(joint_names_.size(), 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveHardware"),
    "Initialized DiffDriveHardware with %zu joints (mock_hardware: %s)",
    joint_names_.size(), mock_hardware_ ? "true" : "false");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

CallbackReturn DiffDriveHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    hw_commands_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "DiffDriveHardware activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    hw_commands_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveHardware"), "DiffDriveHardware deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (mock_hardware_)
  {
    double dt = period.seconds();
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      hw_positions_[i] += hw_velocities_[i] * dt;
    }
  }
  else
  {
    // Physical hardware backend: read encoder ticks and velocities via CAN
    // (Hardware implementation path for SocketCAN / CiA 402)
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (mock_hardware_)
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      hw_velocities_[i] = hw_commands_[i];
    }
  }
  else
  {
    // Physical hardware backend: transmit velocity setpoints to CAN drives
  }
  return hardware_interface::return_type::OK;
}

}  // namespace valtoryn_diff_drive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  valtoryn_diff_drive::DiffDriveHardware, hardware_interface::SystemInterface)
