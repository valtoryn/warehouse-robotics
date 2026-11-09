#include "valtoryn_diff_drive/diff_drive_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace valtoryn_diff_drive
{

CallbackReturn DiffDriveHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
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

  // Enforce fail-closed safety rule: physical backend is not yet implemented.
  if (!mock_hardware_)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveHardware"),
      "Physical hardware backend (CAN / CiA 402) is not yet implemented. "
      "Set <param name=\"mock_hardware\">true</param> for software loopback or "
      "use gz_ros2_control/GazeboSimSystem for Gazebo simulation.");
    return CallbackReturn::ERROR;
  }

  auto can_it = info_.hardware_parameters.find("can_interface");
  if (can_it != info_.hardware_parameters.end())
  {
    can_interface_ = can_it->second;
  }

  if (info_.joints.empty())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveHardware"),
      "No joints configured in <ros2_control> description.");
    return CallbackReturn::ERROR;
  }

  joint_names_.clear();
  std::set<std::string> unique_names;

  for (const auto & joint : info_.joints)
  {
    if (!unique_names.insert(joint.name).second)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveHardware"),
        "Duplicate joint name '%s' in <ros2_control> description.", joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    // Validate command interface: exactly one velocity command interface required
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveHardware"),
        "Joint '%s' has %zu command interfaces found. Exactly 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
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

    // Validate state interfaces: must provide both position and velocity
    bool has_position = false;
    bool has_velocity = false;
    for (const auto & si : joint.state_interfaces)
    {
      if (si.name == hardware_interface::HW_IF_POSITION)
      {
        has_position = true;
      }
      else if (si.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity = true;
      }
    }

    if (!has_position || !has_velocity)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveHardware"),
        "Joint '%s' missing required state interface(s) (has_position=%s, has_velocity=%s).",
        joint.name.c_str(), has_position ? "true" : "false", has_velocity ? "true" : "false");
      return CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
  }

  hw_commands_.assign(joint_names_.size(), 0.0);
  hw_positions_.assign(joint_names_.size(), 0.0);
  hw_velocities_.assign(joint_names_.size(), 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("DiffDriveHardware"),
    "Initialized DiffDriveHardware with %zu joints (mock: %s, CAN: %s)",
    joint_names_.size(), mock_hardware_ ? "true" : "false", can_interface_.c_str());

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
  if (!mock_hardware_)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveHardware"),
      "Cannot activate DiffDriveHardware: physical driver communication is unimplemented.");
    return CallbackReturn::ERROR;
  }

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
  if (!mock_hardware_)
  {
    return hardware_interface::return_type::ERROR;
  }

  double dt = period.seconds();
  if (std::isfinite(dt) && dt > 0.0)
  {
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (std::isfinite(hw_velocities_[i]))
      {
        hw_positions_[i] += hw_velocities_[i] * dt;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!mock_hardware_)
  {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (std::isfinite(hw_commands_[i]))
    {
      hw_velocities_[i] = hw_commands_[i];
    }
    else
    {
      hw_velocities_[i] = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace valtoryn_diff_drive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  valtoryn_diff_drive::DiffDriveHardware, hardware_interface::SystemInterface)

