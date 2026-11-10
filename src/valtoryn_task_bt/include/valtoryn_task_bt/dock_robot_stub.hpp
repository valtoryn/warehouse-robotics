#ifndef VALTORYN_TASK_BT__DOCK_ROBOT_STUB_HPP_
#define VALTORYN_TASK_BT__DOCK_ROBOT_STUB_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace valtoryn_task_bt
{

class DockRobotStub : public BT::SyncActionNode
{
public:
  DockRobotStub(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("station_id", "station_pick_1", "Target station dock identifier"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string station_id = "unknown";
    getInput("station_id", station_id);
    RCLCPP_ERROR(
      rclcpp::get_logger("DockRobotStub"),
      "[%s] Non-functional docking stub invoked at station '%s': "
      "hardware docking actuator and dock sensors are not installed; docking capability unverified. "
      "Failing task to prevent false-positive docking claim.",
      name().c_str(), station_id.c_str());
    return BT::NodeStatus::FAILURE;
  }
};

}  // namespace valtoryn_task_bt

#endif  // VALTORYN_TASK_BT__DOCK_ROBOT_STUB_HPP_

