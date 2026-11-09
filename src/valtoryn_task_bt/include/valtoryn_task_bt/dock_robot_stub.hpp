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
    RCLCPP_INFO(
      rclcpp::get_logger("DockRobotStub"),
      "[%s] Simulating docking alignment to station '%s' (hardware docking actuator unverified; no physical dock sensor attached)",
      name().c_str(), station_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace valtoryn_task_bt

#endif  // VALTORYN_TASK_BT__DOCK_ROBOT_STUB_HPP_
