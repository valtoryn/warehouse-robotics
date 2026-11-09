#ifndef VALTORYN_TASK_BT__PAYLOAD_STUBS_HPP_
#define VALTORYN_TASK_BT__PAYLOAD_STUBS_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace valtoryn_task_bt
{

class PayloadPickStub : public BT::SyncActionNode
{
public:
  PayloadPickStub(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("station_id", "station_pick_1", "Pickup station ID"),
      BT::InputPort<std::string>("payload_id", "tote_standard", "Payload serial / item identifier"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string station_id = "unknown";
    std::string payload_id = "unknown";
    getInput("station_id", station_id);
    getInput("payload_id", payload_id);

    RCLCPP_WARN(
      rclcpp::get_logger("PayloadPickStub"),
      "[%s] Non-functional payload pick stub invoked at station '%s' for item '%s': "
      "material handling hardware is not installed; payload capability unverified (Gate C3 non-functional stub).",
      name().c_str(), station_id.c_str(), payload_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

class PayloadDropStub : public BT::SyncActionNode
{
public:
  PayloadDropStub(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("station_id", "station_drop_1", "Dropoff station ID"),
      BT::InputPort<std::string>("payload_id", "tote_standard", "Payload serial / item identifier"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string station_id = "unknown";
    std::string payload_id = "unknown";
    getInput("station_id", station_id);
    getInput("payload_id", payload_id);

    RCLCPP_WARN(
      rclcpp::get_logger("PayloadDropStub"),
      "[%s] Non-functional payload drop stub invoked at station '%s' for item '%s': "
      "material handling hardware is not installed; payload capability unverified (Gate C3 non-functional stub).",
      name().c_str(), station_id.c_str(), payload_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace valtoryn_task_bt

#endif  // VALTORYN_TASK_BT__PAYLOAD_STUBS_HPP_

