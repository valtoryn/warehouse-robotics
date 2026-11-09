#ifndef VALTORYN_TASK_BT__NAV2_CLIENT_NODE_HPP_
#define VALTORYN_TASK_BT__NAV2_CLIENT_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace valtoryn_task_bt
{

class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  using Nav2Goal = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Nav2Goal>;

  NavigateToPoseAction(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("position_x", 0.0, "Target X coordinate in meters"),
      BT::InputPort<double>("position_y", 0.0, "Target Y coordinate in meters"),
      BT::InputPort<double>("orientation_yaw", 0.0, "Target Yaw orientation in radians"),
      BT::InputPort<std::string>("action_name", "navigate_to_pose", "Nav2 action server topic name"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Nav2Goal>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_handle_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  bool goal_sent_{false};
  bool result_ready_{false};
  GoalHandle::WrappedResult result_;
};

}  // namespace valtoryn_task_bt

#endif  // VALTORYN_TASK_BT__NAV2_CLIENT_NODE_HPP_

