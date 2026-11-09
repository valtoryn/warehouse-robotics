#include "valtoryn_task_bt/nav2_client_node.hpp"

#include <cmath>
#include <chrono>

namespace valtoryn_task_bt
{

NavigateToPoseAction::NavigateToPoseAction(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node)
{
  std::string action_name = "navigate_to_pose";
  getInput("action_name", action_name);
  action_client_ = rclcpp_action::create_client<Nav2Goal>(node_, action_name);
}

BT::NodeStatus NavigateToPoseAction::onStart()
{
  double px = 0.0;
  double py = 0.0;
  double yaw = 0.0;

  if (!getInput("position_x", px)) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required port 'position_x'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("position_y", py)) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Missing required port 'position_y'", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  getInput("orientation_yaw", yaw);

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Nav2 NavigateToPose action server unavailable", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  Nav2Goal::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = px;
  goal_msg.pose.pose.position.y = py;
  goal_msg.pose.pose.position.z = 0.0;

  // Convert yaw to quaternion
  goal_msg.pose.pose.orientation.x = 0.0;
  goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] Dispatching NavigateToPose goal: x=%.2f, y=%.2f, yaw=%.2f rad",
    name().c_str(), px, py, yaw);

  auto send_goal_opts = rclcpp_action::Client<Nav2Goal>::SendGoalOptions();
  send_goal_opts.goal_response_callback =
    [this](const GoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Goal was rejected by Nav2 server", name().c_str());
      } else {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal accepted by Nav2 server", name().c_str());
      }
    };

  send_goal_opts.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      result_ = result;
      result_ready_ = true;
    };

  goal_sent_ = false;
  result_ready_ = false;
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_opts);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  if (!goal_sent_) {
    return BT::NodeStatus::FAILURE;
  }

  // Check if goal was accepted
  if (!goal_handle_) {
    if (goal_handle_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      goal_handle_ = goal_handle_future_.get();
      if (!goal_handle_) {
        return BT::NodeStatus::FAILURE;
      }
    } else {
      return BT::NodeStatus::RUNNING;
    }
  }

  // Check if result is available
  if (result_ready_) {
    switch (result_.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "[%s] Navigation goal reached successfully", name().c_str());
        return BT::NodeStatus::SUCCESS;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(node_->get_logger(), "[%s] Navigation goal aborted by Nav2", name().c_str());
        return BT::NodeStatus::FAILURE;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "[%s] Navigation goal canceled", name().c_str());
        return BT::NodeStatus::FAILURE;
      default:
        RCLCPP_ERROR(node_->get_logger(), "[%s] Unknown navigation action result code", name().c_str());
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "[%s] BehaviorTree halted: canceling active Nav2 goal", name().c_str());
    action_client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace valtoryn_task_bt

