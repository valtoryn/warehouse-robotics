#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "valtoryn_task_bt/dock_robot_stub.hpp"
#include "valtoryn_task_bt/nav2_client_node.hpp"
#include "valtoryn_task_bt/payload_stubs.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("valtoryn_task_executor");

  node->declare_parameter<std::string>("tree_xml", "");
  node->declare_parameter<double>("loop_rate_hz", 10.0);
  node->declare_parameter<bool>("autostart", true);

  std::string tree_xml = node->get_parameter("tree_xml").as_string();
  double loop_rate_hz = node->get_parameter("loop_rate_hz").as_double();
  bool autostart = node->get_parameter("autostart").as_bool();

  if (tree_xml.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'tree_xml' must specify the path to a BehaviorTree XML file");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Initializing BehaviorTreeFactory with registered Valtoryn BT nodes...");
  BT::BehaviorTreeFactory factory;

  // 1. Register NavigateToPoseAction with node dependency injection
  BT::NodeBuilder nav2_builder = [node](const std::string & name, const BT::NodeConfig & config) {
    return std::make_unique<valtoryn_task_bt::NavigateToPoseAction>(name, config, node);
  };
  factory.registerBuilder<valtoryn_task_bt::NavigateToPoseAction>("NavigateToPose", nav2_builder);

  // 2. Register Docking and Payload Stubs
  factory.registerNodeType<valtoryn_task_bt::DockRobotStub>("DockRobot");
  factory.registerNodeType<valtoryn_task_bt::PayloadPickStub>("PayloadPickStub");
  factory.registerNodeType<valtoryn_task_bt::PayloadDropStub>("PayloadDropStub");

  // Load Tree
  RCLCPP_INFO(node->get_logger(), "Loading behavior tree from file: %s", tree_xml.c_str());
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(tree_xml);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "Failed to create BehaviorTree: %s", ex.what());
    rclcpp::shutdown();
    return 2;
  }

  if (!autostart) {
    RCLCPP_INFO(node->get_logger(), "Autostart disabled; waiting for external activation");
    rclcpp::spin(node);
    return 0;
  }

  RCLCPP_INFO(node->get_logger(), "Starting mission execution loop (rate: %.1f Hz)...", loop_rate_hz);
  auto start_time = node->now();
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  std::chrono::milliseconds loop_interval(static_cast<int>(1000.0 / loop_rate_hz));

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    std::this_thread::sleep_for(loop_interval);
  }

  auto elapsed_sec = (node->now() - start_time).seconds();
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Mission completed with SUCCESS in %.2f seconds", elapsed_sec);
  } else if (status == BT::NodeStatus::FAILURE) {
    RCLCPP_ERROR(node->get_logger(), "Mission terminated with FAILURE in %.2f seconds", elapsed_sec);
  } else {
    RCLCPP_WARN(node->get_logger(), "Mission interrupted (status: %d)", static_cast<int>(status));
  }

  rclcpp::shutdown();
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 3;
}
