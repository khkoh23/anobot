#include <chrono>
#include <memory>
#include <unordered_map>
#include <string>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include "anobot_bt/trigger_service_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mock_pickup_bt_runner");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {
    executor.spin();
  });
  BT::BehaviorTreeFactory factory;
  const std::unordered_map<std::string, std::string>
  service_map = {
    {"ResetRod", "/scene/reset_rod_on_workstation"},
    {"VerifyRodAtWorkstation", "/scene/rod_status"},
    {"PlanPregrasp", "/manipulation/plan_pregrasp"},
    {"ExecutePregrasp", "/manipulation/execute_pregrasp"},
    {"PlanLinearGrasp", "/manipulation/plan_grasp_linear"},
    {"ExecuteLinearGrasp", "/manipulation/execute_grasp_linear"},
    {"AttachRod", "/scene/attach_rod"},
    {"VerifyRodAttached", "/scene/rod_status"},
    {"PlanLinearRetreat", "/manipulation/plan_retreat_linear"},
    {"ExecuteLinearRetreat", "/manipulation/execute_retreat_linear"},
    {"VerifyRodAfterRetreat", "/scene/rod_status"},
  };
  for (const auto & [node_name, service_name] : service_map) {
    factory.registerBuilder<anobot_bt::TriggerServiceNode>(node_name, [node, service_name](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<anobot_bt::TriggerServiceNode>(name, config, node, service_name);
    });
  }
  const auto package_share = ament_index_cpp::get_package_share_directory("anobot_bt");
  const auto tree_path = package_share + "/behavior_trees/mock_rod_pickup.xml";
  RCLCPP_INFO(node->get_logger(), "Loading behavior tree: %s", tree_path.c_str());
  auto tree = factory.createTreeFromFile(tree_path);
  BT::StdCoutLogger logger(tree);
  RCLCPP_WARN(node->get_logger(), "Running mock pickup behavior tree");
  BT::NodeStatus tree_status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && tree_status == BT::NodeStatus::RUNNING) {
    tree_status = tree.tickOnce();
    std::this_thread::sleep_for(20ms);
  }
  if (tree_status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Mock pickup behavior tree succeeded");
  } 
  else {
    RCLCPP_ERROR(node->get_logger(), "Mock pickup behavior tree failed");
  }
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
  return (
    tree_status == BT::NodeStatus::SUCCESS
    ? 0
    : 1
  );
}