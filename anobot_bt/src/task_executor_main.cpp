#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "anobot_bt/task_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<
    anobot_bt::TaskExecutorNode>();

  rclcpp::executors::
    MultiThreadedExecutor executor;

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}