#ifndef ANOBOT_BT__TASK_EXECUTOR_NODE_HPP_
#define ANOBOT_BT__TASK_EXECUTOR_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

#include "anobot_interfaces/action/execute_task.hpp"
#include "anobot_interfaces/msg/bt_node_status.hpp"
#include "anobot_interfaces/msg/bt_state.hpp"

namespace anobot_bt
{

class TaskExecutorNode : public rclcpp::Node
{
public:
  using ExecuteTask =
    anobot_interfaces::action::ExecuteTask;

  using GoalHandleExecuteTask =
    rclcpp_action::ServerGoalHandle<ExecuteTask>;

  TaskExecutorNode();

private:
  enum class TaskLifecycleState
  {
    IDLE,
    PRECHECK,
    RUNNING,
    CANCELING,
    CANCELED,
    SUCCESS,
    FAILURE,
    FAULT
  };

  void registerBtNodes();

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<
      const ExecuteTask::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<
      GoalHandleExecuteTask> goal_handle);

  void handleAccepted(
    const std::shared_ptr<
      GoalHandleExecuteTask> goal_handle);

  void executeGoal(
    const std::shared_ptr<
      GoalHandleExecuteTask> goal_handle);

  bool loadTreeForTask(
    const std::string & task_name);

  std::string taskXmlPath(
    const std::string & task_name) const;

  void publishTreeStatus();

  std::string findActiveNode() const;

  std::string currentLeafMessage() const;

  void publishTaskState(
    const std::string & task_name,
    const std::string & overall_state,
    const std::string & active_node,
    const std::string & message);

  void publishLog(
    const std::string & message);

  void setLifecycleState(
    TaskLifecycleState state,
    const std::string & task_name,
    const std::string & active_node,
    const std::string & message);

  std::string lifecycleStateToString(
    TaskLifecycleState state) const;

  std::string statusToString(
    BT::NodeStatus status) const;

  std::string nodeTypeToString(
    BT::NodeType type) const;

  rclcpp_action::Server<
    ExecuteTask>::SharedPtr action_server_;

  rclcpp::Publisher<
    anobot_interfaces::msg::BtNodeStatus
  >::SharedPtr bt_node_pub_;

  rclcpp::Publisher<
    anobot_interfaces::msg::BtState
  >::SharedPtr bt_state_pub_;

  rclcpp::Publisher<
    std_msgs::msg::String
  >::SharedPtr bt_log_pub_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  std::unordered_map<
    std::string,
    std::string> last_status_;

  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> task_running_{false};

  mutable std::mutex tree_mutex_;

  TaskLifecycleState lifecycle_state_{
    TaskLifecycleState::IDLE};
};

}  // namespace anobot_bt

#endif  // ANOBOT_BT__TASK_EXECUTOR_NODE_HPP_