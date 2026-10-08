#ifndef ANOBOT_UI__ROS_BRIDGE_HPP_
#define ANOBOT_UI__ROS_BRIDGE_HPP_

#include <map>
#include <memory>
#include <string>
#include <QObject>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/exceptions.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "anobot_interfaces/action/execute_task.hpp"
#include "anobot_interfaces/msg/bt_node_status.hpp"
#include "anobot_interfaces/msg/bt_state.hpp"

namespace anobot_ui{

class RosBridge : public QObject{
  Q_OBJECT

public:
  using ExecuteTask = anobot_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ClientGoalHandle<ExecuteTask>;
  explicit RosBridge(const rclcpp::Node::SharedPtr & node, QObject * parent = nullptr);
  ~RosBridge() override = default;
  void startTask(const QString & task_name);
  void cancelTask();
  void callOperation(const QString & operation_key);
  void setExecutionAllowed(bool allowed);
  void requestExecutionAllowed();
  void lockExecutionOnShutdown();

Q_SIGNALS:
  void taskStateUpdated(
    const QString & task_name,
    const QString & overall_state,
    const QString & active_node,
    const QString & message
  );
  void btNodeStatusUpdated(
    const QString & node_name,
    const QString & node_type,
    const QString & status
  );
  void logMessage(const QString & message);
  void operationFinished(
    const QString & operation_key,
    bool success,
    const QString & message
  );
  void executionAllowedUpdated(
    bool allowed,
    bool success
  );
  void taskActiveChanged(bool active);
  void rodStatusUpdated(const QString & status, bool success);

private:
  using Trigger = std_srvs::srv::Trigger;
  void createTriggerClients();
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteTask>::SharedPtr action_client_;
  GoalHandleExecuteTask::SharedPtr current_goal_handle_;
  bool task_active_{false};
  bool cancel_in_progress_{false};
  std::map<std::string, std::string> service_names_;
  std::map<std::string, rclcpp::Client<Trigger>::SharedPtr> service_clients_;
  std::shared_ptr<rclcpp::AsyncParametersClient> planner_parameter_client_;
  rclcpp::Subscription<anobot_interfaces::msg::BtNodeStatus>::SharedPtr bt_node_subscription_;
  rclcpp::Subscription<anobot_interfaces::msg::BtState>::SharedPtr bt_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bt_log_subscription_;
};

}  // namespace anobot_ui

#endif  // ANOBOT_UI__ROS_BRIDGE_HPP_