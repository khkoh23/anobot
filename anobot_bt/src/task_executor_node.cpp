#include "anobot_bt/task_executor_node.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "anobot_bt/trigger_service_node.hpp"

using namespace std::chrono_literals;

namespace anobot_bt
{

namespace
{

class TaskRunningGuard
{
public:
  explicit TaskRunningGuard(
    std::atomic<bool> & flag)
  : flag_(flag)
  {
  }

  ~TaskRunningGuard()
  {
    flag_ = false;
  }

private:
  std::atomic<bool> & flag_;
};

struct ServiceNodeDefinition
{
  std::string service_name;
  std::string expected_substring;
};

}  // namespace

TaskExecutorNode::TaskExecutorNode()
: Node("anobot_task_executor")
{
  registerBtNodes();

  bt_node_pub_ = create_publisher<
    anobot_interfaces::msg::BtNodeStatus>(
    "bt_node_status",
    10);

  bt_state_pub_ = create_publisher<
    anobot_interfaces::msg::BtState>(
    "bt_state",
    10);

  bt_log_pub_ = create_publisher<
    std_msgs::msg::String>(
    "bt_log",
    10);

  action_server_ =
    rclcpp_action::create_server<ExecuteTask>(
    this,
    "execute_task",
    std::bind(
      &TaskExecutorNode::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &TaskExecutorNode::handleCancel,
      this,
      std::placeholders::_1),
    std::bind(
      &TaskExecutorNode::handleAccepted,
      this,
      std::placeholders::_1));

  publishTaskState(
    "",
    "IDLE",
    "",
    "Task executor ready");

  publishLog(
    "Anobot task executor ready");

  RCLCPP_INFO(
    get_logger(),
    "Persistent ExecuteTask action server ready");
}

void TaskExecutorNode::registerBtNodes()
{
  const std::unordered_map<
    std::string,
    ServiceNodeDefinition> service_map = {
    {
      "ResetRod",
      {
        "/scene/reset_rod_on_workstation",
        ""
      }
    },
    {
      "VerifyRodAtWorkstation",
      {
        "/scene/rod_status",
        "WORLD_OBJECT"
      }
    },
    {
      "PlanPregrasp",
      {
        "/manipulation/plan_pregrasp",
        ""
      }
    },
    {
      "ExecutePregrasp",
      {
        "/manipulation/execute_pregrasp",
        ""
      }
    },
    {
      "PlanLinearGrasp",
      {
        "/manipulation/plan_grasp_linear",
        ""
      }
    },
    {
      "ExecuteLinearGrasp",
      {
        "/manipulation/execute_grasp_linear",
        ""
      }
    },
    {
      "AttachRod",
      {
        "/scene/attach_rod",
        ""
      }
    },
    {
      "VerifyRodAttached",
      {
        "/scene/rod_status",
        "ATTACHED_TO_GRIPPER"
      }
    },
    {
      "PlanLinearRetreat",
      {
        "/manipulation/plan_retreat_linear",
        ""
      }
    },
    {
      "ExecuteLinearRetreat",
      {
        "/manipulation/execute_retreat_linear",
        ""
      }
    },
    {
      "VerifyRodAfterRetreat",
      {
        "/scene/rod_status",
        "ATTACHED_TO_GRIPPER"
      }
    }
  };

  for (const auto & item : service_map) {
    const std::string node_name =
      item.first;

    const ServiceNodeDefinition definition =
      item.second;

    const BT::NodeBuilder builder = [this, definition](
      const std::string & name,
        const BT::NodeConfig & config)
      {
        return std::make_unique<
          TriggerServiceNode>(
          name,
          config,
          this->shared_from_this(),
          definition.service_name,
          definition.expected_substring);
      };

    factory_.registerBuilder<
      TriggerServiceNode>(
      node_name,
      builder);
  }
}

rclcpp_action::GoalResponse
TaskExecutorNode::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<
    const ExecuteTask::Goal> goal)
{
  if (
    goal->task_name != "mock_rod_pickup")
  {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting unsupported task: %s",
      goal->task_name.c_str());

    return rclcpp_action::GoalResponse::REJECT;
  }

  bool expected = false;

  if (!task_running_.compare_exchange_strong(
      expected,
      true))
  {
    RCLCPP_WARN(
      get_logger(),
      "Rejecting task '%s': another task "
      "is already running",
      goal->task_name.c_str());

    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted task goal: %s",
    goal->task_name.c_str());

  return
    rclcpp_action::GoalResponse::
    ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
TaskExecutorNode::handleCancel(
  const std::shared_ptr<
    GoalHandleExecuteTask>)
{
  cancel_requested_ = true;

  publishLog(
    "Task cancellation requested");

  RCLCPP_WARN(
    get_logger(),
    "Task cancellation requested");

  return
    rclcpp_action::CancelResponse::ACCEPT;
}

void TaskExecutorNode::handleAccepted(
  const std::shared_ptr<
    GoalHandleExecuteTask> goal_handle)
{
  std::thread(
    &TaskExecutorNode::executeGoal,
    this,
    goal_handle).detach();
}

void TaskExecutorNode::executeGoal(
  const std::shared_ptr<
    GoalHandleExecuteTask> goal_handle)
{
  TaskRunningGuard running_guard(
    task_running_);

  const auto goal =
    goal_handle->get_goal();

  const std::string task_name =
    goal->task_name;

  cancel_requested_ = false;

  auto feedback =
    std::make_shared<
    ExecuteTask::Feedback>();

  auto result =
    std::make_shared<
    ExecuteTask::Result>();

  setLifecycleState(
    TaskLifecycleState::PRECHECK,
    task_name,
    "",
    "Loading behavior tree");

  if (!loadTreeForTask(task_name)) {
    setLifecycleState(
      TaskLifecycleState::FAILURE,
      task_name,
      "",
      "Failed to load behavior tree");

    result->success = false;
    result->message =
      "Failed to load behavior tree";

    goal_handle->abort(result);
    return;
  }

  setLifecycleState(
    TaskLifecycleState::RUNNING,
    task_name,
    "",
    "Task started");

  publishLog(
    "Task started: " + task_name);

  while (rclcpp::ok()) {
    if (
      cancel_requested_ ||
      goal_handle->is_canceling())
    {
      setLifecycleState(
        TaskLifecycleState::CANCELING,
        task_name,
        findActiveNode(),
        "Halting behavior tree");

      {
        std::lock_guard<std::mutex> lock(
          tree_mutex_);

        tree_.haltTree();
      }

      publishTreeStatus();

      setLifecycleState(
        TaskLifecycleState::CANCELED,
        task_name,
        "",
        "Task canceled");

      publishLog(
        "Task canceled: " + task_name);

      result->success = false;
      result->message = "Task canceled";

      goal_handle->canceled(result);
      return;
    }

    BT::NodeStatus status;

    {
      std::lock_guard<std::mutex> lock(
        tree_mutex_);

      status = tree_.tickOnce();
    }

    publishTreeStatus();

    const std::string active_node =
      findActiveNode();

    std::string message =
      currentLeafMessage();

    if (message.empty()) {
      message = "Executing";
    }

    feedback->current_state =
      statusToString(status);

    feedback->active_node =
      active_node;

    feedback->message =
      message;

    goal_handle->publish_feedback(
      feedback);

    publishTaskState(
      task_name,
      "RUNNING",
      active_node,
      message);

    if (status == BT::NodeStatus::SUCCESS) {
      setLifecycleState(
        TaskLifecycleState::SUCCESS,
        task_name,
        "",
        "Task succeeded");

      publishLog(
        "Task succeeded: " + task_name);

      result->success = true;
      result->message =
        "Mock rod pickup succeeded";

      goal_handle->succeed(result);
      return;
    }

    if (status == BT::NodeStatus::FAILURE) {
      setLifecycleState(
        TaskLifecycleState::FAILURE,
        task_name,
        active_node,
        "Task failed");

      publishLog(
        "Task failed: " + task_name);

      result->success = false;
      result->message =
        "Behavior tree returned FAILURE";

      goal_handle->abort(result);
      return;
    }

    std::this_thread::sleep_for(50ms);
  }

  setLifecycleState(
    TaskLifecycleState::FAULT,
    task_name,
    "",
    "ROS shutdown during task execution");

  result->success = false;
  result->message =
    "ROS shutdown during task execution";

  goal_handle->abort(result);
}

bool TaskExecutorNode::loadTreeForTask(
  const std::string & task_name)
{
  const std::string xml_path =
    taskXmlPath(task_name);

  if (xml_path.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Unknown task name: %s",
      task_name.c_str());

    return false;
  }

  std::ifstream input(xml_path);

  if (!input.is_open()) {
    RCLCPP_ERROR(
      get_logger(),
      "Could not open tree file: %s",
      xml_path.c_str());

    return false;
  }

  std::stringstream buffer;
  buffer << input.rdbuf();

  try {
    auto blackboard =
      BT::Blackboard::create();

    blackboard->set<
      std::atomic<bool> *>(
      "cancel_requested",
      &cancel_requested_);

    std::lock_guard<std::mutex> lock(
      tree_mutex_);

    tree_ = factory_.createTreeFromText(
      buffer.str(),
      blackboard);
  }
  catch (const std::exception & error) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to create behavior tree: %s",
      error.what());

    return false;
  }

  last_status_.clear();
  publishTreeStatus();

  RCLCPP_INFO(
    get_logger(),
    "Loaded behavior tree: %s",
    xml_path.c_str());

  return true;
}

std::string TaskExecutorNode::taskXmlPath(
  const std::string & task_name) const
{
  const auto share_directory =
    ament_index_cpp::
    get_package_share_directory(
    "anobot_bt");

  if (task_name == "mock_rod_pickup") {
    return share_directory +
      "/behavior_trees/"
      "mock_rod_pickup.xml";
  }

  return "";
}

void TaskExecutorNode::publishTreeStatus()
{
  std::lock_guard<std::mutex> lock(
    tree_mutex_);

  if (!tree_.rootNode()) {
    return;
  }

  BT::applyRecursiveVisitor(
    tree_.rootNode(),
    [&](BT::TreeNode * node)
    {
      const std::string name =
        node->name();

      const std::string status =
        statusToString(node->status());

      const auto iterator =
        last_status_.find(name);

      if (
        iterator != last_status_.end() &&
        iterator->second == status)
      {
        return;
      }

      last_status_[name] = status;

      anobot_interfaces::msg::BtNodeStatus
        message;

      message.node_name = name;

      message.node_type =
        nodeTypeToString(node->type());

      message.status = status;

      bt_node_pub_->publish(message);
    });
}

std::string TaskExecutorNode::findActiveNode() const
{
  std::string active_node;

  std::lock_guard<std::mutex> lock(
    tree_mutex_);

  if (!tree_.rootNode()) {
    return active_node;
  }

  BT::applyRecursiveVisitor(
    tree_.rootNode(),
    [&active_node](BT::TreeNode * node)
    {
      const bool leaf =
        node->type() == BT::NodeType::ACTION ||
        node->type() == BT::NodeType::CONDITION;

      if (
        active_node.empty() &&
        leaf &&
        node->status() ==
        BT::NodeStatus::RUNNING)
      {
        active_node = node->name();
      }
    });

  return active_node;
}

std::string
TaskExecutorNode::currentLeafMessage() const
{
  std::string message;

  std::lock_guard<std::mutex> lock(
    tree_mutex_);

  if (!tree_.rootNode()) {
    return message;
  }

  BT::applyRecursiveVisitor(
    tree_.rootNode(),
    [&message](BT::TreeNode * node)
    {
      if (!message.empty()) {
        return;
      }

      if (
        node->status() !=
        BT::NodeStatus::RUNNING)
      {
        return;
      }

      auto * trigger_node =
        dynamic_cast<TriggerServiceNode *>(
        node);

      if (trigger_node != nullptr) {
        message =
          trigger_node->lastMessage();
      }
    });

  return message;
}

void TaskExecutorNode::publishTaskState(
  const std::string & task_name,
  const std::string & overall_state,
  const std::string & active_node,
  const std::string & message)
{
  anobot_interfaces::msg::BtState
    state_message;

  state_message.task_name =
    task_name;

  state_message.overall_state =
    overall_state;

  state_message.active_node =
    active_node;

  state_message.message =
    message;

  bt_state_pub_->publish(
    state_message);
}

void TaskExecutorNode::publishLog(
  const std::string & message)
{
  std_msgs::msg::String log_message;
  log_message.data = message;

  bt_log_pub_->publish(log_message);

  RCLCPP_INFO(
    get_logger(),
    "%s",
    message.c_str());
}

void TaskExecutorNode::setLifecycleState(
  TaskLifecycleState state,
  const std::string & task_name,
  const std::string & active_node,
  const std::string & message)
{
  lifecycle_state_ = state;

  publishTaskState(
    task_name,
    lifecycleStateToString(state),
    active_node,
    message);
}

std::string
TaskExecutorNode::lifecycleStateToString(
  TaskLifecycleState state) const
{
  switch (state) {
    case TaskLifecycleState::IDLE:
      return "IDLE";

    case TaskLifecycleState::PRECHECK:
      return "PRECHECK";

    case TaskLifecycleState::RUNNING:
      return "RUNNING";

    case TaskLifecycleState::CANCELING:
      return "CANCELING";

    case TaskLifecycleState::CANCELED:
      return "CANCELED";

    case TaskLifecycleState::SUCCESS:
      return "SUCCESS";

    case TaskLifecycleState::FAILURE:
      return "FAILURE";

    case TaskLifecycleState::FAULT:
      return "FAULT";

    default:
      return "UNKNOWN";
  }
}

std::string TaskExecutorNode::statusToString(
  BT::NodeStatus status) const
{
  switch (status) {
    case BT::NodeStatus::IDLE:
      return "IDLE";

    case BT::NodeStatus::RUNNING:
      return "RUNNING";

    case BT::NodeStatus::SUCCESS:
      return "SUCCESS";

    case BT::NodeStatus::FAILURE:
      return "FAILURE";

    default:
      return "UNKNOWN";
  }
}

std::string TaskExecutorNode::nodeTypeToString(
  BT::NodeType type) const
{
  switch (type) {
    case BT::NodeType::ACTION:
      return "ACTION";

    case BT::NodeType::CONDITION:
      return "CONDITION";

    case BT::NodeType::CONTROL:
      return "CONTROL";

    case BT::NodeType::DECORATOR:
      return "DECORATOR";

    case BT::NodeType::SUBTREE:
      return "SUBTREE";

    default:
      return "UNKNOWN";
  }
}

}  // namespace anobot_bt