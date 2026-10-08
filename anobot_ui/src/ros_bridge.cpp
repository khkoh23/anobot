#include "anobot_ui/ros_bridge.hpp"
#include <action_msgs/srv/cancel_goal.hpp>
#include <chrono>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace anobot_ui{

RosBridge::RosBridge(const rclcpp::Node::SharedPtr & node, QObject * parent): QObject(parent), node_(node){
  action_client_ = rclcpp_action::create_client<ExecuteTask>(node_, "/execute_task");
  createTriggerClients();
  controller_list_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
  moveit_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
  planner_parameter_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/pregrasp_planner_api");
  bt_node_subscription_ = node_->create_subscription<anobot_interfaces::msg::BtNodeStatus>("/bt_node_status", 10, [this](const anobot_interfaces::msg::BtNodeStatus::SharedPtr message){
    Q_EMIT btNodeStatusUpdated(QString::fromStdString(message->node_name), QString::fromStdString(message->node_type), QString::fromStdString(message->status));
  });
  bt_state_subscription_ = node_->create_subscription<anobot_interfaces::msg::BtState>("/bt_state", 10, [this](const anobot_interfaces::msg::BtState::SharedPtr message){
    Q_EMIT taskStateUpdated(QString::fromStdString(message->task_name), QString::fromStdString(message->overall_state), QString::fromStdString(message->active_node), QString::fromStdString(message->message));
  });
  bt_log_subscription_ = node_->create_subscription<std_msgs::msg::String>("/bt_log", 10, [this](const std_msgs::msg::String::SharedPtr message){
    Q_EMIT logMessage(QString::fromStdString(message->data));
  });
}

void RosBridge::createTriggerClients(){
  service_names_ = {
    {"apply_workstation_scene", "/scene/apply_workstation_scene"},
    {"reset_rod", "/scene/reset_rod_on_workstation"},
    {"rod_status", "/scene/rod_status"},
    {"plan_pregrasp", "/manipulation/plan_pregrasp"},
    {"execute_pregrasp", "/manipulation/execute_pregrasp"},
    {"plan_grasp", "/manipulation/plan_grasp_linear"},
    {"execute_grasp", "/manipulation/execute_grasp_linear"},
    {"attach_rod", "/scene/attach_rod" },
    {"plan_retreat", "/manipulation/plan_retreat_linear"},
    {"execute_retreat", "/manipulation/execute_retreat_linear"},
    {"detach_rod", "/scene/detach_rod"}
  };
  for (const auto & item : service_names_) {
    service_clients_[item.first] = node_->create_client<Trigger>(item.second);
  }
}

void RosBridge::startTask(const QString & task_name){
  if (!action_client_->wait_for_action_server(2s)){
    Q_EMIT logMessage("ExecuteTask action server unavailable");
    return;
  }
  ExecuteTask::Goal goal;
  goal.task_name = task_name.toStdString();
  current_goal_handle_.reset();
  task_active_ = false;
  cancel_in_progress_ = false;
  auto options = rclcpp_action::Client<ExecuteTask>::SendGoalOptions();
  options.goal_response_callback = [this](GoalHandleExecuteTask::SharedPtr handle){
    current_goal_handle_ = handle;
    if (handle != nullptr) {
      task_active_ = true;
      cancel_in_progress_ = false;
      Q_EMIT taskActiveChanged(true);
      Q_EMIT logMessage("Task goal accepted");
    } 
    else {
      task_active_ = false;
      cancel_in_progress_ = false;
      Q_EMIT taskActiveChanged(false);
      Q_EMIT logMessage("Task goal rejected");
    }
  };
  options.feedback_callback = [this](GoalHandleExecuteTask::SharedPtr handle, 
  const std::shared_ptr<const ExecuteTask::Feedback> feedback){
    (void)handle;
    Q_EMIT taskStateUpdated("", QString::fromStdString(feedback->current_state), QString::fromStdString(feedback->active_node), QString::fromStdString(feedback->message));
  };
  options.result_callback = [this](const GoalHandleExecuteTask::WrappedResult & result){
    QString message;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        message = "Task finished: SUCCEEDED";
        break;
      case rclcpp_action::ResultCode::ABORTED:
        message = "Task finished: ABORTED";
        break;
      case rclcpp_action::ResultCode::CANCELED:
        message = "Task finished: CANCELED";
        break;
      default:
        message = "Task finished: UNKNOWN";
        break;
    }
    if (result.result != nullptr){
      message += ": ";
      message += QString::fromStdString(result.result->message);
    }
    Q_EMIT logMessage(message);
    task_active_ = false;
    cancel_in_progress_ = false;
    Q_EMIT taskActiveChanged(false);
    current_goal_handle_.reset();
  };
  action_client_->async_send_goal(goal, options);
  Q_EMIT logMessage("Sending task: " + task_name);
}

void RosBridge::cancelTask() {
  if (current_goal_handle_ == nullptr || !task_active_){
    Q_EMIT logMessage("No active task to cancel");
    return;
  }
  if (cancel_in_progress_) {
    Q_EMIT logMessage("Cancellation already requested");
    return;
  }
  cancel_in_progress_ = true;
  action_client_->async_cancel_goal(current_goal_handle_, [this](rclcpp_action::Client<ExecuteTask>::CancelResponse::SharedPtr response){
    if (response == nullptr) {
      Q_EMIT logMessage("No cancellation response received");
      cancel_in_progress_ = false;
      return;
    }
    if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE){
      Q_EMIT logMessage("Cancellation request processed");
    } 
    else {
      Q_EMIT logMessage(QString("Cancellation returned code: %1").arg(response->return_code));
      cancel_in_progress_ = false;
    }
  });
  Q_EMIT logMessage("Task cancellation requested");
}

void RosBridge::callOperation(const QString & operation_key){
  const std::string key = operation_key.toStdString();
  const auto client_iterator = service_clients_.find(key);
  const auto name_iterator = service_names_.find(key);
  if (client_iterator == service_clients_.end() || name_iterator == service_names_.end()){
    Q_EMIT operationFinished(operation_key, false, "Unknown operation");
    return;
  }
  const auto client = client_iterator->second;
  if (!client->service_is_ready()) {
    Q_EMIT operationFinished(operation_key, false, QString::fromStdString("Service unavailable: " + name_iterator->second));
    return;
  }
  auto request = std::make_shared<Trigger::Request>();
  client->async_send_request(request, [this, operation_key](rclcpp::Client<Trigger>::SharedFuture future){
    try {
      const auto response = future.get();
      Q_EMIT operationFinished(operation_key, response->success, QString::fromStdString(response->message));
      if (operation_key == "rod_status") {
        Q_EMIT rodStatusUpdated(QString::fromStdString(response->message), response->success);
      }
      if (response->success && (operation_key == "reset_rod" || operation_key == "attach_rod" || operation_key == "detach_rod")){
        callOperation("rod_status");
      }
    }
    catch (const std::exception & error){
      Q_EMIT operationFinished(operation_key, false, QString::fromStdString(error.what()));
    }
  });
  Q_EMIT logMessage("Calling operation: " + operation_key);
}

void RosBridge::setExecutionAllowed(bool allowed){
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("allow_execution", allowed));
  planner_parameter_client_->set_parameters(parameters, [this, allowed](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> completed_future){
    bool success = true;
    try {
      const auto results = completed_future.get();
      for (const auto & result : results) {
        if (!result.successful) {
          success = false;
          break;
        }
      }
    }
    catch (const std::exception &) {
      success = false;
    }
    Q_EMIT executionAllowedUpdated(allowed, success);
    Q_EMIT logMessage(success 
      ? QString("Execution permission set to %1").arg(allowed ? "TRUE" : "FALSE") 
      : "Failed to set execution permission");
  });
}

void RosBridge::requestExecutionAllowed() {
  if (!planner_parameter_client_->service_is_ready()){
    Q_EMIT executionAllowedUpdated(false, false);
    return;
  }
  planner_parameter_client_->get_parameters({"allow_execution"}, [this](auto completed_future){
    try {
      const auto parameters = completed_future.get();
      if (parameters.empty()) {
        Q_EMIT executionAllowedUpdated(false, false);
        return;
      }
      Q_EMIT executionAllowedUpdated(parameters.front().as_bool(), true);
    }
    catch (const std::exception &) {
      Q_EMIT executionAllowedUpdated(false, false);
    }
  });
}

void RosBridge::lockExecutionOnShutdown(){
  if (planner_parameter_client_ == nullptr || !planner_parameter_client_->service_is_ready()){
    return;
  }
  std::vector<rclcpp::Parameter> parameters = {rclcpp::Parameter("allow_execution", false)};
  planner_parameter_client_->set_parameters(parameters);
}

void RosBridge::pollSystemHealth() {
  const bool moveit_ready = moveit_scene_client_ != nullptr && moveit_scene_client_->service_is_ready();
  Q_EMIT componentHealthUpdated("moveit", moveit_ready, moveit_ready ? "Planning-scene service available" : "/get_planning_scene unavailable");
  const auto scene_iterator = service_clients_.find("apply_workstation_scene");
  const bool scene_ready = scene_iterator != service_clients_.end() && scene_iterator->second->service_is_ready();
  Q_EMIT componentHealthUpdated("workstation_scene", scene_ready, scene_ready ? "Scene refresh service available" : "/scene/apply_workstation_scene unavailable");
  const auto manipulation_iterator = service_clients_.find("plan_pregrasp");
  const bool manipulation_ready = manipulation_iterator != service_clients_.end() && manipulation_iterator->second->service_is_ready();
  Q_EMIT componentHealthUpdated("manipulation", manipulation_ready, manipulation_ready ? "Manipulation planner available" : "/manipulation/plan_pregrasp unavailable");
  const auto rod_iterator = service_clients_.find("rod_status");
  const bool rod_ready = rod_iterator != service_clients_.end() && rod_iterator->second->service_is_ready();
  Q_EMIT componentHealthUpdated("rod_lifecycle", rod_ready, rod_ready ? "Rod lifecycle service available" : "/scene/rod_status unavailable");
  const bool bt_ready = action_client_ != nullptr && action_client_->action_server_is_ready();
  Q_EMIT componentHealthUpdated("bt_executor", bt_ready, bt_ready ? "ExecuteTask action server available" : "/execute_task unavailable");
  bool marker_ready = false;
  QString marker_detail = "workstation_marker_26_calibrated missing";
  if (tf_buffer_ != nullptr) {
    try {
      marker_ready = tf_buffer_->canTransform("ur_base", "workstation_marker_26_calibrated", tf2::TimePointZero, tf2::durationFromSec(0.0));
      if (marker_ready) {
        marker_detail = "Calibrated workstation transform available";
      }
    }
    catch (const std::exception & error) {
      marker_ready = false;
      marker_detail = QString::fromStdString(error.what());
    }
  }
  bool mock_marker_node_found = false;
  const auto node_names = node_->get_node_names();
  for (const auto & node_name : node_names) {
    if (node_name == "mock_workstation_marker" || node_name == "/mock_workstation_marker"){
      mock_marker_node_found = true;
      break;
    }
  }
  marker_ready = mock_marker_node_found && tf_buffer_->canTransform("ur_base", "workstation_marker_26_calibrated", tf2::TimePointZero, tf2::durationFromSec(0.0));
  if (!mock_marker_node_found) {
    marker_detail = "Mock marker publisher is offline";
  } 
  else if (!marker_ready) {
    marker_detail = "Mock marker node exists, but transform is unavailable";
  } 
  else {
    marker_detail = "Mock calibrated workstation transform available";
  }
  Q_EMIT componentHealthUpdated("workstation_marker", marker_ready, marker_detail);
  requestControllerStatus();
}

void RosBridge::requestControllerStatus(){
  if (controller_list_client_ == nullptr || !controller_list_client_->service_is_ready()){
    Q_EMIT componentHealthUpdated("controller", false, "/controller_manager/list_controllers unavailable");
    return;
  }
  bool expected = false;
  if (!controller_request_active_.compare_exchange_strong(expected, true)){
    return;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  controller_list_client_->async_send_request(request, [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future){
    bool ready = false;
    QString detail = "scaled_joint_trajectory_controller " "not active";
    try {
      const auto response = future.get();
      for (const auto & controller : response->controller){
        if (controller.name == "scaled_joint_trajectory_controller"){
          ready = controller.state == "active";
          detail = QString("scaled_joint_trajectory_controller: %1").arg(QString::fromStdString(controller.state));
          break;
        }
      }
    }
    catch (const std::exception & error) {
      ready = false;
      detail = QString::fromStdString(error.what());
    }
    controller_request_active_ = false;
    Q_EMIT componentHealthUpdated("controller", ready, detail);
  });
}

}  // namespace anobot_ui