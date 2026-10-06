#ifndef ANOBOT_BT__TRIGGER_SERVICE_NODE_HPP_
#define ANOBOT_BT__TRIGGER_SERVICE_NODE_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace anobot_bt
{

class TriggerServiceNode : public BT::StatefulActionNode
{
public:
  TriggerServiceNode(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & ros_node,
    const std::string & service_name,
    const std::string & expected_substring = "")
  : BT::StatefulActionNode(name, config),
    ros_node_(ros_node),
    service_name_(service_name),
    expected_substring_(expected_substring)
  {
    client_ =
      ros_node_->create_client<std_srvs::srv::Trigger>(
      service_name_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("message")
    };
  }

  const std::string & lastMessage() const
  {
    return last_message_;
  }

  BT::NodeStatus onStart() override
  {
    last_message_.clear();

    if (!client_->wait_for_service(
        std::chrono::seconds(5)))
    {
      last_message_ =
        "Service unavailable: " + service_name_;

      setOutput("message", last_message_);

      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "%s",
        last_message_.c_str());

      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "BT step started: %s",
      name().c_str());

    request_ =
      std::make_shared<
      std_srvs::srv::Trigger::Request>();

    auto future_and_request_id =
      client_->async_send_request(request_);

    future_ =
      future_and_request_id.future.share();

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (!future_.valid()) {
      last_message_ =
        "Invalid service future";

      setOutput("message", last_message_);

      return BT::NodeStatus::FAILURE;
    }

    const auto status =
      future_.wait_for(
      std::chrono::milliseconds(0));

    if (status != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    const auto response = future_.get();

    if (!response) {
      last_message_ =
        "Service returned no response";

      setOutput("message", last_message_);

      return BT::NodeStatus::FAILURE;
    }

    last_message_ = response->message;
    setOutput("message", last_message_);

    if (!response->success) {
      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "BT step failed: %s: %s",
        name().c_str(),
        response->message.c_str());

      return BT::NodeStatus::FAILURE;
    }

    if (
      !expected_substring_.empty() &&
      response->message.find(
        expected_substring_) == std::string::npos)
    {
      last_message_ =
        "Expected response containing '" +
        expected_substring_ +
        "', received: " +
        response->message;

      setOutput("message", last_message_);

      RCLCPP_ERROR(
        ros_node_->get_logger(),
        "BT verification failed: %s",
        last_message_.c_str());

      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "BT step completed: %s: %s",
      name().c_str(),
      response->message.c_str());

    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override
  {
    last_message_ =
      "BT step halted: " + name();

    RCLCPP_WARN(
      ros_node_->get_logger(),
      "%s",
      last_message_.c_str());
  }

private:
  rclcpp::Node::SharedPtr ros_node_;

  std::string service_name_;
  std::string expected_substring_;
  std::string last_message_;

  rclcpp::Client<
    std_srvs::srv::Trigger>::SharedPtr client_;

  std::shared_ptr<
    std_srvs::srv::Trigger::Request> request_;

  rclcpp::Client<
    std_srvs::srv::Trigger>::SharedFuture future_;
};

}  // namespace anobot_bt

#endif  // ANOBOT_BT__TRIGGER_SERVICE_NODE_HPP_