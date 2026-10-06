#ifndef ANOBOT_BT__TRIGGER_SERVICE_NODE_HPP_
#define ANOBOT_BT__TRIGGER_SERVICE_NODE_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace anobot_bt {

class TriggerServiceNode : public BT::StatefulActionNode {

public:
  TriggerServiceNode(const std::string & name, const BT::NodeConfig & config, const rclcpp::Node::SharedPtr & ros_node, const std::string & service_name)
  : BT::StatefulActionNode(name, config), ros_node_(ros_node), service_name_(service_name) {
    client_ = ros_node_->create_client<std_srvs::srv::Trigger>(service_name_);
  }

  static BT::PortsList providedPorts() {
    return {
      BT::OutputPort<std::string>("message")
    };
  }

  BT::NodeStatus onStart() override {
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Service unavailable: %s", service_name_.c_str());
      setOutput("message", "Service unavailable: " + service_name_);
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ros_node_->get_logger(), "BT step started: %s", name().c_str());
    request_ = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_and_request_id = client_->async_send_request(request_);
    future_ = future_and_request_id.future.share();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    if (!future_.valid()) {
      setOutput("message", "Invalid service future");
      return BT::NodeStatus::FAILURE;
    }
    const auto status = future_.wait_for(std::chrono::milliseconds(0));
    if (status != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    const auto response = future_.get();
    if (!response) {
      setOutput("message", "Service returned no response");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("message", response->message);
    if (!response->success) {
      RCLCPP_ERROR(ros_node_->get_logger(), "BT step failed: %s: %s", name().c_str(), response->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(ros_node_->get_logger(), "BT step completed: %s: %s", name().c_str(), response->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {
    RCLCPP_WARN(ros_node_->get_logger(), "BT step halted: %s", name().c_str());
  }

private:
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::shared_ptr<std_srvs::srv::Trigger::Request> request_;
  std::string service_name_;

};

}  // namespace anobot_bt

#endif  // ANOBOT_BT__TRIGGER_SERVICE_NODE_HPP_