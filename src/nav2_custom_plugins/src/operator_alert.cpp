#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace nav2_custom_plugins
{

class OperatorAlert : public BT::SyncActionNode
{
public:
  OperatorAlert(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
      throw std::runtime_error("Failed to get 'node' from blackboard in OperatorAlert");
    }

    publisher_ = node->create_publisher<diagnostic_msgs::msg::KeyValue>(
      "/navigation/operator_alert", 10);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("code", "Event code"),
      BT::InputPort<std::string>("message", "Message to display")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string code;
    std::string message;

    if (!getInput("code", code)) {
      throw BT::RuntimeError("missing required input [code]: ", name());
    }
    if (!getInput("message", message)) {
      throw BT::RuntimeError("missing required input [message]: ", name());
    }

    diagnostic_msgs::msg::KeyValue msg;
    msg.key = code;
    msg.value = message;
    publisher_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr publisher_;
};

}  // namespace nav2_custom_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_plugins::OperatorAlert>("OperatorAlert");
}
