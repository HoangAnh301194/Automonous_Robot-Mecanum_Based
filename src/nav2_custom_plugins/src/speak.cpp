#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/string.hpp"

namespace nav2_custom_plugins
{

class Speak : public BT::SyncActionNode
{
public:
  Speak(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node) {
      throw std::runtime_error("Failed to get 'node' from blackboard in Speak");
    }

    publisher_ = node->create_publisher<std_msgs::msg::String>(
      "/navigation/speech_request", 10);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message", "Message to speak")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string message;

    if (!getInput("message", message)) {
      throw BT::RuntimeError("missing required input [message]: ", name());
    }

    std_msgs::msg::String msg;
    msg.data = message;
    publisher_->publish(msg);

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}  // namespace nav2_custom_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_plugins::Speak>("Speak");
}
