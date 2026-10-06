#include "nav2_custom_plugins/recovery_progress_guard.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

namespace nav2_custom_plugins
{

RecoveryProgressGuard::RecoveryProgressGuard(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  getInput("distance", distance_);
  getInput("timeout", timeout_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

bool RecoveryProgressGuard::getRobotPose(double & x, double & y)
{
  try {
    auto transform = tf_->lookupTransform(
      global_frame_,
      robot_base_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "Failed to get transform from %s to %s: %s", 
      robot_base_frame_.c_str(), global_frame_.c_str(), ex.what());
    return false;
  }
}

BT::NodeStatus RecoveryProgressGuard::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    initialized_ = false;
  }

  if (!initialized_) {
    if (!getRobotPose(start_x_, start_y_)) {
      return BT::NodeStatus::FAILURE;
    }
    start_time_ = node_->now();
    initialized_ = true;
    setStatus(BT::NodeStatus::RUNNING);
  }

  double current_x, current_y;
  if (!getRobotPose(current_x, current_y)) {
    haltChild();
    return BT::NodeStatus::FAILURE;
  }

  double dx = current_x - start_x_;
  double dy = current_y - start_y_;
  double dist = std::hypot(dx, dy);

  if (dist >= distance_) {
    RCLCPP_INFO(node_->get_logger(), "RecoveryProgressGuard: Reached distance %.3f >= %.3f, SUCCESS", dist, distance_);
    haltChild();
    return BT::NodeStatus::SUCCESS;
  }

  double elapsed_time = (node_->now() - start_time_).seconds();
  if (elapsed_time > timeout_) {
    RCLCPP_WARN(node_->get_logger(), "RecoveryProgressGuard: Timeout after %.2f s, distance %.3f < %.3f, FAILURE", elapsed_time, dist, distance_);
    haltChild();
    return BT::NodeStatus::FAILURE;
  }

  auto child_status = child_node_->executeTick();

  if (child_status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node_->get_logger(), "RecoveryProgressGuard: Child SUCCESS before distance reached, SUCCESS");
    return BT::NodeStatus::SUCCESS;
  } else if (child_status == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(node_->get_logger(), "RecoveryProgressGuard: Child FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void RecoveryProgressGuard::halt()
{
  initialized_ = false;
  BT::DecoratorNode::halt();
}

}  // namespace nav2_custom_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_plugins::RecoveryProgressGuard>("RecoveryProgressGuard");
}
