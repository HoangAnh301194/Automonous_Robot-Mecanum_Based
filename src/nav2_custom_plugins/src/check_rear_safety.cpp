#include <string>
#include <memory>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_custom_plugins
{

class CheckRearSafety : public BT::ConditionNode
{
public:
  CheckRearSafety(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(xml_tag_name, conf),
    node_(nullptr)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw std::runtime_error("Failed to get 'node' from blackboard in CheckRearSafety");
    }

    std::string topic;
    getInput("topic", topic);

    sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, 10,
      std::bind(&CheckRearSafety::scanCallback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/rear_scan_obstacle", "Topic to subscribe to"),
      BT::InputPort<double>("min_distance", 0.40, "Minimum safe distance in X"),
      BT::InputPort<double>("timeout", 0.50, "Maximum age of scan data in seconds")
    };
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_scan_ = msg;
  }

  BT::NodeStatus tick() override
  {
    double min_distance = 0.40;
    double timeout = 0.50;
    
    getInput("min_distance", min_distance);
    getInput("timeout", timeout);

    sensor_msgs::msg::LaserScan::SharedPtr current_scan;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_scan = last_scan_;
    }

    if (!current_scan) {
      RCLCPP_WARN(node_->get_logger(), "CheckRearSafety: No scan received yet.");
      return BT::NodeStatus::FAILURE;
    }

    rclcpp::Time now = node_->now();
    rclcpp::Time scan_time(current_scan->header.stamp);
    double age = (now - scan_time).seconds();

    if (age > timeout) {
      RCLCPP_WARN(node_->get_logger(), "CheckRearSafety: Scan data too old (%.2f s).", age);
      return BT::NodeStatus::FAILURE;
    }

    bool has_valid_points = false;
    double robot_width_half = 0.35; // 0.7m total width consideration

    for (size_t i = 0; i < current_scan->ranges.size(); ++i) {
      double r = current_scan->ranges[i];
      
      // If NaN or < min_range, it's invalid
      if (std::isnan(r) || r < current_scan->range_min) {
        continue;
      }
      
      has_valid_points = true;
      
      // If inf or > max_range, it means free space, so it's safe and valid, no collision check needed for this ray
      if (std::isinf(r) || r > current_scan->range_max) {
        continue;
      }
      
      double theta = current_scan->angle_min + i * current_scan->angle_increment;
      double x = r * std::cos(theta);
      double y = r * std::sin(theta);

      // Check if the point is within the bounding box behind the robot
      // Since rear_scan_frame X is backward, x > 0 means behind.
      if (x > 0.0 && x <= min_distance && std::abs(y) <= robot_width_half) {
        RCLCPP_WARN(node_->get_logger(), "CheckRearSafety: Obstacle detected at X: %.2f, Y: %.2f", x, y);
        return BT::NodeStatus::FAILURE;
      }
    }

    if (!has_valid_points) {
      // If ALL points were NaN or < range_min, it's an error state
      RCLCPP_WARN(node_->get_logger(), "CheckRearSafety: No valid points in scan, cannot confirm safety.");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::mutex mutex_;
};

}  // namespace nav2_custom_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_plugins::CheckRearSafety>("CheckRearSafety");
}
