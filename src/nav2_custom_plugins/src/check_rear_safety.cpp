#include <string>
#include <memory>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_custom_plugins
{

enum class SafetyState {
  UNKNOWN,
  SAFE,
  OBSTACLE_TOO_CLOSE,
  SCAN_STALE,
  NO_VALID_SCAN
};

class CheckRearSafety : public BT::ConditionNode
{
public:
  CheckRearSafety(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(xml_tag_name, conf),
    node_(nullptr),
    last_state_(SafetyState::UNKNOWN)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw std::runtime_error("Failed to get 'node' from blackboard in CheckRearSafety");
    }

    std::string topic;
    getInput("topic", topic);

    sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::QoS(1).best_effort(),
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

    SafetyState current_state = SafetyState::UNKNOWN;
    double current_min_found = std::numeric_limits<double>::infinity();
    double age = 0.0;

    if (!current_scan) {
      current_state = SafetyState::NO_VALID_SCAN;
    } else {
      rclcpp::Time now = node_->now();
      rclcpp::Time scan_time(current_scan->header.stamp);
      age = (now - scan_time).seconds();

      if (age > timeout) {
        current_state = SafetyState::SCAN_STALE;
      } else {
        bool has_valid_points = false;
        double robot_width_half = 0.35; // 0.7m total width consideration
        
        for (size_t i = 0; i < current_scan->ranges.size(); ++i) {
          double r = current_scan->ranges[i];
          
          if (std::isnan(r) || r < current_scan->range_min) {
            continue;
          }
          has_valid_points = true;
          
          if (std::isinf(r) || r > current_scan->range_max) {
            continue;
          }
          
          double theta = current_scan->angle_min + i * current_scan->angle_increment;
          double x = r * std::cos(theta);
          double y = r * std::sin(theta);

          if (x > 0.0 && std::abs(y) <= robot_width_half) {
            if (x < current_min_found) {
                current_min_found = x;
            }
          }
        }

        if (!has_valid_points) {
          current_state = SafetyState::NO_VALID_SCAN;
        } else if (current_min_found <= min_distance) {
          current_state = SafetyState::OBSTACLE_TOO_CLOSE;
        } else {
          current_state = SafetyState::SAFE;
        }
      }
    }

    if (current_state != last_state_) {
      std::string state_str;
      switch(current_state) {
        case SafetyState::SAFE: state_str = "SAFE"; break;
        case SafetyState::OBSTACLE_TOO_CLOSE: state_str = "OBSTACLE_TOO_CLOSE"; break;
        case SafetyState::SCAN_STALE: state_str = "SCAN_STALE"; break;
        case SafetyState::NO_VALID_SCAN: state_str = "NO_VALID_SCAN"; break;
        default: state_str = "UNKNOWN"; break;
      }
      
      RCLCPP_INFO(node_->get_logger(), 
        "[CheckRearSafety] State Changed: %s | Age: %.3fs | MinDist: %.2fm | Timeout: %.2fs", 
        state_str.c_str(), age, 
        (std::isinf(current_min_found) ? 99.99 : current_min_found), 
        timeout);
        
      last_state_ = current_state;
    }

    if (current_state == SafetyState::SAFE) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::mutex mutex_;
  SafetyState last_state_;
};

}  // namespace nav2_custom_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_plugins::CheckRearSafety>("CheckRearSafety");
}
