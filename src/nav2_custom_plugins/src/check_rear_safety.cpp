#include <string>
#include <memory>
#include <cmath>
#include <mutex>
#include <thread>
#include <atomic>

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
  NO_VALID_SCAN,
  NO_SCAN
};

class CheckRearSafety : public BT::ConditionNode
{
public:
  CheckRearSafety(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(xml_tag_name, conf),
    node_(nullptr),
    stop_thread_(false),
    last_state_(SafetyState::UNKNOWN),
    has_received_scan_(false),
    cached_finite_count_(0),
    cached_pos_inf_count_(0),
    cached_nan_count_(0),
    cached_min_dist_(std::numeric_limits<double>::infinity())
  {
    // Nav2 Humble provides the `client_node_` (named "_") under the key "node"
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw std::runtime_error("Failed to get 'node' from blackboard in CheckRearSafety");
    }

    std::string topic;
    getInput("topic", topic);

    // Create a mutually exclusive callback group for the subscriber
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group_;

    sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&CheckRearSafety::scanCallback, this, std::placeholders::_1),
      sub_opts);

    // Since `client_node_` is not spun by Nav2's main executor in Humble,
    // and spinning inside `tick()` is banned by requirement, we must spawn
    // a background thread to spin this callback group to receive callbacks.
    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    executor_thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_thread_) {
        executor_.spin_some(std::chrono::milliseconds(10));
      }
    });
  }

  ~CheckRearSafety() override
  {
    stop_thread_ = true;
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
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
    has_received_scan_ = true;
    last_scan_time_ = msg->header.stamp;
    
    int finite_count = 0;
    int pos_inf_count = 0;
    int nan_count = 0;
    double current_min_found = std::numeric_limits<double>::infinity();
    double robot_width_half = 0.35; // 0.7m total width consideration
    
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      
      if (std::isnan(r) || r < msg->range_min) {
        nan_count++;
        continue;
      }
      
      if (std::isinf(r) || r > msg->range_max) {
        pos_inf_count++;
        continue;
      }
      
      finite_count++;
      
      double theta = msg->angle_min + i * msg->angle_increment;
      double x = r * std::cos(theta);
      double y = r * std::sin(theta);

      if (x > 0.0 && std::abs(y) <= robot_width_half) {
        if (x < current_min_found) {
            current_min_found = x;
        }
      }
    }

    cached_finite_count_ = finite_count;
    cached_pos_inf_count_ = pos_inf_count;
    cached_nan_count_ = nan_count;
    cached_min_dist_ = current_min_found;

    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
      "CheckRearSafety scan: ranges=%zu finite=%d pos_inf=%d nan=%d min_dist=%.2f",
      msg->ranges.size(), finite_count, pos_inf_count, nan_count,
      (std::isinf(current_min_found) ? 99.99 : current_min_found));
  }

  BT::NodeStatus tick() override
  {
    double min_distance = 0.40;
    double timeout = 0.50;
    
    getInput("min_distance", min_distance);
    getInput("timeout", timeout);

    bool has_scan = false;
    rclcpp::Time scan_time;
    int finite_count = 0;
    int pos_inf_count = 0;
    int nan_count = 0;
    double current_min_found = std::numeric_limits<double>::infinity();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      has_scan = has_received_scan_;
      if (has_scan) {
        scan_time = last_scan_time_;
        finite_count = cached_finite_count_;
        pos_inf_count = cached_pos_inf_count_;
        nan_count = cached_nan_count_;
        current_min_found = cached_min_dist_;
      }
    }

    SafetyState current_state = SafetyState::UNKNOWN;
    double age = 0.0;

    if (!has_scan) {
      current_state = SafetyState::NO_SCAN;
    } else {
      rclcpp::Time now = node_->now();
      age = (now - scan_time).seconds();

      if (age > timeout) {
        current_state = SafetyState::SCAN_STALE;
      } else {
        if (finite_count == 0 && pos_inf_count == 0) {
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
        case SafetyState::NO_SCAN: state_str = "NO_SCAN"; break;
        default: state_str = "UNKNOWN"; break;
      }
      
      RCLCPP_INFO(node_->get_logger(), 
        "CheckRearSafety: %s | Age: %.3fs | MinDist: %.2fm | Timeout: %.2fs | finite: %d | pos_inf: %d | nan: %d", 
        state_str.c_str(), age, 
        (std::isinf(current_min_found) ? 99.99 : current_min_found), 
        timeout, finite_count, pos_inf_count, nan_count);
        
      last_state_ = current_state;
    }

    if (current_state == SafetyState::SAFE) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  std::atomic<bool> stop_thread_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::mutex mutex_;
  SafetyState last_state_;
  
  bool has_received_scan_;
  rclcpp::Time last_scan_time_;
  int cached_finite_count_;
  int cached_pos_inf_count_;
  int cached_nan_count_;
  double cached_min_dist_;
};

}  // namespace nav2_custom_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_plugins::CheckRearSafety>("CheckRearSafety");
}
