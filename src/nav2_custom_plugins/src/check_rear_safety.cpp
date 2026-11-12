#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/condition_node.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav2_custom_plugins/rear_safety_evaluator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_custom_plugins
{

class CheckRearSafety : public BT::ConditionNode
{
public:
  CheckRearSafety(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(xml_tag_name, conf),
    node_(nullptr),
    stop_thread_(false),
    has_last_reason_(false),
    last_reason_(RearSafetyReason::SENSOR_STALE)
  {
    // Nav2 Humble provides `client_node_` under the blackboard key "node".
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw std::runtime_error("Failed to get 'node' from blackboard in CheckRearSafety");
    }

    if (!config().blackboard->get("tf_buffer", tf_) || !tf_) {
      owned_tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*owned_tf_, node_, true);
      tf_ = owned_tf_;
    }

    std::string topic = "/rear_scan_obstacle";
    getInput("topic", topic);

    std::string health_topic;
    getInput("health_topic", health_topic);
    if (health_topic.empty()) {
      health_topic = topic + "/health";
    }

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = callback_group_;

    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS(),
      std::bind(&CheckRearSafety::scanCallback, this, std::placeholders::_1),
      sub_opts);

    health_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      health_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
      std::bind(&CheckRearSafety::healthCallback, this, std::placeholders::_1),
      sub_opts);

    executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    executor_thread_ = std::thread([this]() {
      while (rclcpp::ok() && !stop_thread_) {
        executor_.spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
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
      BT::InputPort<std::string>("topic", "/rear_scan_obstacle", "Rear LaserScan topic"),
      BT::InputPort<std::string>("health_topic", "", "Rear detector health DiagnosticArray topic"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot base frame"),
      BT::InputPort<double>("min_distance", 0.50, "Minimum safe rear distance"),
      BT::InputPort<double>("timeout", 0.50, "Maximum age of scan/health data in seconds"),
      BT::InputPort<bool>("require_health", true, "Require rear observability health verification")
    };
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = msg;
  }

  void healthCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_health_ = msg;
  }

  BT::NodeStatus tick() override
  {
    double min_distance = 0.50;
    double timeout = 0.50;
    std::string robot_base_frame = "base_footprint";
    bool require_health = true;

    getInput("min_distance", min_distance);
    getInput("timeout", timeout);
    getInput("robot_base_frame", robot_base_frame);
    getInput("require_health", require_health);

    RearSafetyConfig cfg;
    cfg.min_distance = min_distance;
    cfg.scan_timeout = timeout;
    cfg.health_timeout = timeout;
    cfg.require_health = require_health;
    RearSafetyEvaluator evaluator(cfg);

    RearSafetyInput input;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      input.scan = latest_scan_;
      input.health = latest_health_;
    }
    input.now = node_->now();
    input.remaining_dist = 0.30;
    input.current_speed = 0.10;

    if (input.scan && !input.scan->header.frame_id.empty()) {
      if (input.scan->header.frame_id == robot_base_frame) {
        input.tf_valid = true;
        input.scan_to_base_tx = 0.0;
        input.scan_to_base_ty = 0.0;
        input.scan_to_base_yaw = 0.0;
      } else {
        try {
          const auto tf_msg = tf_->lookupTransform(
            robot_base_frame, input.scan->header.frame_id, tf2::TimePointZero);
          input.scan_to_base_tx = tf_msg.transform.translation.x;
          input.scan_to_base_ty = tf_msg.transform.translation.y;
          tf2::Quaternion q(
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w);
          double roll = 0.0;
          double pitch = 0.0;
          double yaw = 0.0;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          input.scan_to_base_yaw = yaw;
          input.tf_valid = std::isfinite(yaw);
        } catch (const tf2::TransformException &) {
          input.tf_valid = false;
        }
      }
    }

    const RearSafetyResult res = evaluator.evaluate(input);

    if (!has_last_reason_ || res.reason != last_reason_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "CheckRearSafety: %s (%s) | age=%.3fs | req_reach=%.2fm",
        toString(res.reason), res.detail.c_str(), res.scan_age, res.required_scan_reach);
      last_reason_ = res.reason;
      has_last_reason_ = true;
    }

    return res.safe ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::Buffer> owned_tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  std::atomic<bool> stop_thread_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_sub_;

  std::mutex mutex_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr latest_health_;
  bool has_last_reason_;
  RearSafetyReason last_reason_;
};

}  // namespace nav2_custom_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_plugins::CheckRearSafety>("CheckRearSafety");
}
