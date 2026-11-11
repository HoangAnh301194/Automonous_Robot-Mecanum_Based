#ifndef NAV2_CUSTOM_PLUGINS__REAR_SAFETY_EVALUATOR_HPP_
#define NAV2_CUSTOM_PLUGINS__REAR_SAFETY_EVALUATOR_HPP_

#include <cstdint>
#include <limits>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_custom_plugins
{

enum class RearSafetyReason : uint8_t
{
  SAFE = 0,
  REAR_OBSTACLE,
  SENSOR_STALE,
  SENSOR_INVALID,
  TF_ERROR
};

const char * toString(RearSafetyReason reason);

struct RearSafetyConfig
{
  double min_distance{0.55};
  double scan_timeout{0.50};
  double health_timeout{0.50};
  double max_scan_health_skew{0.35};
  double max_future_skew{0.25};

  double robot_rear_x{-0.35};
  double robot_front_x{0.31};
  double robot_half_width{0.28};
  double corridor_lateral_margin{0.07};  // 0.28 + 0.07 = 0.35 m half corridor
  double rear_safety_margin{0.10};
  double control_latency{0.10};
  double braking_decel{1.0};

  double max_nan_ratio{0.25};
  bool require_health{true};

  // Geometric observability thresholds for all-inf / sparse LaserScan verification
  double min_corridor_observed_ratio{0.60};
  double min_near_zone_observed_ratio{0.55};
  double min_center_observed_ratio{0.60};
  double min_sector_coverage_ratio{0.70};
  double max_observed_angle_min{-0.32};
  double min_observed_angle_max{0.32};
  double max_observed_y_min{-0.24};
  double min_observed_y_max{0.24};
  double max_observed_x_min{0.50};
  double min_required_x_max{0.55};
  double max_required_reach{1.20};
};

struct ParsedRearHealth
{
  bool valid{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  uint8_t level{2};  // DiagnosticStatus::ERROR = 2

  bool has_status{false};
  bool has_observability_metrics{false};
  bool has_center_observed_ratio{false};

  bool camera_ok{false};
  bool tf_ok{false};
  bool ground_ok{false};
  bool corridor_observed{false};

  double valid_ratio{0.0};
  double current_valid_ratio{0.0};
  double corridor_observed_ratio{0.0};
  double near_zone_observed_ratio{0.0};
  double center_observed_ratio{0.0};
  double sector_coverage_ratio{0.0};
  double observed_angle_min{0.0};
  double observed_angle_max{0.0};
  double observed_x_min{0.0};
  double observed_x_max{0.0};
  double observed_y_min{0.0};
  double observed_y_max{0.0};
};

struct RearSafetyInput
{
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr health;
  rclcpp::Time now{0, 0, RCL_ROS_TIME};

  // Rigid 2D transform from scan frame into robot_base_frame (base_footprint):
  // x_base = scan_to_base_tx + cos(yaw)*x_scan - sin(yaw)*y_scan
  // y_base = scan_to_base_ty + sin(yaw)*x_scan + cos(yaw)*y_scan
  bool tf_valid{false};
  double scan_to_base_tx{0.0};
  double scan_to_base_ty{0.0};
  double scan_to_base_yaw{0.0};

  double remaining_dist{0.30};
  double current_speed{0.10};
};

struct RearSafetyResult
{
  bool safe{false};
  RearSafetyReason reason{RearSafetyReason::SENSOR_INVALID};
  std::string detail;

  double scan_age{0.0};
  double health_age{0.0};
  double min_obstacle_dist_scan{std::numeric_limits<double>::infinity()};
  double min_bumper_clearance{std::numeric_limits<double>::infinity()};
  double required_bumper_clearance{0.0};
  double required_scan_reach{0.0};

  int finite_count{0};
  int pos_inf_count{0};
  int invalid_count{0};
  bool corridor_observed{false};
};

class RearSafetyEvaluator
{
public:
  explicit RearSafetyEvaluator(const RearSafetyConfig & config = RearSafetyConfig());

  static ParsedRearHealth parseHealth(const diagnostic_msgs::msg::DiagnosticArray & msg);

  RearSafetyResult evaluate(const RearSafetyInput & input) const;

  const RearSafetyConfig & config() const {return config_;}

private:
  RearSafetyConfig config_;
};

}  // namespace nav2_custom_plugins

#endif  // NAV2_CUSTOM_PLUGINS__REAR_SAFETY_EVALUATOR_HPP_

