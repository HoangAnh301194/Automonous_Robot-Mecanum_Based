#ifndef NAV2_CUSTOM_PLUGINS__CUSTOM_BACKUP_EVALUATOR_HPP_
#define NAV2_CUSTOM_PLUGINS__CUSTOM_BACKUP_EVALUATOR_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_custom_plugins/rear_safety_evaluator.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/time.hpp"

namespace nav2_custom_plugins
{

enum class BackupFailureCode : uint8_t
{
  NONE = 0,
  INVALID_GOAL,
  REAR_OBSTACLE,
  SENSOR_STALE,
  SENSOR_INVALID,
  TF_ERROR,
  COSTMAP_INVALID,
  TRAJECTORY_BLOCKED,
  TIMEOUT,
  CANCELED
};

const char * toString(BackupFailureCode code);

BackupFailureCode mapRearReason(RearSafetyReason reason);

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CustomBackUpConfig
{
  double costmap_timeout{2.5};
  double footprint_timeout{2.5};
  std::string expected_costmap_frame{"odom"};
  uint8_t lethal_cost_threshold{254};

  double step_resolution_factor{0.4};
  double max_step_m{0.025};
  double rear_stop_margin{0.05};
  double rear_corner_lateral_margin{0.04};
  double rear_corner_band_length{0.18};

  double front_overlap_min_x{0.08};
  double max_front_overlap_depth{0.23};
  double max_front_overlap_ratio{0.35};
  double whitelist_match_radius{0.085};
  double max_forward_push_m{0.04};

  RearSafetyConfig rear;
};

struct BackupSessionState
{
  bool active{false};
  bool front_overlap_accepted{false};
  bool front_overlap_cleared{false};
  size_t initial_front_overlap_count{0};
  double initial_front_overlap_penetration{0.0};
  double initial_min_front_x_rel{std::numeric_limits<double>::infinity()};
  std::vector<geometry_msgs::msg::Point> initial_front_world_pts;
  int consecutive_clear_cycles{0};
};

struct BackupEvalInput
{
  bool pose_valid{false};
  Pose2D current_pose;
  double remaining_dist{0.0};
  double traveled_signed{0.0};
  double current_speed{0.0};
  rclcpp::Time now{0, 0, RCL_ROS_TIME};

  nav2_msgs::msg::Costmap::ConstSharedPtr costmap;
  std::vector<geometry_msgs::msg::Point> footprint_in_base;
  bool footprint_valid{false};
  rclcpp::Time footprint_stamp{0, 0, RCL_ROS_TIME};

  RearSafetyInput rear_input;
};

struct BackupEvalResult
{
  bool safe{false};
  BackupFailureCode failure_code{BackupFailureCode::TRAJECTORY_BLOCKED};
  std::string detail;

  bool front_overlap_accepted{false};
  size_t front_overlap_count{0};
  double front_overlap_penetration{0.0};

  RearSafetyResult rear_result;
  int samples_checked{0};
};

class CustomBackUpEvaluator
{
public:
  explicit CustomBackUpEvaluator(const CustomBackUpConfig & config = CustomBackUpConfig());

  BackupEvalResult initializeAndEvaluateStart(
    const BackupEvalInput & input,
    BackupSessionState & session) const;

  BackupEvalResult evaluateCycle(
    const BackupEvalInput & input,
    BackupSessionState & session) const;

  const CustomBackUpConfig & config() const {return config_;}

private:
  BackupEvalResult evaluateCommon(
    const BackupEvalInput & input,
    BackupSessionState & session,
    bool is_start) const;

  CustomBackUpConfig config_;
  RearSafetyEvaluator rear_evaluator_;
};

}  // namespace nav2_custom_plugins

#endif  // NAV2_CUSTOM_PLUGINS__CUSTOM_BACKUP_EVALUATOR_HPP_

