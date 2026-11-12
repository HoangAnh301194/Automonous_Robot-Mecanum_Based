#ifndef NAV2_CUSTOM_PLUGINS__CUSTOM_BACKUP_HPP_
#define NAV2_CUSTOM_PLUGINS__CUSTOM_BACKUP_HPP_

#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_custom_plugins/custom_backup_evaluator.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_custom_plugins
{

class CustomBackUp : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;

  CustomBackUp();
  ~CustomBackUp() override = default;

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const BackUpAction::Goal> command) override;

  nav2_behaviors::Status onCycleUpdate() override;

  void onConfigure() override;

  void onCleanup() override;

  void onActionCompletion() override;

private:
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void rearScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void rearHealthCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool populateCurrentPose(Pose2D & pose_out);
  bool populateRearTf(RearSafetyInput & rear_in, const std::string & scan_frame);
  bool populateFootprintInBase(
    std::vector<geometry_msgs::msg::Point> & fp_out,
    rclcpp::Time & stamp_out);
  BackupEvalInput buildEvalInput(
    double remaining_dist,
    double traveled_signed,
    double current_speed,
    const rclcpp::Time & now,
    bool & pose_ok,
    Pose2D & current_pose_out);

  void logFailedOnce(BackupFailureCode code, double dist, const std::string & debug_detail);

  BackUpAction::Feedback::SharedPtr feedback_;

  // Configuration
  std::string local_frame_{"odom"};
  std::string local_costmap_topic_{"local_costmap/costmap_raw"};
  std::string local_footprint_topic_{"local_costmap/published_footprint"};
  std::string rear_scan_topic_{"/rear_scan_obstacle"};
  std::string rear_health_topic_{"/rear_scan_obstacle/health"};
  std::string odom_topic_{"/odom"};

  double acceleration_limit_{0.0};
  double deceleration_limit_{0.0};
  double minimum_speed_{0.0};
  double max_backup_distance_{2.0};
  double max_backup_speed_{0.5};
  double partial_success_min_distance_{0.08};

  CustomBackUpConfig eval_config_;
  std::unique_ptr<CustomBackUpEvaluator> evaluator_;

  // Subscriptions
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr local_footprint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr rear_health_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::mutex data_mutex_;
  nav2_msgs::msg::Costmap::ConstSharedPtr latest_costmap_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr latest_footprint_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_rear_scan_;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr latest_rear_health_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_;

  // Active execution state
  geometry_msgs::msg::PoseStamped initial_pose_;
  double initial_yaw_{0.0};
  double target_dist_{0.0};
  double cmd_speed_{0.0};
  double time_allowance_sec_{0.0};
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time end_time_{0, 0, RCL_ROS_TIME};
  double last_vel_{std::numeric_limits<double>::max()};
  double last_traveled_dist_{0.0};
  bool completed_or_failed_{false};
  bool motion_started_{false};
  bool pending_partial_stop_{false};

  BackupSessionState session_;
  StopConfirmState stop_state_;
};

}  // namespace nav2_custom_plugins

#endif  // NAV2_CUSTOM_PLUGINS__CUSTOM_BACKUP_HPP_

