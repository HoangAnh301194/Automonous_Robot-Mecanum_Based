#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/behavior.hpp"
#include "nav2_custom_plugins/custom_backup.hpp"
#include "nav2_custom_plugins/custom_backup_evaluator.hpp"
#include "nav2_custom_plugins/rear_safety_evaluator.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcutils/logging.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

namespace
{

constexpr double kPi = 3.14159265358979323846;

diagnostic_msgs::msg::KeyValue makeKV(const std::string & k, const std::string & v)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = k;
  kv.value = v;
  return kv;
}

sensor_msgs::msg::LaserScan::SharedPtr makeRearScan(
  const rclcpp::Time & stamp,
  int num_bins = 360,
  double finite_range = std::numeric_limits<double>::infinity(),
  double finite_angle = 0.0,
  double finite_half_width_rad = 0.05)
{
  auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  msg->header.stamp = stamp;
  msg->header.frame_id = "rear_scan_frame";
  msg->angle_min = -static_cast<float>(kPi / 2.0);
  msg->angle_max = static_cast<float>(kPi / 2.0);
  msg->angle_increment = static_cast<float>(kPi / static_cast<double>(num_bins));
  msg->range_min = 0.2f;
  msg->range_max = 2.5f;
  msg->ranges.assign(
    static_cast<size_t>(num_bins),
    std::numeric_limits<float>::infinity());

  if (std::isfinite(finite_range)) {
    for (int i = 0; i < num_bins; ++i) {
      const double theta = static_cast<double>(msg->angle_min) +
        static_cast<double>(i) * static_cast<double>(msg->angle_increment);
      if (std::abs(theta - finite_angle) <= finite_half_width_rad) {
        msg->ranges[static_cast<size_t>(i)] = static_cast<float>(finite_range);
      }
    }
  }
  return msg;
}

diagnostic_msgs::msg::DiagnosticArray::SharedPtr makeRearHealth(
  const rclcpp::Time & stamp,
  bool corridor_observed = true,
  double valid_ratio = 0.95,
  double corr_ratio = 0.92,
  double near_ratio = 0.95,
  double center_ratio = 0.94,
  double sector_ratio = 1.0,
  double angle_min = -0.425,
  double angle_max = 0.425,
  double x_min = 0.16,
  double x_max = 1.40,
  double y_min = -0.35,
  double y_max = 0.35,
  bool include_geom_metrics = true,
  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK,
  bool cam_ok = true,
  bool tf_ok = true,
  bool ground_ok = true)
{
  auto arr = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
  arr->header.stamp = stamp;
  arr->header.frame_id = "rear_scan_frame";

  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "rear_depth_obstacle_detector";
  st.level = level;
  st.message = corridor_observed ? "OK" : "REAR_CORRIDOR_NOT_OBSERVED";

  st.values.push_back(makeKV("camera_ok", cam_ok ? "true" : "false"));
  st.values.push_back(makeKV("tf_ok", tf_ok ? "true" : "false"));
  st.values.push_back(makeKV("ground_ok", ground_ok ? "true" : "false"));
  st.values.push_back(makeKV("corridor_observed", corridor_observed ? "true" : "false"));
  st.values.push_back(makeKV("valid_ratio", std::to_string(valid_ratio)));

  if (include_geom_metrics) {
    st.values.push_back(makeKV("current_valid_ratio", std::to_string(valid_ratio)));
    st.values.push_back(makeKV("corridor_observed_ratio", std::to_string(corr_ratio)));
    st.values.push_back(makeKV("near_zone_observed_ratio", std::to_string(near_ratio)));
    st.values.push_back(makeKV("center_observed_ratio", std::to_string(center_ratio)));
    st.values.push_back(makeKV("sector_coverage_ratio", std::to_string(sector_ratio)));
    st.values.push_back(makeKV("observed_angle_min", std::to_string(angle_min)));
    st.values.push_back(makeKV("observed_angle_max", std::to_string(angle_max)));
    st.values.push_back(makeKV("observed_x_min", std::to_string(x_min)));
    st.values.push_back(makeKV("observed_x_max", std::to_string(x_max)));
    st.values.push_back(makeKV("observed_y_min", std::to_string(y_min)));
    st.values.push_back(makeKV("observed_y_max", std::to_string(y_max)));
  }

  arr->status.push_back(st);
  return arr;
}

std::vector<geometry_msgs::msg::Point> makeRobotFootprint()
{
  std::vector<geometry_msgs::msg::Point> fp(4);
  fp[0].x = 0.31;
  fp[0].y = 0.28;
  fp[1].x = 0.31;
  fp[1].y = -0.28;
  fp[2].x = -0.35;
  fp[2].y = -0.28;
  fp[3].x = -0.35;
  fp[3].y = 0.28;
  return fp;
}

nav2_msgs::msg::Costmap::SharedPtr makeCostmap(
  const rclcpp::Time & stamp,
  double origin_x = -1.5,
  double origin_y = -1.5,
  uint32_t size_x = 30,
  uint32_t size_y = 30,
  float res = 0.1f,
  const std::string & frame_id = "odom")
{
  auto cm = std::make_shared<nav2_msgs::msg::Costmap>();
  cm->header.stamp = stamp;
  cm->header.frame_id = frame_id;
  cm->metadata.map_load_time = stamp;
  cm->metadata.update_time = stamp;
  cm->metadata.layer = "master";
  cm->metadata.resolution = res;
  cm->metadata.size_x = size_x;
  cm->metadata.size_y = size_y;
  cm->metadata.origin.position.x = origin_x;
  cm->metadata.origin.position.y = origin_y;
  cm->metadata.origin.orientation.w = 1.0;
  cm->data.assign(static_cast<size_t>(size_x) * static_cast<size_t>(size_y), 0);
  return cm;
}

void setCostAtWorld(
  nav2_msgs::msg::Costmap & cm,
  double wx,
  double wy,
  uint8_t cost)
{
  const int mx = static_cast<int>(
    std::floor((wx - cm.metadata.origin.position.x) / cm.metadata.resolution));
  const int my = static_cast<int>(
    std::floor((wy - cm.metadata.origin.position.y) / cm.metadata.resolution));
  ASSERT_GE(mx, 0);
  ASSERT_LT(mx, static_cast<int>(cm.metadata.size_x));
  ASSERT_GE(my, 0);
  ASSERT_LT(my, static_cast<int>(cm.metadata.size_y));
  cm.data[static_cast<size_t>(my) * cm.metadata.size_x + static_cast<size_t>(mx)] = cost;
}

nav2_custom_plugins::RearSafetyInput makeDefaultRearInput(
  const rclcpp::Time & now,
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan,
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr health)
{
  nav2_custom_plugins::RearSafetyInput in;
  in.now = now;
  in.scan = std::move(scan);
  in.health = std::move(health);
  in.tf_valid = true;
  in.scan_to_base_tx = -0.25;
  in.scan_to_base_ty = 0.0;
  in.scan_to_base_yaw = kPi;
  in.remaining_dist = 0.30;
  in.current_speed = 0.10;
  return in;
}

nav2_custom_plugins::BackupEvalInput makeDefaultEvalInput(
  const rclcpp::Time & now,
  const nav2_custom_plugins::Pose2D & pose = {0.0, 0.0, 0.0},
  double rem_dist = 0.30,
  double traveled_signed = 0.0)
{
  nav2_custom_plugins::BackupEvalInput in;
  in.now = now;
  in.pose_valid = true;
  in.current_pose = pose;
  in.remaining_dist = rem_dist;
  in.traveled_signed = traveled_signed;
  in.current_speed = 0.10;
  in.costmap = makeCostmap(now);
  in.footprint_in_base = makeRobotFootprint();
  in.footprint_valid = true;
  in.footprint_stamp = now;
  in.rear_input = makeDefaultRearInput(now, makeRearScan(now), makeRearHealth(now));
  return in;
}

nav_msgs::msg::Odometry::SharedPtr makeOdometry(
  const rclcpp::Time & stamp,
  double x = 0.0,
  double y = 0.0,
  double vx = 0.0,
  double wz = 0.0)
{
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.stamp = stamp;
  odom->header.frame_id = "odom";
  odom->child_frame_id = "base_footprint";
  odom->pose.pose.position.x = x;
  odom->pose.pose.position.y = y;
  odom->pose.pose.orientation.w = 1.0;
  odom->twist.twist.linear.x = vx;
  odom->twist.twist.linear.y = 0.0;
  odom->twist.twist.angular.z = wz;
  return odom;
}

struct CapturedLogEntry
{
  int severity{0};
  std::string message;
};

class ScopedLogCapture
{
public:
  ScopedLogCapture()
  {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    active_instance_ = this;
    prev_handler_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&ScopedLogCapture::outputHandler);
  }

  ~ScopedLogCapture()
  {
    rcutils_logging_set_output_handler(prev_handler_);
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (active_instance_ == this) {
      active_instance_ = nullptr;
    }
  }

  int countContaining(const std::string & substr, int severity = -1) const
  {
    std::lock_guard<std::mutex> lock(entries_mutex_);
    int count = 0;
    for (const auto & e : entries_) {
      if (severity >= 0 && e.severity != severity) {
        continue;
      }
      if (e.message.find(substr) != std::string::npos) {
        ++count;
      }
    }
    return count;
  }

private:
  static void outputHandler(
    const rcutils_log_location_t * location,
    int severity,
    const char * name,
    rcutils_time_point_value_t timestamp,
    const char * format,
    va_list * args)
  {
    std::string formatted;
    if (format != nullptr && args != nullptr) {
      va_list args_copy;
      va_copy(args_copy, *args);
      char buf[1024];
      const int n = std::vsnprintf(buf, sizeof(buf), format, args_copy);
      va_end(args_copy);
      if (n > 0) {
        formatted.assign(buf, static_cast<size_t>(std::min<int>(n, sizeof(buf) - 1)));
      }
    }

    rcutils_logging_output_handler_t next = nullptr;
    {
      std::lock_guard<std::mutex> lock(instance_mutex_);
      if (active_instance_ != nullptr) {
        next = active_instance_->prev_handler_;
        std::lock_guard<std::mutex> elock(active_instance_->entries_mutex_);
        active_instance_->entries_.push_back({severity, formatted});
      }
    }

    if (next != nullptr && args != nullptr) {
      va_list args_fwd;
      va_copy(args_fwd, *args);
      next(location, severity, name, timestamp, format, &args_fwd);
      va_end(args_fwd);
    }
  }

  rcutils_logging_output_handler_t prev_handler_{nullptr};
  mutable std::mutex entries_mutex_;
  std::vector<CapturedLogEntry> entries_;

  static std::mutex instance_mutex_;
  static ScopedLogCapture * active_instance_;
};

std::mutex ScopedLogCapture::instance_mutex_;
ScopedLogCapture * ScopedLogCapture::active_instance_ = nullptr;

}  // namespace

TEST(RearSafetyEvaluatorTest, ClearAllInfWithFullObservabilityHealthIsSafe)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now), makeRearHealth(now)));
  EXPECT_TRUE(res.safe);
  EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::SAFE);
}

TEST(RearSafetyEvaluatorTest, AllInfWithoutHealthIsRejected)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now), nullptr));
  EXPECT_FALSE(res.safe);
  EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_STALE);
}

TEST(RearSafetyEvaluatorTest, AllInfHighValidRatioButUnobservedCorridorIsRejected)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  auto health = makeRearHealth(now, /*corridor_observed=*/false, /*valid_ratio=*/0.96);
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now), health));
  EXPECT_FALSE(res.safe);
  EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_INVALID);
}

TEST(RearSafetyEvaluatorTest, AllInfSpoofedOkWithoutGeometricMetricsIsRejected)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  auto health = makeRearHealth(
    now, true, 0.98, 0.95, 0.95, 0.95, 1.0, -0.4, 0.4, 0.15, 1.4, -0.35, 0.35,
    /*include_geom_metrics=*/false);
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now), health));
  EXPECT_FALSE(res.safe);
  EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_INVALID);
}

TEST(RearSafetyEvaluatorTest, AllInfHalfCoveredAsymmetricAngleIsRejected)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  auto health = makeRearHealth(
    now, true, 0.65, 0.65, 0.65, 0.65, 0.50, /*angle_min=*/-0.05, /*angle_max=*/0.42);
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now), health));
  EXPECT_FALSE(res.safe);
  EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_INVALID);
}

TEST(RearSafetyEvaluatorTest, AllInfInsufficientReachIsRejected)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  auto health = makeRearHealth(
    now, true, 0.95, 0.90, 0.90, 0.90, 1.0, -0.4, 0.4, 0.15, /*x_max=*/0.42);
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now), health));
  EXPECT_FALSE(res.safe);
  EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_INVALID);
}

TEST(RearSafetyEvaluatorTest, RearObstacleInsideThresholdBlocksEvenWhenHealthOk)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now, 360, 0.48, 0.0, 0.05), makeRearHealth(now)));
  EXPECT_FALSE(res.safe);
  EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::REAR_OBSTACLE);
}

TEST(RearSafetyEvaluatorTest, ObstacleFarOutsideLateralCorridorIsAllowed)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  // r=1.0, angle=1.0 rad => y_s = 0.84 m (outside +/-0.35 m corridor)
  const auto res = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(now, 360, 1.0, 1.0, 0.03), makeRearHealth(now)));
  EXPECT_TRUE(res.safe);
}

TEST(RearSafetyEvaluatorTest, StaleScanAndBadTfAreRejected)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  const rclcpp::Time stale_t(99, 0, RCL_ROS_TIME);  // 1.0 s old > 0.50 s
  const auto res_stale = eval.evaluate(
    makeDefaultRearInput(now, makeRearScan(stale_t), makeRearHealth(stale_t)));
  EXPECT_FALSE(res_stale.safe);
  EXPECT_EQ(res_stale.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_STALE);

  auto tf_bad = makeDefaultRearInput(now, makeRearScan(now), makeRearHealth(now));
  tf_bad.tf_valid = false;
  const auto res_tf = eval.evaluate(tf_bad);
  EXPECT_FALSE(res_tf.safe);
  EXPECT_EQ(res_tf.reason, nav2_custom_plugins::RearSafetyReason::TF_ERROR);
}

TEST(RearSafetyEvaluatorTest, ZeroPointFiveZeroMinDistanceAndDynamicClearancePriority)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  EXPECT_DOUBLE_EQ(eval.config().min_distance, 0.50);
  EXPECT_DOUBLE_EQ(eval.config().min_required_x_max, 0.50);

  const rclcpp::Time now(100, 0, RCL_ROS_TIME);

  // Small remaining_dist (0.05 m) & stationary speed (0.0): dynamic clearance (0.15 m) < baseline (0.40 m).
  // Therefore 0.50 m min_distance (= 0.40 m bumper clearance from rear_scan at tx=-0.25) governs:
  {
    auto in_block_495 = makeDefaultRearInput(now, makeRearScan(now, 360, 0.495, 0.0, 0.05), makeRearHealth(now));
    in_block_495.remaining_dist = 0.05;
    in_block_495.current_speed = 0.0;
    const auto r1 = eval.evaluate(in_block_495);
    EXPECT_FALSE(r1.safe);
    EXPECT_EQ(r1.reason, nav2_custom_plugins::RearSafetyReason::REAR_OBSTACLE);
    EXPECT_NEAR(r1.required_scan_reach, 0.50, 1e-6);

    auto in_block_500 = makeDefaultRearInput(now, makeRearScan(now, 360, 0.500, 0.0, 0.05), makeRearHealth(now));
    in_block_500.remaining_dist = 0.05;
    in_block_500.current_speed = 0.0;
    const auto r2 = eval.evaluate(in_block_500);
    EXPECT_FALSE(r2.safe);
    EXPECT_EQ(r2.reason, nav2_custom_plugins::RearSafetyReason::REAR_OBSTACLE);

    auto in_safe_515 = makeDefaultRearInput(now, makeRearScan(now, 360, 0.515, 0.0, 0.05), makeRearHealth(now));
    in_safe_515.remaining_dist = 0.05;
    in_safe_515.current_speed = 0.0;
    const auto r3 = eval.evaluate(in_safe_515);
    EXPECT_TRUE(r3.safe);
    EXPECT_EQ(r3.reason, nav2_custom_plugins::RearSafetyReason::SAFE);
  }

  // Higher speed / full 0.30 m remaining_dist: dynamic clearance > baseline (0.40 m) MUST still win!
  // remaining=0.30, speed=0.25 -> latency=0.025, braking=0.03125, margin=0.10 => bumper req=0.45625 => scan req=0.55625 m.
  {
    auto in_dyn = makeDefaultRearInput(now, makeRearScan(now, 360, 0.52, 0.0, 0.05), makeRearHealth(now));
    in_dyn.remaining_dist = 0.30;
    in_dyn.current_speed = 0.25;
    const auto r_dyn = eval.evaluate(in_dyn);
    EXPECT_FALSE(r_dyn.safe);
    EXPECT_EQ(r_dyn.reason, nav2_custom_plugins::RearSafetyReason::REAR_OBSTACLE);
    EXPECT_GT(r_dyn.required_scan_reach, 0.55);
  }
}

TEST(RearSafetyEvaluatorTest, HealthyOccludingObstacleReturnsRearObstacleWhileBrokenCameraRejectsIt)
{
  nav2_custom_plugins::RearSafetyEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);

  // Healthy rear camera noticing an obstacle close behind that occludes floor beyond (WARN + corridor_observed=false)
  {
    auto health = makeRearHealth(
      now, /*corridor_observed=*/false, 0.50, 0.30, 0.40, 0.30, 0.50,
      -0.4, 0.4, 0.16, 0.42, -0.35, 0.35, true,
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      /*cam_ok=*/true, /*tf_ok=*/true, /*ground_ok=*/true);
    const auto res = eval.evaluate(
      makeDefaultRearInput(now, makeRearScan(now, 360, 0.40, 0.0, 0.05), health));
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.reason, nav2_custom_plugins::RearSafetyReason::REAR_OBSTACLE);
  }

  // Unhealthy camera (camera_ok=false or stale health) with finite obstacle ray must report SENSOR_INVALID / SENSOR_STALE
  {
    auto bad_cam = makeRearHealth(
      now, false, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true,
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      /*cam_ok=*/false, /*tf_ok=*/true, /*ground_ok=*/true);
    const auto res_cam = eval.evaluate(
      makeDefaultRearInput(now, makeRearScan(now, 360, 0.40, 0.0, 0.05), bad_cam));
    EXPECT_FALSE(res_cam.safe);
    EXPECT_EQ(res_cam.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_INVALID);

    const rclcpp::Time old_health_t(98, 0, RCL_ROS_TIME);
    const auto res_stale = eval.evaluate(
      makeDefaultRearInput(now, makeRearScan(now, 360, 0.40, 0.0, 0.05), makeRearHealth(old_health_t)));
    EXPECT_FALSE(res_stale.safe);
    EXPECT_EQ(res_stale.reason, nav2_custom_plugins::RearSafetyReason::SENSOR_STALE);
  }
}

TEST(CustomBackUpEvaluatorTest, ClearFrontAndClearRearIsAccepted)
{
  nav2_custom_plugins::CustomBackUpEvaluator eval;
  nav2_custom_plugins::BackupSessionState session;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);
  const auto res = eval.initializeAndEvaluateStart(makeDefaultEvalInput(now), session);
  EXPECT_TRUE(res.safe);
  EXPECT_FALSE(res.front_overlap_accepted);
}

TEST(CustomBackUpEvaluatorTest, RearSweepAndRearCornersAreBlocked)
{
  nav2_custom_plugins::CustomBackUpEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);

  // 1) Directly behind rear edge along backup sweep (-0.55, 0.0)
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(now);
    auto cm = makeCostmap(now);
    setCostAtWorld(*cm, -0.55, 0.0, 254);
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }

  // 2) Rear-left corner along backup sweep (-0.50, +0.30)
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(now);
    auto cm = makeCostmap(now);
    setCostAtWorld(*cm, -0.50, 0.30, 254);
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }

  // 3) Rear-right corner along backup sweep (-0.50, -0.30)
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(now);
    auto cm = makeCostmap(now);
    setCostAtWorld(*cm, -0.50, -0.30, 254);
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }
}

TEST(CustomBackUpEvaluatorTest, NarrowCorridorSafeVsEncroachingWallRejected)
{
  nav2_custom_plugins::CustomBackUpEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);

  // Walls at y = +/- 0.45 m leave plenty of margin -> SAFE
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(now);
    auto cm = makeCostmap(now);
    for (double x = -0.9; x <= 0.6; x += 0.1) {
      setCostAtWorld(*cm, x, 0.45, 254);
      setCostAtWorld(*cm, x, -0.45, 254);
    }
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_TRUE(res.safe);
  }

  // Wall encroaching into rear corner margin at y = +0.31 m -> BLOCKED
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(now);
    auto cm = makeCostmap(now);
    setCostAtWorld(*cm, -0.45, 0.31, 254);
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }
}

TEST(CustomBackUpEvaluatorTest, UnknownSpaceAndStaleCostmapAreRejected)
{
  nav2_custom_plugins::CustomBackUpEvaluator eval;
  const rclcpp::Time now(100, 0, RCL_ROS_TIME);

  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(now);
    auto cm = makeCostmap(now);
    setCostAtWorld(*cm, -0.55, 0.0, 255);  // NO_INFORMATION
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }

  {
    nav2_custom_plugins::BackupSessionState session;
    const rclcpp::Time old_t(95, 0, RCL_ROS_TIME);
    auto in = makeDefaultEvalInput(now);
    in.costmap = makeCostmap(old_t);
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::COSTMAP_INVALID);
  }
}

TEST(CustomBackUpEvaluatorTest, PreExistingFrontOverlapRecedesEvenAcrossRollingWindowShift)
{
  nav2_custom_plugins::CustomBackUpEvaluator eval;
  nav2_custom_plugins::BackupSessionState session;
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);

  auto in0 = makeDefaultEvalInput(t0, {0.0, 0.0, 0.0}, 0.30, 0.0);
  auto cm0 = makeCostmap(t0, -1.5, -1.5);
  // Person standing right in front: lethal cell at world x=+0.25, y=+0.05 (inside [0.20, 0.30])
  setCostAtWorld(*cm0, 0.25, 0.05, 254);
  in0.costmap = cm0;

  const auto start_res = eval.initializeAndEvaluateStart(in0, session);
  ASSERT_TRUE(start_res.safe);
  EXPECT_TRUE(start_res.front_overlap_accepted);
  EXPECT_GT(start_res.front_overlap_count, 0u);

  // Simulate moving back 0.15 m with rolling-window costmap origin shifted by (-0.20, 0.0)!
  const rclcpp::Time t1(101, 0, RCL_ROS_TIME);
  auto in1 = makeDefaultEvalInput(t1, {-0.15, 0.0, 0.0}, 0.15, 0.15);
  auto cm1 = makeCostmap(t1, -1.7, -1.5);
  setCostAtWorld(*cm1, 0.25, 0.05, 254);
  in1.costmap = cm1;
  const auto cyc_res = eval.evaluateCycle(in1, session);
  EXPECT_TRUE(cyc_res.safe);
}

TEST(CustomBackUpEvaluatorTest, DeepOrMidOverlapAtStartAndSteppingForwardMidBackupAreBlocked)
{
  nav2_custom_plugins::CustomBackUpEvaluator eval;
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);

  // 1) Deep front overlap (< 0.08 m) rejected
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(t0);
    auto cm = makeCostmap(t0);
    setCostAtWorld(*cm, 0.05, 0.05, 254);
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }

  // 2) Short goal 0.10 m cannot clear overlap at +0.15 m -> rejected
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in = makeDefaultEvalInput(t0, {0.0, 0.0, 0.0}, 0.10, 0.0);
    auto cm = makeCostmap(t0);
    setCostAtWorld(*cm, 0.15, 0.05, 254);
    in.costmap = cm;
    const auto res = eval.initializeAndEvaluateStart(in, session);
    EXPECT_FALSE(res.safe);
    EXPECT_EQ(res.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }

  // 3) Shallow front overlap accepted at t0, then person steps deeper (+0.12 m) at t1 -> SAFETY_STOP
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in0 = makeDefaultEvalInput(t0, {0.0, 0.0, 0.0}, 0.30, 0.0);
    auto cm0 = makeCostmap(t0);
    setCostAtWorld(*cm0, 0.25, 0.05, 254);
    in0.costmap = cm0;
    ASSERT_TRUE(eval.initializeAndEvaluateStart(in0, session).safe);

    const rclcpp::Time t1(100, 500000000, RCL_ROS_TIME);
    auto in1 = makeDefaultEvalInput(t1, {-0.05, 0.0, 0.0}, 0.25, 0.05);
    auto cm1 = makeCostmap(t1);
    setCostAtWorld(*cm1, 0.12, 0.05, 254);  // Stepped forward outside whitelist!
    in1.costmap = cm1;
    const auto res1 = eval.evaluateCycle(in1, session);
    EXPECT_FALSE(res1.safe);
    EXPECT_EQ(res1.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }

  // 4) Forward physical push guard
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in0 = makeDefaultEvalInput(t0);
    ASSERT_TRUE(eval.initializeAndEvaluateStart(in0, session).safe);
    auto in1 = makeDefaultEvalInput(t0, {0.06, 0.0, 0.0}, 0.30, -0.06);
    const auto res1 = eval.evaluateCycle(in1, session);
    EXPECT_FALSE(res1.safe);
    EXPECT_EQ(res1.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);
  }
}

TEST(CustomBackUpEvaluatorTest, PartialStopCycleAndOdometryStopConfirmationLogic)
{
  nav2_custom_plugins::CustomBackUpEvaluator eval;
  const rclcpp::Time t0(100, 0, RCL_ROS_TIME);

  // 1) evaluatePartialStopCycle allows REAR_OBSTACLE when camera/TF/costmap are healthy,
  //    but rejects invalid costmap, unknown cell under footprint, or unhealthy camera.
  {
    nav2_custom_plugins::BackupSessionState session;
    auto in_start = makeDefaultEvalInput(t0, {0.0, 0.0, 0.0}, 0.30, 0.0);
    ASSERT_TRUE(eval.initializeAndEvaluateStart(in_start, session).safe);

    const rclcpp::Time t1(101, 0, RCL_ROS_TIME);
    auto in_stop = makeDefaultEvalInput(t1, {-0.15, 0.0, 0.0}, 0.15, 0.15);
    in_stop.rear_input.scan = makeRearScan(t1, 360, 0.40, 0.0, 0.05);
    in_stop.rear_input.health = makeRearHealth(t1);
    EXPECT_TRUE(eval.evaluatePartialStopCycle(in_stop, session).safe);

    // Invalid/stale costmap rejects partial stop
    auto in_bad_cm = in_stop;
    in_bad_cm.costmap = makeCostmap(rclcpp::Time(90, 0, RCL_ROS_TIME));
    const auto r_cm = eval.evaluatePartialStopCycle(in_bad_cm, session);
    EXPECT_FALSE(r_cm.safe);
    EXPECT_EQ(r_cm.failure_code, nav2_custom_plugins::BackupFailureCode::COSTMAP_INVALID);

    // Unknown cell under robot footprint rejects partial stop
    auto in_unk = in_stop;
    auto cm_unk = makeCostmap(t1);
    setCostAtWorld(*cm_unk, -0.15, 0.0, 255);
    in_unk.costmap = cm_unk;
    const auto r_unk = eval.evaluatePartialStopCycle(in_unk, session);
    EXPECT_FALSE(r_unk.safe);
    EXPECT_EQ(r_unk.failure_code, nav2_custom_plugins::BackupFailureCode::TRAJECTORY_BLOCKED);

    // Unhealthy camera rejects partial stop
    auto in_bad_cam = in_stop;
    in_bad_cam.rear_input.health = nullptr;
    const auto r_cam = eval.evaluatePartialStopCycle(in_bad_cam, session);
    EXPECT_FALSE(r_cam.safe);
    EXPECT_EQ(r_cam.failure_code, nav2_custom_plugins::BackupFailureCode::SENSOR_STALE);
  }

  // 2) evaluateStopConfirmation waits for post-trigger zero twist, settled pose, distinct odom & dwell
  {
    nav2_custom_plugins::StopConfirmConfig cfg;
    nav2_custom_plugins::StopConfirmState st;
    st.active = true;
    st.trigger_time = t0;

    // Pre-trigger odom timestamp even with vx=0 must NOT confirm stop
    const auto pre_odom = makeOdometry(rclcpp::Time(99, 950000000, RCL_ROS_TIME), -0.15, 0.0, 0.0, 0.0);
    auto r = nav2_custom_plugins::evaluateStopConfirmation(pre_odom, rclcpp::Time(100, 50000000, RCL_ROS_TIME), {-0.15, 0.0, 0.0}, cfg, st);
    EXPECT_FALSE(r.stopped);
    EXPECT_FALSE(r.failed);

    // Odom with nonzero velocity must NOT confirm stop
    const rclcpp::Time t_mov(100, 100000000, RCL_ROS_TIME);
    r = nav2_custom_plugins::evaluateStopConfirmation(
      makeOdometry(t_mov, -0.16, 0.0, -0.06, 0.0), t_mov, {-0.16, 0.0, 0.0}, cfg, st);
    EXPECT_FALSE(r.stopped);
    EXPECT_FALSE(r.failed);

    // Odom first zero-vel sample sets anchor -> not stopped yet
    const rclcpp::Time t_stop1(100, 150000000, RCL_ROS_TIME);
    r = nav2_custom_plugins::evaluateStopConfirmation(
      makeOdometry(t_stop1, -0.16, 0.0, 0.0, 0.0), t_stop1, {-0.16, 0.0, 0.0}, cfg, st);
    EXPECT_FALSE(r.stopped);
    EXPECT_FALSE(r.failed);

    // Same zero-vel sample + drift in pose -> resets dwell, NOT stopped
    const rclcpp::Time t_drift(100, 200000000, RCL_ROS_TIME);
    r = nav2_custom_plugins::evaluateStopConfirmation(
      makeOdometry(t_drift, -0.17, 0.0, 0.0, 0.0), t_drift, {-0.17, 0.0, 0.0}, cfg, st);
    EXPECT_FALSE(r.stopped);
    EXPECT_FALSE(r.failed);

    // Next distinct settled sample within dwell < 0.08 s -> still waiting
    const rclcpp::Time t_settle1(100, 250000000, RCL_ROS_TIME);
    r = nav2_custom_plugins::evaluateStopConfirmation(
      makeOdometry(t_settle1, -0.17, 0.0, 0.0, 0.0), t_settle1, {-0.17, 0.0, 0.0}, cfg, st);
    EXPECT_FALSE(r.stopped);
    EXPECT_FALSE(r.failed);

    // Third settled sample with dwell >= 0.08 s -> CONFIRMED STOPPED!
    const rclcpp::Time t_settle2(100, 300000000, RCL_ROS_TIME);
    r = nav2_custom_plugins::evaluateStopConfirmation(
      makeOdometry(t_settle2, -0.17, 0.0, 0.0, 0.0), t_settle2, {-0.17, 0.0, 0.0}, cfg, st);
    EXPECT_TRUE(r.stopped);
    EXPECT_FALSE(r.failed);

    // Stale odom must fail cleanly
    nav2_custom_plugins::StopConfirmState st_stale;
    st_stale.active = true;
    st_stale.trigger_time = t0;
    const auto r_stale = nav2_custom_plugins::evaluateStopConfirmation(
      makeOdometry(t0, -0.15, 0.0, 0.0, 0.0), rclcpp::Time(101, 0, RCL_ROS_TIME), {-0.15, 0.0, 0.0}, cfg, st_stale);
    EXPECT_FALSE(r_stale.stopped);
    EXPECT_TRUE(r_stale.failed);
    EXPECT_EQ(r_stale.failure_code, nav2_custom_plugins::BackupFailureCode::SENSOR_STALE);
  }
}

class CustomBackUpActionFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    server_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_behavior_server");
    server_node_->declare_parameter("cycle_frequency", 20.0);
    server_node_->declare_parameter("global_frame", std::string("map"));
    server_node_->declare_parameter("local_frame", std::string("odom"));
    server_node_->declare_parameter("robot_base_frame", std::string("base_footprint"));
    server_node_->declare_parameter("transform_tolerance", 1.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(server_node_->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    client_node_ = std::make_shared<rclcpp::Node>("test_action_client");

    cmd_vel_sub_ = client_node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 20,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_history_.push_back(*msg);
      });

    costmap_pub_ = client_node_->create_publisher<nav2_msgs::msg::Costmap>(
      "local_costmap/costmap_raw",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    footprint_pub_ = client_node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
      "local_costmap/published_footprint", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    scan_pub_ = client_node_->create_publisher<sensor_msgs::msg::LaserScan>(
      "/rear_scan_obstacle", rclcpp::SensorDataQoS());
    health_pub_ = client_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/rear_scan_obstacle/health", rclcpp::QoS(rclcpp::KeepLast(10)));
    odom_pub_ = client_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(rclcpp::KeepLast(20)));

    behavior_ = std::make_shared<nav2_custom_plugins::CustomBackUp>();
    behavior_->configure(server_node_, "custom_backup", tf_buffer_, nullptr);
    server_node_->activate();
    behavior_->activate();

    action_client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(
      client_node_, "custom_backup");

    executor_.add_node(server_node_->get_node_base_interface());
    executor_.add_node(client_node_);
    spin_thread_ = std::thread([this]() {executor_.spin();});

    ASSERT_TRUE(action_client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    behavior_->deactivate();
    server_node_->deactivate();
    behavior_->cleanup();
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    behavior_.reset();
    action_client_.reset();
    client_node_.reset();
    server_node_.reset();
  }

  void publishStaticRearTf(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped rear_tf;
    rear_tf.header.stamp = stamp;
    rear_tf.header.frame_id = "base_footprint";
    rear_tf.child_frame_id = "rear_scan_frame";
    rear_tf.transform.translation.x = -0.25;
    rear_tf.transform.translation.y = 0.0;
    rear_tf.transform.translation.z = 0.0;
    rear_tf.transform.rotation.z = 1.0;  // pi yaw
    rear_tf.transform.rotation.w = 0.0;
    tf_buffer_->setTransform(rear_tf, "test_authority", false);
  }

  void publishPoseAndEnv(
    double x, double y,
    bool front_overlap = false,
    double rear_obstacle_dist = std::numeric_limits<double>::infinity(),
    double odom_vx = 0.0,
    double odom_wz = 0.0)
  {
    publishEnvCustom(
      x, y, front_overlap, rear_obstacle_dist, odom_vx, odom_wz,
      /*publish_rear_cam=*/true, /*healthy_rear_cam=*/true,
      /*publish_odom_tf=*/true, /*publish_rear_tf=*/true);
  }

  void publishEnvCustom(
    double x, double y,
    bool front_overlap,
    double rear_obstacle_dist,
    double odom_vx,
    double odom_wz,
    bool publish_rear_cam,
    bool healthy_rear_cam,
    bool publish_odom_tf,
    bool publish_rear_tf)
  {
    const rclcpp::Time now = server_node_->now();
    if (publish_rear_tf) {
      publishStaticRearTf(now);
    }

    if (publish_odom_tf) {
      geometry_msgs::msg::TransformStamped odom_tf;
      odom_tf.header.stamp = now;
      odom_tf.header.frame_id = "odom";
      odom_tf.child_frame_id = "base_footprint";
      odom_tf.transform.translation.x = x;
      odom_tf.transform.translation.y = y;
      odom_tf.transform.rotation.w = 1.0;
      tf_buffer_->setTransform(odom_tf, "test_authority", false);
    }

    auto cm = makeCostmap(now, -1.5 + x, -1.5 + y);
    if (front_overlap) {
      setCostAtWorld(*cm, 0.25, 0.05, 254);
    }
    costmap_pub_->publish(*cm);

    geometry_msgs::msg::PolygonStamped fp;
    fp.header.stamp = now;
    fp.header.frame_id = "base_footprint";
    for (const auto & pt : makeRobotFootprint()) {
      geometry_msgs::msg::Point32 p32;
      p32.x = static_cast<float>(pt.x);
      p32.y = static_cast<float>(pt.y);
      fp.polygon.points.push_back(p32);
    }
    footprint_pub_->publish(fp);

    odom_pub_->publish(*makeOdometry(now, x, y, odom_vx, odom_wz));

    if (publish_rear_cam) {
      auto scan = makeRearScan(now, 360, rear_obstacle_dist, 0.0, 0.05);
      diagnostic_msgs::msg::DiagnosticArray::SharedPtr health;
      if (healthy_rear_cam) {
        health = makeRearHealth(now);
      } else {
        health = makeRearHealth(
          now, false, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true,
          diagnostic_msgs::msg::DiagnosticStatus::ERROR,
          /*cam_ok=*/false, /*tf_ok=*/true, /*ground_ok=*/true);
      }
      health_pub_->publish(*health);
      scan_pub_->publish(*scan);
    }
  }

  std::vector<geometry_msgs::msg::Twist> getCmdHistory()
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    return cmd_history_;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_custom_plugins::CustomBackUp> behavior_;
  rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr action_client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spin_thread_;

  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr costmap_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  std::mutex cmd_mutex_;
  std::vector<geometry_msgs::msg::Twist> cmd_history_;
};

TEST_F(CustomBackUpActionFixture, SucceedsWithPreExistingFrontOverlapAndStopsCleanly)
{
  publishPoseAndEnv(0.0, 0.0, /*front_overlap=*/true);
  std::this_thread::sleep_for(80ms);
  publishPoseAndEnv(0.0, 0.0, /*front_overlap=*/true);
  std::this_thread::sleep_for(50ms);

  nav2_msgs::action::BackUp::Goal goal;
  goal.target.x = 0.30;
  goal.speed = 0.10f;
  goal.time_allowance = rclcpp::Duration::from_seconds(6.0);

  auto goal_future = action_client_->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(3s), std::future_status::ready);
  auto gh = goal_future.get();
  ASSERT_NE(gh, nullptr);

  auto result_future = action_client_->async_get_result(gh);

  // Advance simulated robot backward in 4 steps to 0.31 m while keeping rear clear
  for (double d = 0.08; d <= 0.33; d += 0.08) {
    std::this_thread::sleep_for(70ms);
    publishPoseAndEnv(-d, 0.0, /*front_overlap=*/true);
  }

  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);

  std::this_thread::sleep_for(50ms);
  const auto cmds = getCmdHistory();
  ASSERT_GE(cmds.size(), 2u);
  EXPECT_LT(cmds.front().linear.x, -0.05);
  EXPECT_NEAR(cmds.back().linear.x, 0.0, 1e-6);
}

TEST_F(CustomBackUpActionFixture, MidBackupRearHazardTriggersImmediateSafetyStop)
{
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(80ms);
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(50ms);

  nav2_msgs::action::BackUp::Goal goal;
  goal.target.x = 0.30;
  goal.speed = 0.10f;
  goal.time_allowance = rclcpp::Duration::from_seconds(6.0);

  auto goal_future = action_client_->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(3s), std::future_status::ready);
  auto gh = goal_future.get();
  ASSERT_NE(gh, nullptr);

  auto result_future = action_client_->async_get_result(gh);
  std::this_thread::sleep_for(90ms);
  // Inject rear obstacle at 0.40 m (< 0.50 m) at 0.02 m (< 0.08 m) -> ABORTED!
  publishPoseAndEnv(-0.02, 0.0, false, /*rear_obstacle_dist=*/0.40);

  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  const auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);

  std::this_thread::sleep_for(50ms);
  const auto cmds = getCmdHistory();
  ASSERT_FALSE(cmds.empty());
  EXPECT_NEAR(cmds.back().linear.x, 0.0, 1e-6);
}

TEST_F(CustomBackUpActionFixture, InvalidGoalAndTimeoutAndCancelStopRobot)
{
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(80ms);

  // 1) Invalid goal
  {
    nav2_msgs::action::BackUp::Goal bad_goal;
    bad_goal.target.x = 0.30;
    bad_goal.target.y = 0.20;
    bad_goal.speed = 0.10f;
    bad_goal.time_allowance = rclcpp::Duration::from_seconds(2.0);
    auto gh_fut = action_client_->async_send_goal(bad_goal);
    ASSERT_EQ(gh_fut.wait_for(2s), std::future_status::ready);
    auto gh = gh_fut.get();
    ASSERT_NE(gh, nullptr);
    auto res_fut = action_client_->async_get_result(gh);
    ASSERT_EQ(res_fut.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::ABORTED);
  }

  // 2) Timeout
  {
    publishPoseAndEnv(0.0, 0.0, false);
    nav2_msgs::action::BackUp::Goal goal;
    goal.target.x = 0.30;
    goal.speed = 0.10f;
    goal.time_allowance = rclcpp::Duration::from_seconds(0.20);
    auto gh_fut = action_client_->async_send_goal(goal);
    ASSERT_EQ(gh_fut.wait_for(2s), std::future_status::ready);
    auto gh = gh_fut.get();
    ASSERT_NE(gh, nullptr);
    auto res_fut = action_client_->async_get_result(gh);
    ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::ABORTED);
    std::this_thread::sleep_for(50ms);
    EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);
  }

  // 3) Cancel
  {
    publishPoseAndEnv(0.0, 0.0, false);
    nav2_msgs::action::BackUp::Goal goal;
    goal.target.x = 0.30;
    goal.speed = 0.10f;
    goal.time_allowance = rclcpp::Duration::from_seconds(6.0);
    auto gh_fut = action_client_->async_send_goal(goal);
    ASSERT_EQ(gh_fut.wait_for(2s), std::future_status::ready);
    auto gh = gh_fut.get();
    ASSERT_NE(gh, nullptr);
    std::this_thread::sleep_for(80ms);
    auto cancel_fut = action_client_->async_cancel_goal(gh);
    ASSERT_EQ(cancel_fut.wait_for(2s), std::future_status::ready);
    auto res_fut = action_client_->async_get_result(gh);
    ASSERT_EQ(res_fut.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::CANCELED);
    std::this_thread::sleep_for(50ms);
    EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);
  }
}

TEST_F(CustomBackUpActionFixture, PartialSuccessBelowZeroPointZeroEightFails)
{
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(80ms);
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(50ms);

  nav2_msgs::action::BackUp::Goal goal;
  goal.target.x = 0.30;
  goal.speed = 0.10f;
  goal.time_allowance = rclcpp::Duration::from_seconds(6.0);

  auto gh_fut = action_client_->async_send_goal(goal);
  ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
  auto gh = gh_fut.get();
  ASSERT_NE(gh, nullptr);
  auto res_fut = action_client_->async_get_result(gh);

  std::this_thread::sleep_for(70ms);
  // Back up 0.05 m (< 0.08 m), then rear obstacle appears at 0.40 m
  publishPoseAndEnv(-0.05, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
  std::this_thread::sleep_for(70ms);
  publishPoseAndEnv(-0.05, 0.0, false, /*rear_obstacle_dist=*/0.40, 0.0, 0.0);

  ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::ABORTED);
  std::this_thread::sleep_for(50ms);
  ASSERT_FALSE(getCmdHistory().empty());
  EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);
}

TEST_F(CustomBackUpActionFixture, PartialSuccessAtExactBoundaryZeroPointZeroEightSucceeds)
{
  ScopedLogCapture logs;
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(80ms);
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(50ms);

  nav2_msgs::action::BackUp::Goal goal;
  goal.target.x = 0.30;
  goal.speed = 0.10f;
  goal.time_allowance = rclcpp::Duration::from_seconds(6.0);

  auto gh_fut = action_client_->async_send_goal(goal);
  ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
  auto gh = gh_fut.get();
  ASSERT_NE(gh, nullptr);
  auto res_fut = action_client_->async_get_result(gh);

  std::this_thread::sleep_for(70ms);
  publishPoseAndEnv(-0.08, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
  std::this_thread::sleep_for(70ms);

  // Rear obstacle at exact 0.08 m traveled; stop odom confirms stationary across multiple cycles
  for (int i = 0; i < 6; ++i) {
    publishPoseAndEnv(-0.08, 0.0, false, /*rear_obstacle_dist=*/0.40, 0.0, 0.0);
    if (res_fut.wait_for(70ms) == std::future_status::ready) {
      break;
    }
  }

  ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  std::this_thread::sleep_for(50ms);
  ASSERT_FALSE(getCmdHistory().empty());
  EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);

  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] PARTIAL_SUCCESS reason=REAR_OBSTACLE", RCUTILS_LOG_SEVERITY_INFO), 1);
  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] FAILED"), 0);
  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] SUCCESS"), 0);
  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] SAFETY_STOP"), 0);
}

TEST_F(CustomBackUpActionFixture, PartialSuccessAtZeroPointOneFiveWaitsUntilOdomConfirmsStopped)
{
  ScopedLogCapture logs;
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(80ms);
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(50ms);

  nav2_msgs::action::BackUp::Goal goal;
  goal.target.x = 0.30;
  goal.speed = 0.10f;
  goal.time_allowance = rclcpp::Duration::from_seconds(6.0);

  auto gh_fut = action_client_->async_send_goal(goal);
  ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
  auto gh = gh_fut.get();
  ASSERT_NE(gh, nullptr);
  auto res_fut = action_client_->async_get_result(gh);

  std::this_thread::sleep_for(70ms);
  publishPoseAndEnv(-0.08, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
  std::this_thread::sleep_for(70ms);
  publishPoseAndEnv(-0.15, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
  std::this_thread::sleep_for(70ms);

  // Rear obstacle appears at 0.15 m, while odom STILL shows robot coasting at -0.06 m/s
  for (int i = 0; i < 4; ++i) {
    const double coast_x = -0.15 - 0.005 * static_cast<double>(i);
    publishPoseAndEnv(coast_x, 0.0, false, /*rear_obstacle_dist=*/0.40, /*odom_vx=*/-0.06, 0.0);
    std::this_thread::sleep_for(65ms);
  }

  // Must NOT return SUCCEEDED while odom still reports motion!
  EXPECT_EQ(res_fut.wait_for(20ms), std::future_status::timeout);
  ASSERT_FALSE(getCmdHistory().empty());
  EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);

  // Now odom confirms robot is completely stationary -> should transition to SUCCEEDED
  for (int i = 0; i < 6; ++i) {
    publishPoseAndEnv(-0.165, 0.0, false, /*rear_obstacle_dist=*/0.40, /*odom_vx=*/0.0, 0.0);
    if (res_fut.wait_for(70ms) == std::future_status::ready) {
      break;
    }
  }

  ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  std::this_thread::sleep_for(50ms);
  EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);

  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] PARTIAL_SUCCESS reason=REAR_OBSTACLE dist=0.165m", RCUTILS_LOG_SEVERITY_INFO), 1);
  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] FAILED"), 0);
  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] SUCCESS"), 0);
  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] SAFETY_STOP"), 0);
}

TEST_F(CustomBackUpActionFixture, ClearRearDoesNotStopAtZeroPointZeroEightAndCompletesZeroPointThreeZero)
{
  ScopedLogCapture logs;
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(80ms);
  publishPoseAndEnv(0.0, 0.0, false);
  std::this_thread::sleep_for(50ms);

  nav2_msgs::action::BackUp::Goal goal;
  goal.target.x = 0.30;
  goal.speed = 0.10f;
  goal.time_allowance = rclcpp::Duration::from_seconds(6.0);

  auto gh_fut = action_client_->async_send_goal(goal);
  ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
  auto gh = gh_fut.get();
  ASSERT_NE(gh, nullptr);
  auto res_fut = action_client_->async_get_result(gh);

  for (double d : {0.08, 0.14, 0.22}) {
    std::this_thread::sleep_for(70ms);
    publishPoseAndEnv(-d, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
  }
  std::this_thread::sleep_for(90ms);
  // Still RUNNING at 0.22 m (>= 0.08 m) when rear is clear!
  EXPECT_EQ(res_fut.wait_for(20ms), std::future_status::timeout);
  ASSERT_FALSE(getCmdHistory().empty());
  EXPECT_LT(getCmdHistory().back().linear.x, -0.05);

  std::this_thread::sleep_for(60ms);
  publishPoseAndEnv(-0.31, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);

  ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  std::this_thread::sleep_for(50ms);
  EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);
  EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] SUCCESS", RCUTILS_LOG_SEVERITY_INFO), 1);
  EXPECT_EQ(logs.countContaining("PARTIAL_SUCCESS"), 0);
}

TEST_F(CustomBackUpActionFixture, LostCameraAtZeroPointTwoZeroAndDuringPartialStopMustFail)
{
  // 1) At 0.20 m, camera reports unhealthy even with spurious obstacle ray -> ABORTED (no Partial Success!)
  {
    ScopedLogCapture logs;
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(80ms);
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(50ms);

    nav2_msgs::action::BackUp::Goal goal;
    goal.target.x = 0.30;
    goal.speed = 0.10f;
    goal.time_allowance = rclcpp::Duration::from_seconds(6.0);
    auto gh_fut = action_client_->async_send_goal(goal);
    ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
    auto gh = gh_fut.get();
    ASSERT_NE(gh, nullptr);
    auto res_fut = action_client_->async_get_result(gh);

    std::this_thread::sleep_for(70ms);
    publishPoseAndEnv(-0.20, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
    std::this_thread::sleep_for(70ms);
    publishEnvCustom(
      -0.20, 0.0, false, /*rear_obstacle_dist=*/0.40, 0.0, 0.0,
      /*publish_rear_cam=*/true, /*healthy_rear_cam=*/false,
      /*publish_odom_tf=*/true, /*publish_rear_tf=*/true);

    ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::ABORTED);
    std::this_thread::sleep_for(50ms);
    EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);
    EXPECT_EQ(logs.countContaining("PARTIAL_SUCCESS"), 0);
    EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] FAILED reason=SENSOR_INVALID", RCUTILS_LOG_SEVERITY_ERROR), 1);
  }

  // 2) At 0.15 m, rear obstacle triggers deceleration while moving, then camera drops -> ABORTED!
  {
    ScopedLogCapture logs;
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(80ms);
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(50ms);

    nav2_msgs::action::BackUp::Goal goal;
    goal.target.x = 0.30;
    goal.speed = 0.10f;
    goal.time_allowance = rclcpp::Duration::from_seconds(6.0);
    auto gh_fut = action_client_->async_send_goal(goal);
    ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
    auto gh = gh_fut.get();
    ASSERT_NE(gh, nullptr);
    auto res_fut = action_client_->async_get_result(gh);

    std::this_thread::sleep_for(70ms);
    publishPoseAndEnv(-0.15, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
    std::this_thread::sleep_for(70ms);
    // Rear obstacle triggers pending_partial_stop_ while odom still moving
    publishPoseAndEnv(-0.15, 0.0, false, /*rear_obstacle_dist=*/0.40, -0.06, 0.0);
    std::this_thread::sleep_for(70ms);
    // Camera dies mid-stop!
    publishEnvCustom(
      -0.15, 0.0, false, 0.40, 0.0, 0.0,
      /*publish_rear_cam=*/true, /*healthy_rear_cam=*/false,
      /*publish_odom_tf=*/true, /*publish_rear_tf=*/true);

    ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(logs.countContaining("PARTIAL_SUCCESS"), 0);
  }
}

TEST_F(CustomBackUpActionFixture, LostTfAtZeroPointTwoZeroFailsAndDoesNotPartialSucceed)
{
  // 1) Drop rear camera TF cleanly at 0.20 m (>= 0.08 m) while publishing obstacle scan -> must ABORT with TF_ERROR
  {
    ScopedLogCapture logs;
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(80ms);
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(50ms);

    nav2_msgs::action::BackUp::Goal goal;
    goal.target.x = 0.30;
    goal.speed = 0.10f;
    goal.time_allowance = rclcpp::Duration::from_seconds(6.0);
    auto gh_fut = action_client_->async_send_goal(goal);
    ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
    auto gh = gh_fut.get();
    ASSERT_NE(gh, nullptr);
    auto res_fut = action_client_->async_get_result(gh);

    std::this_thread::sleep_for(70ms);
    publishPoseAndEnv(-0.20, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
    std::this_thread::sleep_for(70ms);

    tf_buffer_->clear();
    publishEnvCustom(
      -0.20, 0.0, false, /*rear_obstacle_dist=*/0.40, 0.0, 0.0,
      /*publish_rear_cam=*/true, /*healthy_rear_cam=*/true,
      /*publish_odom_tf=*/true, /*publish_rear_tf=*/false);

    ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::ABORTED);
    std::this_thread::sleep_for(50ms);
    EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);
    EXPECT_EQ(logs.countContaining("PARTIAL_SUCCESS"), 0);
    EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] FAILED reason=TF_ERROR", RCUTILS_LOG_SEVERITY_ERROR), 1);
  }

  // 2) Drop odom -> base_footprint TF completely at 0.20 m -> must ABORT with TF_ERROR
  {
    ScopedLogCapture logs;
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(80ms);
    publishPoseAndEnv(0.0, 0.0, false);
    std::this_thread::sleep_for(50ms);

    nav2_msgs::action::BackUp::Goal goal;
    goal.target.x = 0.30;
    goal.speed = 0.10f;
    goal.time_allowance = rclcpp::Duration::from_seconds(6.0);
    auto gh_fut = action_client_->async_send_goal(goal);
    ASSERT_EQ(gh_fut.wait_for(3s), std::future_status::ready);
    auto gh = gh_fut.get();
    ASSERT_NE(gh, nullptr);
    auto res_fut = action_client_->async_get_result(gh);

    std::this_thread::sleep_for(70ms);
    publishPoseAndEnv(-0.20, 0.0, false, std::numeric_limits<double>::infinity(), -0.10, 0.0);
    std::this_thread::sleep_for(70ms);

    tf_buffer_->clear();
    ASSERT_EQ(res_fut.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(res_fut.get().code, rclcpp_action::ResultCode::ABORTED);
    std::this_thread::sleep_for(50ms);
    EXPECT_NEAR(getCmdHistory().back().linear.x, 0.0, 1e-6);
    EXPECT_EQ(logs.countContaining("PARTIAL_SUCCESS"), 0);
    EXPECT_EQ(logs.countContaining("[CUSTOM_BACKUP] FAILED reason=TF_ERROR", RCUTILS_LOG_SEVERITY_ERROR), 1);
  }
}

TEST(CustomBackUpPluginlibTest, CheckRearSafetyBtNodeStrictAllInfAndPluginRegistration)
{
  // Verify pluginlib registration and dynamic instantiation for nav2_core::Behavior
  {
    pluginlib::ClassLoader<nav2_core::Behavior> behavior_loader("nav2_core", "nav2_core::Behavior");
    const auto declared = behavior_loader.getDeclaredClasses();
    EXPECT_NE(
      std::find(declared.begin(), declared.end(), "nav2_custom_plugins/CustomBackUp"),
      declared.end());
    auto plugin_instance = behavior_loader.createUniqueInstance("nav2_custom_plugins/CustomBackUp");
    ASSERT_NE(plugin_instance, nullptr);
  }

  // Verify CheckRearSafety BT plugin handles all-inf strictly
  auto bt_node = std::make_shared<rclcpp::Node>("test_bt_check_rear");
  auto tf_buf = std::make_shared<tf2_ros::Buffer>(bt_node->get_clock());
  geometry_msgs::msg::TransformStamped rear_tf;
  rear_tf.header.stamp = bt_node->now();
  rear_tf.header.frame_id = "base_footprint";
  rear_tf.child_frame_id = "rear_scan_frame";
  rear_tf.transform.translation.x = -0.25;
  rear_tf.transform.rotation.z = 1.0;
  rear_tf.transform.rotation.w = 0.0;
  tf_buf->setTransform(rear_tf, "test_authority", true);

  auto scan_pub = bt_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/rear_scan_obstacle_bt", rclcpp::SensorDataQoS());
  auto health_pub = bt_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/rear_scan_obstacle_bt/health", rclcpp::QoS(rclcpp::KeepLast(10)));

  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin("libnav2_check_rear_safety_plugin.so");

  auto bb = BT::Blackboard::create();
  bb->set("node", bt_node);
  bb->set("tf_buffer", tf_buf);

  const std::string xml = R"(
    <root main_tree_to_execute="Main">
      <BehaviorTree ID="Main">
        <CheckRearSafety topic="/rear_scan_obstacle_bt" min_distance="0.50" timeout="0.50"/>
      </BehaviorTree>
    </root>
  )";
  auto tree = factory.createTreeFromText(xml, bb);

  // Case 1: all-inf without health -> FAILURE
  scan_pub->publish(*makeRearScan(bt_node->now()));
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(25ms);
  }
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  // Case 2: all-inf + valid_ratio=0.95 but corridor_observed=false -> FAILURE
  const auto t2 = bt_node->now();
  health_pub->publish(*makeRearHealth(t2, false, 0.95));
  scan_pub->publish(*makeRearScan(t2));
  for (int i = 0; i < 5; ++i) {
    std::this_thread::sleep_for(25ms);
  }
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

  // Case 3: all-inf + matched full observability health -> SUCCESS
  const auto t3 = bt_node->now();
  health_pub->publish(*makeRearHealth(t3, true, 0.95));
  scan_pub->publish(*makeRearScan(t3));
  for (int i = 0; i < 6; ++i) {
    std::this_thread::sleep_for(25ms);
  }
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

TEST(CustomBackUpPluginlibTest, NavigateToPoseCustomRecoveryXmlBackupSubtreeBehavior)
{
  // Load actual navigate_to_pose_custom_recovery.xml from mo_hinh/config
  const auto xml_path =
    std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
    "mo_hinh" / "config" / "navigate_to_pose_custom_recovery.xml";
  ASSERT_TRUE(std::filesystem::exists(xml_path)) << "Missing BT XML: " << xml_path;

  std::ifstream ifs(xml_path);
  ASSERT_TRUE(ifs.is_open());
  std::ostringstream buffer;
  buffer << ifs.rdbuf();
  const std::string full_xml = buffer.str();

  EXPECT_EQ(full_xml.find("SafeBackUp"), std::string::npos);
  EXPECT_NE(full_xml.find("<BackUp server_name=\"custom_backup\" backup_dist=\"0.30\" backup_speed=\"0.10\" time_allowance=\"6.0\"/>"), std::string::npos);
  EXPECT_NE(full_xml.find("<Fallback name=\"ConditionalClearBeforeReplanning\">"), std::string::npos);
  EXPECT_NE(full_xml.find("<CheckRearSafety topic=\"/rear_scan_obstacle\" min_distance=\"0.50\" timeout=\"0.50\"/>"), std::string::npos);

  const std::string start_marker = "<RecoveryNode number_of_retries=\"3\" name=\"BackupRecovery\">";
  const std::string end_marker = "</RecoveryNode>";
  const size_t start_pos = full_xml.find(start_marker);
  ASSERT_NE(start_pos, std::string::npos);
  const size_t first_close = full_xml.find(end_marker, start_pos);
  ASSERT_NE(first_close, std::string::npos);
  const std::string backup_recovery_subtree =
    full_xml.substr(start_pos, (first_close + end_marker.size()) - start_pos);

  const std::string wrapped_xml = std::string(
    "<root main_tree_to_execute=\"BackupOnlyTree\">\n"
    "  <BehaviorTree ID=\"BackupOnlyTree\">\n") +
    backup_recovery_subtree +
    "\n  </BehaviorTree>\n</root>\n";

  auto bt_node = std::make_shared<rclcpp::Node>("test_bt_backup_recovery_xml");
  auto tf_buf = std::make_shared<tf2_ros::Buffer>(bt_node->get_clock());
  geometry_msgs::msg::TransformStamped rear_tf;
  rear_tf.header.stamp = bt_node->now();
  rear_tf.header.frame_id = "base_footprint";
  rear_tf.child_frame_id = "rear_scan_frame";
  rear_tf.transform.translation.x = -0.25;
  rear_tf.transform.rotation.z = 1.0;
  rear_tf.transform.rotation.w = 0.0;
  tf_buf->setTransform(rear_tf, "test_authority", true);

  auto scan_pub = bt_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/rear_scan_obstacle", rclcpp::SensorDataQoS());
  auto health_pub = bt_node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/rear_scan_obstacle/health", rclcpp::QoS(rclcpp::KeepLast(10)));

  struct SimCounters
  {
    int backup_ticks{0};
    int backup_halted{0};
    int clear_calls{0};
    int wait_calls{0};
    bool backup_should_succeed{true};
    int backup_ticks_to_complete{1};
    double last_backup_dist{0.0};
    double last_backup_speed{0.0};
    std::string last_server_name;
    std::vector<std::string> cleared_services;
  } sim;

  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin("libnav2_check_rear_safety_plugin.so");
  factory.registerFromPlugin("libnav2_recovery_node_bt_node.so");

  factory.registerSimpleAction(
    "OperatorAlert",
    [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
    {BT::InputPort<std::string>("code"), BT::InputPort<std::string>("message")});

  factory.registerSimpleAction(
    "Speak",
    [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;},
    {BT::InputPort<std::string>("message")});

  class StatefulBackUpStub : public BT::StatefulActionNode
  {
  public:
    StatefulBackUpStub(
      const std::string & name,
      const BT::NodeConfiguration & config,
      SimCounters * counters)
    : BT::StatefulActionNode(name, config), counters_(counters), tick_in_run_(0) {}

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("server_name"),
        BT::InputPort<double>("backup_dist"),
        BT::InputPort<double>("backup_speed"),
        BT::InputPort<double>("time_allowance")
      };
    }

    BT::NodeStatus onStart() override
    {
      tick_in_run_ = 1;
      getInput("server_name", counters_->last_server_name);
      getInput("backup_dist", counters_->last_backup_dist);
      getInput("backup_speed", counters_->last_backup_speed);
      ++counters_->backup_ticks;
      if (tick_in_run_ < counters_->backup_ticks_to_complete) {
        return BT::NodeStatus::RUNNING;
      }
      return counters_->backup_should_succeed ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onRunning() override
    {
      ++tick_in_run_;
      ++counters_->backup_ticks;
      if (tick_in_run_ < counters_->backup_ticks_to_complete) {
        return BT::NodeStatus::RUNNING;
      }
      return counters_->backup_should_succeed ?
             BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    void onHalted() override
    {
      ++counters_->backup_halted;
    }

  private:
    SimCounters * counters_;
    int tick_in_run_;
  };

  factory.registerBuilder<StatefulBackUpStub>(
    "BackUp",
    [&sim](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<StatefulBackUpStub>(name, config, &sim);
    });

  factory.registerSimpleAction(
    "ClearEntireCostmap",
    [&sim](BT::TreeNode & self) {
      ++sim.clear_calls;
      std::string svc;
      self.getInput("service_name", svc);
      sim.cleared_services.push_back(svc);
      return BT::NodeStatus::SUCCESS;
    },
    {BT::InputPort<std::string>("service_name")});

  factory.registerSimpleAction(
    "Wait",
    [&sim](BT::TreeNode &) {
      ++sim.wait_calls;
      return BT::NodeStatus::SUCCESS;
    },
    {BT::InputPort<double>("wait_duration")});

  auto bb = BT::Blackboard::create();
  bb->set("node", bt_node);
  bb->set("tf_buffer", tf_buf);
  bb->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(500));

  auto tree = factory.createTreeFromText(wrapped_xml, bb);

  auto publishRear = [&](double obstacle_dist) {
    for (int i = 0; i < 4; ++i) {
      const auto ts = bt_node->now();
      health_pub->publish(*makeRearHealth(ts, true, 0.95));
      scan_pub->publish(*makeRearScan(ts, 360, obstacle_dist, 0.0, 0.05));
      std::this_thread::sleep_for(25ms);
    }
  };

  // Case A: Clear 0.30 m backup -> clears both costmaps and succeeds (transitions to Replanning)
  sim = SimCounters();
  sim.backup_should_succeed = true;
  sim.backup_ticks_to_complete = 1;
  publishRear(std::numeric_limits<double>::infinity());
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(sim.last_server_name, "custom_backup");
  EXPECT_DOUBLE_EQ(sim.last_backup_dist, 0.30);
  EXPECT_DOUBLE_EQ(sim.last_backup_speed, 0.10);
  EXPECT_EQ(sim.clear_calls, 2);
  EXPECT_EQ(sim.wait_calls, 0);
  EXPECT_EQ(sim.backup_halted, 0);

  // Case B: Mid-backup rear obstacle -> BackUp is NOT canceled/halted by BT, finishes as Partial Success,
  // ConditionalClearBeforeReplanning SKIPS clearing costmaps (clear_calls == 0), and BackupRecovery returns SUCCESS!
  sim = SimCounters();
  sim.backup_should_succeed = true;
  sim.backup_ticks_to_complete = 2;
  publishRear(std::numeric_limits<double>::infinity());
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(sim.backup_ticks, 1);

  // Obstacle appears behind robot while BackUp is RUNNING (partial stop in progress)
  publishRear(0.40);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(sim.backup_ticks, 2);
  EXPECT_EQ(sim.backup_halted, 0);
  EXPECT_EQ(sim.clear_calls, 0);
  EXPECT_EQ(sim.wait_calls, 0);

  // Case C: Failed backup (< 0.08 m or sensor/TF error) -> skips clear, triggers BackUpFailed retries (Wait 2s x 3)
  sim = SimCounters();
  sim.backup_should_succeed = false;
  sim.backup_ticks_to_complete = 1;
  publishRear(std::numeric_limits<double>::infinity());
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(sim.clear_calls, 0);
  EXPECT_EQ(sim.wait_calls, 3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
