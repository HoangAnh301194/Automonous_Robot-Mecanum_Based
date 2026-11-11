#include "nav2_custom_plugins/custom_backup.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer_interface.h"

namespace nav2_custom_plugins
{

namespace
{

double quaternionToYaw(const geometry_msgs::msg::Quaternion & q_msg)
{
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

CustomBackUp::CustomBackUp()
: nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>(),
  feedback_(std::make_shared<BackUpAction::Feedback>())
{
}

void CustomBackUp::onConfigure()
{
  auto node = this->node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in CustomBackUp::onConfigure"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, "local_frame", rclcpp::ParameterValue(std::string("odom")));
  nav2_util::declare_parameter_if_not_declared(
    node, "local_costmap_topic", rclcpp::ParameterValue(std::string("local_costmap/costmap_raw")));
  nav2_util::declare_parameter_if_not_declared(
    node, "local_footprint_topic",
    rclcpp::ParameterValue(std::string("local_costmap/published_footprint")));

  node->get_parameter("local_frame", local_frame_);
  node->get_parameter("local_costmap_topic", local_costmap_topic_);
  node->get_parameter("local_footprint_topic", local_footprint_topic_);

  const std::string p = this->behavior_name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, p + "rear_scan_topic", rclcpp::ParameterValue(std::string("/rear_scan_obstacle")));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "rear_health_topic", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "min_distance", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "rear_scan_timeout", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "rear_health_timeout", rclcpp::ParameterValue(0.50));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "costmap_timeout", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "footprint_timeout", rclcpp::ParameterValue(2.5));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "require_rear_health", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "acceleration_limit", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "deceleration_limit", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "minimum_speed", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "max_backup_distance", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "max_backup_speed", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "rear_stop_margin", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "rear_corner_lateral_margin", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "front_overlap_min_x", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "max_front_overlap_depth", rclcpp::ParameterValue(0.23));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "max_front_overlap_ratio", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "whitelist_match_radius", rclcpp::ParameterValue(0.085));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "min_corridor_observed_ratio", rclcpp::ParameterValue(0.60));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "min_near_zone_observed_ratio", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "min_center_observed_ratio", rclcpp::ParameterValue(0.60));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "min_sector_coverage_ratio", rclcpp::ParameterValue(0.70));
  nav2_util::declare_parameter_if_not_declared(
    node, p + "min_required_x_max", rclcpp::ParameterValue(0.55));

  node->get_parameter(p + "rear_scan_topic", rear_scan_topic_);
  node->get_parameter(p + "rear_health_topic", rear_health_topic_);
  if (rear_health_topic_.empty()) {
    rear_health_topic_ = rear_scan_topic_ + "/health";
  }

  node->get_parameter(p + "min_distance", eval_config_.rear.min_distance);
  node->get_parameter(p + "rear_scan_timeout", eval_config_.rear.scan_timeout);
  node->get_parameter(p + "rear_health_timeout", eval_config_.rear.health_timeout);
  node->get_parameter(p + "costmap_timeout", eval_config_.costmap_timeout);
  node->get_parameter(p + "footprint_timeout", eval_config_.footprint_timeout);
  node->get_parameter(p + "require_rear_health", eval_config_.rear.require_health);
  node->get_parameter(p + "acceleration_limit", acceleration_limit_);
  node->get_parameter(p + "deceleration_limit", deceleration_limit_);
  node->get_parameter(p + "minimum_speed", minimum_speed_);
  node->get_parameter(p + "max_backup_distance", max_backup_distance_);
  node->get_parameter(p + "max_backup_speed", max_backup_speed_);
  node->get_parameter(p + "rear_stop_margin", eval_config_.rear_stop_margin);
  node->get_parameter(p + "rear_corner_lateral_margin", eval_config_.rear_corner_lateral_margin);
  node->get_parameter(p + "front_overlap_min_x", eval_config_.front_overlap_min_x);
  node->get_parameter(p + "max_front_overlap_depth", eval_config_.max_front_overlap_depth);
  node->get_parameter(p + "max_front_overlap_ratio", eval_config_.max_front_overlap_ratio);
  node->get_parameter(p + "whitelist_match_radius", eval_config_.whitelist_match_radius);
  node->get_parameter(
    p + "min_corridor_observed_ratio", eval_config_.rear.min_corridor_observed_ratio);
  node->get_parameter(
    p + "min_near_zone_observed_ratio", eval_config_.rear.min_near_zone_observed_ratio);
  node->get_parameter(
    p + "min_center_observed_ratio", eval_config_.rear.min_center_observed_ratio);
  node->get_parameter(
    p + "min_sector_coverage_ratio", eval_config_.rear.min_sector_coverage_ratio);
  node->get_parameter(
    p + "min_required_x_max", eval_config_.rear.min_required_x_max);

  eval_config_.expected_costmap_frame = local_frame_;
  acceleration_limit_ = std::abs(acceleration_limit_);
  deceleration_limit_ = -std::abs(deceleration_limit_);

  evaluator_ = std::make_unique<CustomBackUpEvaluator>(eval_config_);

  local_costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    local_costmap_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&CustomBackUp::costmapCallback, this, std::placeholders::_1));

  local_footprint_sub_ = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
    local_footprint_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&CustomBackUp::footprintCallback, this, std::placeholders::_1));

  rear_scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    rear_scan_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&CustomBackUp::rearScanCallback, this, std::placeholders::_1));

  rear_health_sub_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    rear_health_topic_,
    rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&CustomBackUp::rearHealthCallback, this, std::placeholders::_1));

  RCLCPP_DEBUG(
    this->logger_,
    "[CUSTOM_BACKUP] Configured frame=%s costmap=%s footprint=%s rear_scan=%s rear_health=%s",
    local_frame_.c_str(), local_costmap_topic_.c_str(), local_footprint_topic_.c_str(),
    rear_scan_topic_.c_str(), rear_health_topic_.c_str());
}

void CustomBackUp::onCleanup()
{
  local_costmap_sub_.reset();
  local_footprint_sub_.reset();
  rear_scan_sub_.reset();
  rear_health_sub_.reset();

  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_costmap_.reset();
  latest_footprint_.reset();
  latest_rear_scan_.reset();
  latest_rear_health_.reset();

  last_vel_ = std::numeric_limits<double>::max();
  motion_started_ = false;
  completed_or_failed_ = false;
  session_ = BackupSessionState();
}

void CustomBackUp::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_costmap_ = msg;
}

void CustomBackUp::footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_footprint_ = msg;
}

void CustomBackUp::rearScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_rear_scan_ = msg;
}

void CustomBackUp::rearHealthCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_rear_health_ = msg;
}

bool CustomBackUp::populateCurrentPose(Pose2D & pose_out)
{
  try {
    const auto tf_msg = this->tf_->lookupTransform(
      local_frame_, this->robot_base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(0.01));

    if (tf_msg.header.stamp.sec != 0 || tf_msg.header.stamp.nanosec != 0) {
      const rclcpp::Time tf_time(tf_msg.header.stamp, this->clock_->get_clock_type());
      const double tf_age = (this->clock_->now() - tf_time).seconds();
      if (!std::isfinite(tf_age) || tf_age > this->transform_tolerance_ || tf_age < -0.50) {
        return false;
      }
    }

    pose_out.x = tf_msg.transform.translation.x;
    pose_out.y = tf_msg.transform.translation.y;
    pose_out.yaw = quaternionToYaw(tf_msg.transform.rotation);
    return std::isfinite(pose_out.x) && std::isfinite(pose_out.y) && std::isfinite(pose_out.yaw);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->logger_, "[CUSTOM_BACKUP] Current pose TF failed (%s -> %s): %s",
      this->robot_base_frame_.c_str(), local_frame_.c_str(), ex.what());
    return false;
  }
}

bool CustomBackUp::populateRearTf(RearSafetyInput & rear_in, const std::string & scan_frame)
{
  if (scan_frame.empty()) {
    return false;
  }
  if (scan_frame == this->robot_base_frame_) {
    rear_in.tf_valid = true;
    rear_in.scan_to_base_tx = 0.0;
    rear_in.scan_to_base_ty = 0.0;
    rear_in.scan_to_base_yaw = 0.0;
    return true;
  }
  try {
    const auto tf_msg = this->tf_->lookupTransform(
      this->robot_base_frame_, scan_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.01));
    rear_in.scan_to_base_tx = tf_msg.transform.translation.x;
    rear_in.scan_to_base_ty = tf_msg.transform.translation.y;
    rear_in.scan_to_base_yaw = quaternionToYaw(tf_msg.transform.rotation);
    rear_in.tf_valid = std::isfinite(rear_in.scan_to_base_yaw);
    return rear_in.tf_valid;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->logger_, "[CUSTOM_BACKUP] Rear TF lookup failed (%s -> %s): %s",
      scan_frame.c_str(), this->robot_base_frame_.c_str(), ex.what());
    rear_in.tf_valid = false;
    return false;
  }
}

bool CustomBackUp::populateFootprintInBase(
  std::vector<geometry_msgs::msg::Point> & fp_out,
  rclcpp::Time & stamp_out)
{
  fp_out.clear();
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr fp_msg;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    fp_msg = latest_footprint_;
  }
  if (!fp_msg || fp_msg->polygon.points.size() < 3 || fp_msg->header.frame_id.empty()) {
    return false;
  }

  stamp_out = rclcpp::Time(fp_msg->header.stamp, this->clock_->get_clock_type());
  if (fp_msg->header.frame_id == this->robot_base_frame_) {
    fp_out.reserve(fp_msg->polygon.points.size());
    for (const auto & p32 : fp_msg->polygon.points) {
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(p32.x);
      p.y = static_cast<double>(p32.y);
      p.z = static_cast<double>(p32.z);
      fp_out.push_back(p);
    }
    return true;
  }

  try {
    const tf2::TimePoint tp =
      (fp_msg->header.stamp.sec == 0 && fp_msg->header.stamp.nanosec == 0) ?
      tf2::TimePointZero : tf2_ros::fromMsg(fp_msg->header.stamp);
    const auto tf_msg = this->tf_->lookupTransform(
      this->robot_base_frame_, fp_msg->header.frame_id, tp,
      tf2::durationFromSec(0.01));
    const double tx = tf_msg.transform.translation.x;
    const double ty = tf_msg.transform.translation.y;
    const double yaw = quaternionToYaw(tf_msg.transform.rotation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    fp_out.reserve(fp_msg->polygon.points.size());
    for (const auto & p32 : fp_msg->polygon.points) {
      geometry_msgs::msg::Point p;
      const double px = static_cast<double>(p32.x);
      const double py = static_cast<double>(p32.y);
      p.x = tx + c * px - s * py;
      p.y = ty + s * px + c * py;
      p.z = 0.0;
      fp_out.push_back(p);
    }
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->logger_, "[CUSTOM_BACKUP] Footprint TF failed (%s -> %s): %s",
      fp_msg->header.frame_id.c_str(), this->robot_base_frame_.c_str(), ex.what());
    fp_out.clear();
    return false;
  }
}

BackupEvalInput CustomBackUp::buildEvalInput(
  double remaining_dist,
  double traveled_signed,
  double current_speed,
  const rclcpp::Time & now,
  bool & pose_ok,
  Pose2D & current_pose_out)
{
  BackupEvalInput input;
  input.now = now;
  input.remaining_dist = remaining_dist;
  input.traveled_signed = traveled_signed;
  input.current_speed = current_speed;

  pose_ok = populateCurrentPose(current_pose_out);
  input.pose_valid = pose_ok;
  input.current_pose = current_pose_out;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    input.costmap = latest_costmap_;
    input.rear_input.scan = latest_rear_scan_;
    input.rear_input.health = latest_rear_health_;
  }

  input.footprint_valid = populateFootprintInBase(input.footprint_in_base, input.footprint_stamp);

  if (input.rear_input.scan) {
    populateRearTf(input.rear_input, input.rear_input.scan->header.frame_id);
  }
  return input;
}

void CustomBackUp::logFailedOnce(
  BackupFailureCode code,
  double dist,
  const std::string & debug_detail)
{
  if (completed_or_failed_) {
    return;
  }
  completed_or_failed_ = true;
  RCLCPP_DEBUG(
    this->logger_, "[CUSTOM_BACKUP] Failure detail: reason=%s detail=%s dist=%.3fm",
    toString(code), debug_detail.c_str(), dist);
  RCLCPP_ERROR(
    this->logger_, "[CUSTOM_BACKUP] FAILED reason=%s dist=%.3fm",
    toString(code), dist);
}

nav2_behaviors::Status CustomBackUp::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  completed_or_failed_ = false;
  motion_started_ = false;
  last_traveled_dist_ = 0.0;
  last_vel_ = std::numeric_limits<double>::max();
  session_ = BackupSessionState();

  if (!command) {
    logFailedOnce(BackupFailureCode::INVALID_GOAL, 0.0, "null_goal");
    return nav2_behaviors::Status::FAILED;
  }

  const double tx = static_cast<double>(command->target.x);
  const double ty = static_cast<double>(command->target.y);
  const double tz = static_cast<double>(command->target.z);
  const double spd = static_cast<double>(command->speed);
  time_allowance_sec_ = rclcpp::Duration(command->time_allowance).seconds();

  if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz) ||
    !std::isfinite(spd) || !std::isfinite(time_allowance_sec_) ||
    ty != 0.0 || tz != 0.0 ||
    std::abs(tx) < 1e-4 || std::abs(tx) > max_backup_distance_ ||
    std::abs(spd) < 1e-4 || std::abs(spd) > max_backup_speed_ ||
    time_allowance_sec_ < 0.0)
  {
    std::ostringstream oss;
    oss << "invalid_goal_fields(tx=" << tx << ",ty=" << ty << ",tz=" << tz
        << ",spd=" << spd << ",timeout=" << time_allowance_sec_ << ")";
    logFailedOnce(BackupFailureCode::INVALID_GOAL, 0.0, oss.str());
    return nav2_behaviors::Status::FAILED;
  }

  target_dist_ = std::abs(tx);
  cmd_speed_ = -std::abs(spd);
  command_time_allowance_ = command->time_allowance;

  const rclcpp::Time now = this->clock_->now();
  start_time_ = now;
  end_time_ = now + command_time_allowance_;

  bool pose_ok = false;
  Pose2D start_pose;
  BackupEvalInput eval_in = buildEvalInput(
    target_dist_, 0.0, std::abs(cmd_speed_), now, pose_ok, start_pose);

  if (!pose_ok) {
    logFailedOnce(BackupFailureCode::TF_ERROR, 0.0, "initial_local_pose_unavailable");
    return nav2_behaviors::Status::FAILED;
  }

  initial_pose_.header.frame_id = local_frame_;
  initial_pose_.header.stamp = now;
  initial_pose_.pose.position.x = start_pose.x;
  initial_pose_.pose.position.y = start_pose.y;
  initial_yaw_ = start_pose.yaw;

  const BackupEvalResult start_res = evaluator_->initializeAndEvaluateStart(eval_in, session_);
  RCLCPP_DEBUG(
    this->logger_,
    "[CUSTOM_BACKUP] Start eval safe=%s reason=%s detail=%s samples=%d rear_min=%.3fm req_reach=%.3fm",
    start_res.safe ? "true" : "false", toString(start_res.failure_code), start_res.detail.c_str(),
    start_res.samples_checked, start_res.rear_result.min_obstacle_dist_scan,
    start_res.rear_result.required_scan_reach);

  if (!start_res.safe) {
    this->stopRobot();
    logFailedOnce(start_res.failure_code, 0.0, start_res.detail);
    return nav2_behaviors::Status::FAILED;
  }

  motion_started_ = true;
  RCLCPP_INFO(
    this->logger_,
    "[CUSTOM_BACKUP] START target=%.2fm speed=%.2fm/s timeout=%.2fs",
    target_dist_, std::abs(cmd_speed_), time_allowance_sec_);

  if (start_res.front_overlap_accepted) {
    RCLCPP_INFO(
      this->logger_,
      "[CUSTOM_BACKUP] FRONT_OVERLAP_ACCEPTED count=%zu max_penetration=%.3fm",
      start_res.front_overlap_count, start_res.front_overlap_penetration);
  }

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status CustomBackUp::onCycleUpdate()
{
  const rclcpp::Time now = this->clock_->now();
  const rclcpp::Duration time_remaining = end_time_ - now;
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    this->stopRobot();
    logFailedOnce(BackupFailureCode::TIMEOUT, last_traveled_dist_, "time_allowance_exceeded");
    return nav2_behaviors::Status::FAILED;
  }

  Pose2D current_pose;
  bool pose_ok = false;
  if (!populateCurrentPose(current_pose)) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_, "[CUSTOM_BACKUP] SAFETY_STOP reason=%s dist=%.3fm",
      toString(BackupFailureCode::TF_ERROR), last_traveled_dist_);
    logFailedOnce(BackupFailureCode::TF_ERROR, last_traveled_dist_, "cycle_pose_tf_unavailable");
    return nav2_behaviors::Status::FAILED;
  }

  const double dx = current_pose.x - initial_pose_.pose.position.x;
  const double dy = current_pose.y - initial_pose_.pose.position.y;
  const double traveled_signed = -(dx * std::cos(initial_yaw_) + dy * std::sin(initial_yaw_));
  const double distance = std::max(0.0, traveled_signed);
  last_traveled_dist_ = distance;

  feedback_->distance_traveled = static_cast<float>(distance);
  this->action_server_->publish_feedback(feedback_);

  if (distance >= target_dist_) {
    this->stopRobot();
    const double elapsed = (now - start_time_).seconds();
    completed_or_failed_ = true;
    RCLCPP_INFO(
      this->logger_, "[CUSTOM_BACKUP] SUCCESS dist=%.3fm elapsed=%.2fs",
      distance, elapsed);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  const double remaining_dist = std::max(0.0, target_dist_ - distance);
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = 0.0;
  cmd_vel->angular.z = 0.0;

  if (acceleration_limit_ == 0.0 || deceleration_limit_ == 0.0) {
    cmd_vel->linear.x = cmd_speed_;
  } else {
    const double current_speed =
      (last_vel_ == std::numeric_limits<double>::max()) ? 0.0 : last_vel_;
    const double min_feasible_speed = current_speed - acceleration_limit_ / this->cycle_frequency_;
    const double max_feasible_speed = current_speed - deceleration_limit_ / this->cycle_frequency_;
    cmd_vel->linear.x = std::clamp(cmd_speed_, min_feasible_speed, max_feasible_speed);

    const double max_vel_to_stop = std::sqrt(-2.0 * deceleration_limit_ * remaining_dist);
    if (max_vel_to_stop < std::abs(cmd_vel->linear.x)) {
      cmd_vel->linear.x = -max_vel_to_stop;
    }
  }

  if (minimum_speed_ > 0.0 && std::abs(cmd_vel->linear.x) < minimum_speed_) {
    cmd_vel->linear.x = -minimum_speed_;
  }

  BackupEvalInput eval_in = buildEvalInput(
    remaining_dist, traveled_signed, std::abs(cmd_vel->linear.x), now, pose_ok, current_pose);
  const BackupEvalResult cycle_res = evaluator_->evaluateCycle(eval_in, session_);

  RCLCPP_DEBUG_THROTTLE(
    this->logger_, *this->clock_, 500,
    "[CUSTOM_BACKUP] Cycle dist=%.3fm rem=%.3fm safe=%s reason=%s detail=%s rear_min=%.3fm",
    distance, remaining_dist, cycle_res.safe ? "true" : "false",
    toString(cycle_res.failure_code), cycle_res.detail.c_str(),
    cycle_res.rear_result.min_obstacle_dist_scan);

  if (!cycle_res.safe) {
    this->stopRobot();
    RCLCPP_WARN(
      this->logger_, "[CUSTOM_BACKUP] SAFETY_STOP reason=%s dist=%.3fm",
      toString(cycle_res.failure_code), distance);
    logFailedOnce(cycle_res.failure_code, distance, cycle_res.detail);
    return nav2_behaviors::Status::FAILED;
  }

  last_vel_ = cmd_vel->linear.x;
  this->vel_pub_->publish(std::move(cmd_vel));
  return nav2_behaviors::Status::RUNNING;
}

void CustomBackUp::onActionCompletion()
{
  if (motion_started_ && !completed_or_failed_) {
    logFailedOnce(BackupFailureCode::CANCELED, last_traveled_dist_, "action_canceled_or_preempted");
  }
  last_vel_ = std::numeric_limits<double>::max();
  motion_started_ = false;
  session_ = BackupSessionState();
}

}  // namespace nav2_custom_plugins

PLUGINLIB_EXPORT_CLASS(nav2_custom_plugins::CustomBackUp, nav2_core::Behavior)
