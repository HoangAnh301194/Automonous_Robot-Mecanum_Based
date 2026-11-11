#include "nav2_custom_plugins/rear_safety_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace nav2_custom_plugins
{

const char * toString(RearSafetyReason reason)
{
  switch (reason) {
    case RearSafetyReason::SAFE:
      return "SAFE";
    case RearSafetyReason::REAR_OBSTACLE:
      return "REAR_OBSTACLE";
    case RearSafetyReason::SENSOR_STALE:
      return "SENSOR_STALE";
    case RearSafetyReason::SENSOR_INVALID:
      return "SENSOR_INVALID";
    case RearSafetyReason::TF_ERROR:
      return "TF_ERROR";
    default:
      return "SENSOR_INVALID";
  }
}

namespace
{

bool parseBoolValue(const std::string & s, bool & out)
{
  if (s == "true" || s == "True" || s == "1") {
    out = true;
    return true;
  }
  if (s == "false" || s == "False" || s == "0") {
    out = false;
    return true;
  }
  return false;
}

bool parseDoubleValue(const std::string & s, double & out)
{
  try {
    size_t idx = 0;
    double val = std::stod(s, &idx);
    if (idx == 0 || !std::isfinite(val)) {
      return false;
    }
    out = val;
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

RearSafetyEvaluator::RearSafetyEvaluator(const RearSafetyConfig & config)
: config_(config)
{
}

ParsedRearHealth RearSafetyEvaluator::parseHealth(const diagnostic_msgs::msg::DiagnosticArray & msg)
{
  ParsedRearHealth parsed;
  parsed.stamp = rclcpp::Time(msg.header.stamp);
  if (msg.status.empty()) {
    return parsed;
  }

  // Prefer matching status by name if multiple diagnostics share the array.
  const diagnostic_msgs::msg::DiagnosticStatus * target = &msg.status.front();
  for (const auto & st : msg.status) {
    if (st.name.find("rear") != std::string::npos) {
      target = &st;
      break;
    }
  }

  parsed.has_status = true;
  parsed.level = target->level;

  bool has_cam = false;
  bool has_tf = false;
  bool has_ground = false;
  bool has_corr = false;
  bool has_corr_ratio = false;
  bool has_near_ratio = false;
  bool has_sector_ratio = false;
  bool has_amin = false;
  bool has_amax = false;
  bool has_xmin = false;
  bool has_xmax = false;
  bool has_ymin = false;
  bool has_ymax = false;
  bool parse_error = false;

  for (const auto & kv : target->values) {
    bool bval = false;
    double dval = 0.0;
    if (kv.key == "camera_ok") {
      has_cam = parseBoolValue(kv.value, bval);
      parsed.camera_ok = bval;
      parse_error |= !has_cam;
    } else if (kv.key == "tf_ok") {
      has_tf = parseBoolValue(kv.value, bval);
      parsed.tf_ok = bval;
      parse_error |= !has_tf;
    } else if (kv.key == "ground_ok") {
      has_ground = parseBoolValue(kv.value, bval);
      parsed.ground_ok = bval;
      parse_error |= !has_ground;
    } else if (kv.key == "corridor_observed") {
      has_corr = parseBoolValue(kv.value, bval);
      parsed.corridor_observed = bval;
      parse_error |= !has_corr;
    } else if (kv.key == "valid_ratio") {
      if (parseDoubleValue(kv.value, dval)) {
        parsed.valid_ratio = dval;
      } else {
        parse_error = true;
      }
    } else if (kv.key == "current_valid_ratio") {
      if (parseDoubleValue(kv.value, dval)) {
        parsed.current_valid_ratio = dval;
      } else {
        parse_error = true;
      }
    } else if (kv.key == "corridor_observed_ratio") {
      has_corr_ratio = parseDoubleValue(kv.value, dval);
      parsed.corridor_observed_ratio = dval;
      parse_error |= !has_corr_ratio;
    } else if (kv.key == "near_zone_observed_ratio") {
      has_near_ratio = parseDoubleValue(kv.value, dval);
      parsed.near_zone_observed_ratio = dval;
      parse_error |= !has_near_ratio;
    } else if (kv.key == "center_observed_ratio") {
      parsed.has_center_observed_ratio = parseDoubleValue(kv.value, dval);
      parsed.center_observed_ratio = dval;
      parse_error |= !parsed.has_center_observed_ratio;
    } else if (kv.key == "sector_coverage_ratio") {
      has_sector_ratio = parseDoubleValue(kv.value, dval);
      parsed.sector_coverage_ratio = dval;
      parse_error |= !has_sector_ratio;
    } else if (kv.key == "observed_angle_min") {
      has_amin = parseDoubleValue(kv.value, dval);
      parsed.observed_angle_min = dval;
      parse_error |= !has_amin;
    } else if (kv.key == "observed_angle_max") {
      has_amax = parseDoubleValue(kv.value, dval);
      parsed.observed_angle_max = dval;
      parse_error |= !has_amax;
    } else if (kv.key == "observed_x_min") {
      has_xmin = parseDoubleValue(kv.value, dval);
      parsed.observed_x_min = dval;
      parse_error |= !has_xmin;
    } else if (kv.key == "observed_x_max") {
      has_xmax = parseDoubleValue(kv.value, dval);
      parsed.observed_x_max = dval;
      parse_error |= !has_xmax;
    } else if (kv.key == "observed_y_min") {
      has_ymin = parseDoubleValue(kv.value, dval);
      parsed.observed_y_min = dval;
      parse_error |= !has_ymin;
    } else if (kv.key == "observed_y_max") {
      has_ymax = parseDoubleValue(kv.value, dval);
      parsed.observed_y_max = dval;
      parse_error |= !has_ymax;
    }
  }

  parsed.has_observability_metrics =
    has_corr_ratio && has_near_ratio && has_sector_ratio &&
    has_amin && has_amax && has_xmin && has_xmax && has_ymin && has_ymax;
  parsed.valid = !parse_error && has_cam && has_tf && has_ground && has_corr;
  return parsed;
}

RearSafetyResult RearSafetyEvaluator::evaluate(const RearSafetyInput & input) const
{
  RearSafetyResult result;

  if (!input.scan) {
    result.reason = RearSafetyReason::SENSOR_STALE;
    result.detail = "rear_scan_missing";
    return result;
  }

  const auto & scan = *input.scan;
  if (scan.header.stamp.sec == 0 && scan.header.stamp.nanosec == 0) {
    result.reason = RearSafetyReason::SENSOR_INVALID;
    result.detail = "rear_scan_zero_stamp";
    return result;
  }

  const rclcpp::Time scan_time(scan.header.stamp, input.now.get_clock_type());
  result.scan_age = (input.now - scan_time).seconds();
  if (!std::isfinite(result.scan_age) ||
    result.scan_age > config_.scan_timeout ||
    result.scan_age < -config_.max_future_skew)
  {
    result.reason = RearSafetyReason::SENSOR_STALE;
    result.detail = "rear_scan_stale_or_future";
    return result;
  }

  if (scan.ranges.size() < 2 ||
    !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
    scan.range_max <= scan.range_min ||
    !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
    std::abs(scan.angle_increment) <= 1e-6)
  {
    result.reason = RearSafetyReason::SENSOR_INVALID;
    result.detail = "rear_scan_malformed_metadata";
    return result;
  }

  if (!input.tf_valid ||
    !std::isfinite(input.scan_to_base_tx) ||
    !std::isfinite(input.scan_to_base_ty) ||
    !std::isfinite(input.scan_to_base_yaw))
  {
    result.reason = RearSafetyReason::TF_ERROR;
    result.detail = "rear_scan_tf_invalid";
    return result;
  }

  const double cam_to_bumper = std::max(0.0, input.scan_to_base_tx - config_.robot_rear_x);
  const double baseline_bumper_clearance = std::max(0.0, config_.min_distance - cam_to_bumper);

  const double speed = std::isfinite(input.current_speed) ? std::abs(input.current_speed) : 0.0;
  const double rem_dist = std::isfinite(input.remaining_dist) ? std::max(0.0, input.remaining_dist) : 0.0;
  const double eff_age = std::max(0.0, result.scan_age);
  const double latency_dist = speed * (eff_age + config_.control_latency);
  const double braking_dist = (config_.braking_decel > 1e-4) ?
    (speed * speed) / (2.0 * config_.braking_decel) : 0.0;
  const double dynamic_bumper_clearance =
    rem_dist + latency_dist + braking_dist + config_.rear_safety_margin;

  result.required_bumper_clearance = std::max(baseline_bumper_clearance, dynamic_bumper_clearance);
  result.required_scan_reach = std::min(
    config_.max_required_reach,
    std::max(config_.min_required_x_max, cam_to_bumper + result.required_bumper_clearance));

  const double cos_yaw = std::cos(input.scan_to_base_yaw);
  const double sin_yaw = std::sin(input.scan_to_base_yaw);
  const double corridor_half_width = config_.robot_half_width + config_.corridor_lateral_margin;
  const double rear_x_cutoff_in_base = std::max(
    input.scan_to_base_tx + 0.05,
    config_.robot_rear_x + 0.15);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = static_cast<double>(scan.ranges[i]);
    if (std::isnan(r) || r <= 0.0 || r < static_cast<double>(scan.range_min)) {
      ++result.invalid_count;
      continue;
    }
    if (std::isinf(r)) {
      if (r > 0.0) {
        ++result.pos_inf_count;
      } else {
        ++result.invalid_count;
      }
      continue;
    }
    if (r > static_cast<double>(scan.range_max)) {
      ++result.pos_inf_count;
      continue;
    }

    ++result.finite_count;
    const double theta = static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    const double x_s = r * std::cos(theta);
    const double y_s = r * std::sin(theta);

    const double x_b = input.scan_to_base_tx + cos_yaw * x_s - sin_yaw * y_s;
    const double y_b = input.scan_to_base_ty + sin_yaw * x_s + cos_yaw * y_s;

    if (x_b <= rear_x_cutoff_in_base && std::abs(y_b) <= corridor_half_width) {
      const double bumper_clearance = config_.robot_rear_x - x_b;
      if (bumper_clearance < result.min_bumper_clearance) {
        result.min_bumper_clearance = bumper_clearance;
      }
      if (x_s > 0.0 && x_s < result.min_obstacle_dist_scan) {
        result.min_obstacle_dist_scan = x_s;
      }
    }
  }

  // Always block immediately on a detected obstacle inside the rear corridor clearance
  if (result.finite_count > 0 &&
    result.min_bumper_clearance <= result.required_bumper_clearance)
  {
    std::ostringstream oss;
    oss << "obstacle_bumper_clearance=" << result.min_bumper_clearance
        << "<=req=" << result.required_bumper_clearance;
    result.reason = RearSafetyReason::REAR_OBSTACLE;
    result.detail = oss.str();
    return result;
  }

  const int total_rays = static_cast<int>(scan.ranges.size());
  const double invalid_ratio = static_cast<double>(result.invalid_count) /
    static_cast<double>(std::max(1, total_rays));
  if ((result.finite_count + result.pos_inf_count) == 0 || invalid_ratio > config_.max_nan_ratio) {
    result.reason = RearSafetyReason::SENSOR_INVALID;
    result.detail = "rear_scan_too_many_invalid_rays";
    return result;
  }

  if (config_.require_health) {
    if (!input.health) {
      result.reason = RearSafetyReason::SENSOR_STALE;
      result.detail = "rear_health_missing";
      return result;
    }

    const ParsedRearHealth health = parseHealth(*input.health);
    if (!health.has_status || !health.valid) {
      result.reason = RearSafetyReason::SENSOR_INVALID;
      result.detail = "rear_health_malformed_or_incomplete";
      return result;
    }

    if (input.health->header.stamp.sec == 0 && input.health->header.stamp.nanosec == 0) {
      result.reason = RearSafetyReason::SENSOR_INVALID;
      result.detail = "rear_health_zero_stamp";
      return result;
    }

    const rclcpp::Time health_time(input.health->header.stamp, input.now.get_clock_type());
    result.health_age = (input.now - health_time).seconds();
    if (!std::isfinite(result.health_age) ||
      result.health_age > config_.health_timeout ||
      result.health_age < -config_.max_future_skew)
    {
      result.reason = RearSafetyReason::SENSOR_STALE;
      result.detail = "rear_health_stale";
      return result;
    }

    const double skew = std::abs((scan_time - health_time).seconds());
    if (!std::isfinite(skew) || skew > config_.max_scan_health_skew) {
      result.reason = RearSafetyReason::SENSOR_STALE;
      result.detail = "rear_scan_health_time_skew";
      return result;
    }

    result.corridor_observed = health.corridor_observed;
    if (health.level != diagnostic_msgs::msg::DiagnosticStatus::OK ||
      !health.camera_ok || !health.tf_ok || !health.ground_ok ||
      !health.corridor_observed)
    {
      result.reason = RearSafetyReason::SENSOR_INVALID;
      result.detail = "rear_camera_unhealthy_or_corridor_unobserved";
      return result;
    }

    if (!health.has_observability_metrics) {
      result.reason = RearSafetyReason::SENSOR_INVALID;
      result.detail = "rear_health_missing_observability_metrics";
      return result;
    }

    const double center_ratio = health.has_center_observed_ratio ?
      health.center_observed_ratio : health.corridor_observed_ratio;

    const bool geom_ok =
      health.corridor_observed_ratio >= config_.min_corridor_observed_ratio &&
      health.near_zone_observed_ratio >= config_.min_near_zone_observed_ratio &&
      center_ratio >= config_.min_center_observed_ratio &&
      health.sector_coverage_ratio >= config_.min_sector_coverage_ratio &&
      health.observed_angle_min <= config_.max_observed_angle_min &&
      health.observed_angle_max >= config_.min_observed_angle_max &&
      health.observed_y_min <= config_.max_observed_y_min &&
      health.observed_y_max >= config_.min_observed_y_max &&
      health.observed_x_min <= config_.max_observed_x_min &&
      health.observed_x_max >= result.required_scan_reach;

    if (!geom_ok) {
      std::ostringstream oss;
      oss << "observability_insufficient(corr=" << health.corridor_observed_ratio
          << ",near=" << health.near_zone_observed_ratio
          << ",center=" << center_ratio
          << ",sector=" << health.sector_coverage_ratio
          << ",ang=[" << health.observed_angle_min << "," << health.observed_angle_max << "]"
          << ",y=[" << health.observed_y_min << "," << health.observed_y_max << "]"
          << ",x=[" << health.observed_x_min << "," << health.observed_x_max << "]"
          << ",req_reach=" << result.required_scan_reach << ")";
      result.reason = RearSafetyReason::SENSOR_INVALID;
      result.detail = oss.str();
      return result;
    }
  } else {
    result.corridor_observed = (result.finite_count > 0 || result.pos_inf_count > 0);
  }

  result.safe = true;
  result.reason = RearSafetyReason::SAFE;
  result.detail = "rear_corridor_verified_safe";
  return result;
}

}  // namespace nav2_custom_plugins

