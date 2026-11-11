#include "nav2_custom_plugins/custom_backup_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace nav2_custom_plugins
{

namespace
{

constexpr uint8_t kNoInformation = 255;

struct CostmapGridTransform
{
  double ox{0.0};
  double oy{0.0};
  double cos_yaw{1.0};
  double sin_yaw{0.0};
  double res{0.1};
  int size_x{0};
  int size_y{0};

  bool worldToMap(double wx, double wy, int & mx, int & my) const
  {
    const double dx = wx - ox;
    const double dy = wy - oy;
    const double lx = cos_yaw * dx + sin_yaw * dy;
    const double ly = -sin_yaw * dx + cos_yaw * dy;
    mx = static_cast<int>(std::floor(lx / res));
    my = static_cast<int>(std::floor(ly / res));
    return mx >= 0 && mx < size_x && my >= 0 && my < size_y;
  }

  void mapToWorldCenter(int mx, int my, double & wx, double & wy) const
  {
    const double lx = (static_cast<double>(mx) + 0.5) * res;
    const double ly = (static_cast<double>(my) + 0.5) * res;
    wx = ox + cos_yaw * lx - sin_yaw * ly;
    wy = oy + sin_yaw * lx + cos_yaw * ly;
  }

  uint8_t getCost(const nav2_msgs::msg::Costmap & cm, int mx, int my) const
  {
    return cm.data[static_cast<size_t>(my) * static_cast<size_t>(size_x) +
      static_cast<size_t>(mx)];
  }
};

struct CellSample
{
  int mx{0};
  int my{0};
  uint8_t cost{0};
  double wx{0.0};
  double wy{0.0};
  double x_rel{0.0};
  double y_rel{0.0};
};

bool pointInPolygon(
  double x, double y,
  const std::vector<geometry_msgs::msg::Point> & poly)
{
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[i].x;
    const double yi = poly[i].y;
    const double xj = poly[j].x;
    const double yj = poly[j].y;

    const bool intersect = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

bool isWithinWhitelist(
  double wx, double wy,
  const std::vector<geometry_msgs::msg::Point> & whitelist,
  double radius)
{
  const double r2 = radius * radius;
  for (const auto & pt : whitelist) {
    const double dx = wx - pt.x;
    const double dy = wy - pt.y;
    if ((dx * dx + dy * dy) <= r2) {
      return true;
    }
  }
  return false;
}

void poseBaseToWorld(
  const Pose2D & pose,
  double xb, double yb,
  double & wx, double & wy)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  wx = pose.x + c * xb - s * yb;
  wy = pose.y + s * xb + c * yb;
}

void poseWorldToBase(
  const Pose2D & pose,
  double wx, double wy,
  double & xb, double & yb)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double dx = wx - pose.x;
  const double dy = wy - pose.y;
  xb = c * dx + s * dy;
  yb = -s * dx + c * dy;
}

bool collectPoseCells(
  const nav2_msgs::msg::Costmap & cm,
  const CostmapGridTransform & grid,
  const Pose2D & pose,
  const std::vector<geometry_msgs::msg::Point> & footprint_in_base,
  double fp_min_x,
  double fp_max_x,
  double fp_min_y,
  double fp_max_y,
  const CustomBackUpConfig & cfg,
  std::vector<CellSample> & out_footprint_cells,
  std::vector<CellSample> & out_guard_cells,
  std::string & err_detail)
{
  out_footprint_cells.clear();
  out_guard_cells.clear();

  const size_t total_cells = static_cast<size_t>(grid.size_x) * static_cast<size_t>(grid.size_y);
  // Deduplicate cells per pose check using small uint8 masks (footprint = bit 1, guard = bit 2).
  std::vector<uint8_t> cell_marks(total_cells, 0);

  auto addCell = [&](int mx, int my, uint8_t bit) -> void {
    const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(grid.size_x) +
      static_cast<size_t>(mx);
    if ((cell_marks[idx] & bit) != 0) {
      return;
    }
    cell_marks[idx] |= bit;

    CellSample sample;
    sample.mx = mx;
    sample.my = my;
    sample.cost = grid.getCost(cm, mx, my);
    grid.mapToWorldCenter(mx, my, sample.wx, sample.wy);
    poseWorldToBase(pose, sample.wx, sample.wy, sample.x_rel, sample.y_rel);
    if (bit == 1) {
      out_footprint_cells.push_back(sample);
    } else {
      out_guard_cells.push_back(sample);
    }
  };

  // Transform footprint vertices into world coordinates and compute grid bounding box.
  std::vector<geometry_msgs::msg::Point> fp_world(footprint_in_base.size());
  for (size_t i = 0; i < footprint_in_base.size(); ++i) {
    poseBaseToWorld(pose, footprint_in_base[i].x, footprint_in_base[i].y, fp_world[i].x, fp_world[i].y);
    int vx = 0;
    int vy = 0;
    if (!grid.worldToMap(fp_world[i].x, fp_world[i].y, vx, vy)) {
      err_detail = "footprint_vertex_out_of_costmap_bounds";
      return false;
    }
  }

  // Also ensure the rear stop & rear corner safety rectangle lies within costmap bounds.
  const double guard_min_x = fp_min_x - cfg.rear_stop_margin;
  const double guard_min_y = fp_min_y - cfg.rear_corner_lateral_margin;
  const double guard_max_y = fp_max_y + cfg.rear_corner_lateral_margin;

  const double ext_corners[4][2] = {
    {guard_min_x, guard_min_y},
    {guard_min_x, guard_max_y},
    {fp_max_x, guard_max_y},
    {fp_max_x, guard_min_y}
  };

  int min_mx = grid.size_x - 1;
  int max_mx = 0;
  int min_my = grid.size_y - 1;
  int max_my = 0;

  for (int i = 0; i < 4; ++i) {
    double wx = 0.0;
    double wy = 0.0;
    poseBaseToWorld(pose, ext_corners[i][0], ext_corners[i][1], wx, wy);
    int mx = 0;
    int my = 0;
    if (!grid.worldToMap(wx, wy, mx, my)) {
      err_detail = "rear_guard_out_of_costmap_bounds";
      return false;
    }
    min_mx = std::min(min_mx, mx);
    max_mx = std::max(max_mx, mx);
    min_my = std::min(min_my, my);
    max_my = std::max(max_my, my);
  }

  min_mx = std::max(0, min_mx - 1);
  max_mx = std::min(grid.size_x - 1, max_mx + 1);
  min_my = std::max(0, min_my - 1);
  max_my = std::min(grid.size_y - 1, max_my + 1);

  // 1) Rasterize footprint polygon edges finely so thin obstacles on edges cannot slip through.
  const double edge_step = std::max(0.01, grid.res * 0.25);
  for (size_t i = 0; i < fp_world.size(); ++i) {
    const auto & p0 = fp_world[i];
    const auto & p1 = fp_world[(i + 1) % fp_world.size()];
    const double seg_len = std::hypot(p1.x - p0.x, p1.y - p0.y);
    const int n_steps = std::max(1, static_cast<int>(std::ceil(seg_len / edge_step)));
    for (int k = 0; k <= n_steps; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n_steps);
      const double sx = p0.x + t * (p1.x - p0.x);
      const double sy = p0.y + t * (p1.y - p0.y);
      int mx = 0;
      int my = 0;
      if (!grid.worldToMap(sx, sy, mx, my)) {
        err_detail = "footprint_edge_out_of_costmap_bounds";
        return false;
      }
      addCell(mx, my, 1);
    }
  }

  // 2) Include all interior cell centers inside footprint polygon.
  for (int my = min_my; my <= max_my; ++my) {
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      double wx = 0.0;
      double wy = 0.0;
      grid.mapToWorldCenter(mx, my, wx, wy);
      if (pointInPolygon(wx, wy, fp_world)) {
        addCell(mx, my, 1);
      }
    }
  }

  // 3) Explicitly sample rear stopping buffer strip AND left/right rear corner strips.
  const double rear_strip_x_end = fp_min_x + 0.02;
  const double corner_band_x_end = std::min(fp_max_x, fp_min_x + cfg.rear_corner_band_length);

  auto sampleBoxBase = [&](double bx0, double bx1, double by0, double by1) -> bool {
    const int nx = std::max(1, static_cast<int>(std::ceil((bx1 - bx0) / edge_step)));
    const int ny = std::max(1, static_cast<int>(std::ceil((by1 - by0) / edge_step)));
    for (int ix = 0; ix <= nx; ++ix) {
      const double bx = bx0 + (bx1 - bx0) * static_cast<double>(ix) / static_cast<double>(nx);
      for (int iy = 0; iy <= ny; ++iy) {
        const double by = by0 + (by1 - by0) * static_cast<double>(iy) / static_cast<double>(ny);
        double wx = 0.0;
        double wy = 0.0;
        poseBaseToWorld(pose, bx, by, wx, wy);
        int mx = 0;
        int my = 0;
        if (!grid.worldToMap(wx, wy, mx, my)) {
          return false;
        }
        addCell(mx, my, 2);
      }
    }
    return true;
  };

  if (cfg.rear_stop_margin > 1e-4) {
    if (!sampleBoxBase(guard_min_x, rear_strip_x_end, guard_min_y, guard_max_y)) {
      err_detail = "rear_stop_strip_out_of_bounds";
      return false;
    }
  }
  if (cfg.rear_corner_lateral_margin > 1e-4 && cfg.rear_corner_band_length > 1e-4) {
    if (!sampleBoxBase(guard_min_x, corner_band_x_end, guard_min_y, fp_min_y) ||
      !sampleBoxBase(guard_min_x, corner_band_x_end, fp_max_y, guard_max_y))
    {
      err_detail = "rear_corner_strip_out_of_bounds";
      return false;
    }
  }

  return true;
}

}  // namespace

const char * toString(BackupFailureCode code)
{
  switch (code) {
    case BackupFailureCode::NONE:
      return "NONE";
    case BackupFailureCode::INVALID_GOAL:
      return "INVALID_GOAL";
    case BackupFailureCode::REAR_OBSTACLE:
      return "REAR_OBSTACLE";
    case BackupFailureCode::SENSOR_STALE:
      return "SENSOR_STALE";
    case BackupFailureCode::SENSOR_INVALID:
      return "SENSOR_INVALID";
    case BackupFailureCode::TF_ERROR:
      return "TF_ERROR";
    case BackupFailureCode::COSTMAP_INVALID:
      return "COSTMAP_INVALID";
    case BackupFailureCode::TRAJECTORY_BLOCKED:
      return "TRAJECTORY_BLOCKED";
    case BackupFailureCode::TIMEOUT:
      return "TIMEOUT";
    case BackupFailureCode::CANCELED:
      return "CANCELED";
    default:
      return "TRAJECTORY_BLOCKED";
  }
}

BackupFailureCode mapRearReason(RearSafetyReason reason)
{
  switch (reason) {
    case RearSafetyReason::SAFE:
      return BackupFailureCode::NONE;
    case RearSafetyReason::REAR_OBSTACLE:
      return BackupFailureCode::REAR_OBSTACLE;
    case RearSafetyReason::SENSOR_STALE:
      return BackupFailureCode::SENSOR_STALE;
    case RearSafetyReason::SENSOR_INVALID:
      return BackupFailureCode::SENSOR_INVALID;
    case RearSafetyReason::TF_ERROR:
      return BackupFailureCode::TF_ERROR;
    default:
      return BackupFailureCode::SENSOR_INVALID;
  }
}

CustomBackUpEvaluator::CustomBackUpEvaluator(const CustomBackUpConfig & config)
: config_(config),
  rear_evaluator_(config.rear)
{
}

BackupEvalResult CustomBackUpEvaluator::initializeAndEvaluateStart(
  const BackupEvalInput & input,
  BackupSessionState & session) const
{
  session = BackupSessionState();
  return evaluateCommon(input, session, true);
}

BackupEvalResult CustomBackUpEvaluator::evaluateCycle(
  const BackupEvalInput & input,
  BackupSessionState & session) const
{
  return evaluateCommon(input, session, false);
}

BackupEvalResult CustomBackUpEvaluator::evaluateCommon(
  const BackupEvalInput & input,
  BackupSessionState & session,
  bool is_start) const
{
  BackupEvalResult result;

  // 1. Validate robot pose TF
  if (!input.pose_valid ||
    !std::isfinite(input.current_pose.x) ||
    !std::isfinite(input.current_pose.y) ||
    !std::isfinite(input.current_pose.yaw))
  {
    result.failure_code = BackupFailureCode::TF_ERROR;
    result.detail = "current_robot_pose_unavailable";
    return result;
  }

  if (!std::isfinite(input.remaining_dist) || input.remaining_dist < 0.0) {
    result.failure_code = BackupFailureCode::INVALID_GOAL;
    result.detail = "invalid_remaining_distance";
    return result;
  }

  // 2. Continuous mandatory rear depth camera safety verification
  RearSafetyInput rear_in = input.rear_input;
  rear_in.now = input.now;
  rear_in.remaining_dist = input.remaining_dist;
  rear_in.current_speed = input.current_speed;
  result.rear_result = rear_evaluator_.evaluate(rear_in);
  if (!result.rear_result.safe) {
    result.failure_code = mapRearReason(result.rear_result.reason);
    result.detail = result.rear_result.detail;
    return result;
  }

  // 3. Guard against forward physical push during backup
  if (!is_start && input.traveled_signed < -config_.max_forward_push_m) {
    result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
    result.detail = "robot_pushed_forward_excessively";
    return result;
  }

  // 4. Validate local costmap
  if (!input.costmap) {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_missing";
    return result;
  }

  const auto & cm = *input.costmap;
  if (cm.header.stamp.sec == 0 && cm.header.stamp.nanosec == 0) {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_zero_stamp";
    return result;
  }

  const rclcpp::Time cm_time(cm.header.stamp, input.now.get_clock_type());
  const double cm_age = (input.now - cm_time).seconds();
  if (!std::isfinite(cm_age) || cm_age > config_.costmap_timeout || cm_age < -0.50) {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_stale";
    return result;
  }

  if (!config_.expected_costmap_frame.empty() &&
    cm.header.frame_id != config_.expected_costmap_frame)
  {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_frame_mismatch";
    return result;
  }

  const double res = static_cast<double>(cm.metadata.resolution);
  if (cm.metadata.size_x == 0 || cm.metadata.size_y == 0 ||
    !std::isfinite(res) || res <= 0.0 || res > 1.0)
  {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_invalid_metadata";
    return result;
  }

  const size_t expected_len = static_cast<size_t>(cm.metadata.size_x) *
    static_cast<size_t>(cm.metadata.size_y);
  if (cm.data.size() != expected_len) {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_data_size_mismatch";
    return result;
  }

  if (!std::isfinite(cm.metadata.origin.position.x) ||
    !std::isfinite(cm.metadata.origin.position.y))
  {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_invalid_origin";
    return result;
  }

  CostmapGridTransform grid;
  grid.ox = cm.metadata.origin.position.x;
  grid.oy = cm.metadata.origin.position.y;
  grid.res = res;
  grid.size_x = static_cast<int>(cm.metadata.size_x);
  grid.size_y = static_cast<int>(cm.metadata.size_y);

  const auto & oq = cm.metadata.origin.orientation;
  const double q_norm = std::sqrt(oq.x * oq.x + oq.y * oq.y + oq.z * oq.z + oq.w * oq.w);
  if (!std::isfinite(q_norm) || q_norm < 1e-6) {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "local_costmap_invalid_orientation";
    return result;
  }
  tf2::Quaternion tf_q(oq.x / q_norm, oq.y / q_norm, oq.z / q_norm, oq.w / q_norm);
  double r_tmp = 0.0;
  double p_tmp = 0.0;
  double origin_yaw = 0.0;
  tf2::Matrix3x3(tf_q).getRPY(r_tmp, p_tmp, origin_yaw);
  grid.cos_yaw = std::cos(origin_yaw);
  grid.sin_yaw = std::sin(origin_yaw);

  // 5. Validate footprint polygon
  if (!input.footprint_valid || input.footprint_in_base.size() < 3) {
    result.failure_code = BackupFailureCode::COSTMAP_INVALID;
    result.detail = "robot_footprint_missing_or_invalid";
    return result;
  }

  if (input.footprint_stamp.nanoseconds() != 0) {
    const rclcpp::Time fp_time(input.footprint_stamp.nanoseconds(), input.now.get_clock_type());
    const double fp_age = (input.now - fp_time).seconds();
    if (!std::isfinite(fp_age) || fp_age > config_.footprint_timeout || fp_age < -0.50) {
      result.failure_code = BackupFailureCode::COSTMAP_INVALID;
      result.detail = "robot_footprint_stale";
      return result;
    }
  }

  double fp_min_x = std::numeric_limits<double>::infinity();
  double fp_max_x = -std::numeric_limits<double>::infinity();
  double fp_min_y = std::numeric_limits<double>::infinity();
  double fp_max_y = -std::numeric_limits<double>::infinity();
  for (const auto & pt : input.footprint_in_base) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
      result.failure_code = BackupFailureCode::COSTMAP_INVALID;
      result.detail = "robot_footprint_non_finite_vertex";
      return result;
    }
    fp_min_x = std::min(fp_min_x, pt.x);
    fp_max_x = std::max(fp_max_x, pt.x);
    fp_min_y = std::min(fp_min_y, pt.y);
    fp_max_y = std::max(fp_max_y, pt.y);
  }

  // 6. Simulate entire remaining backup trajectory P(s) along -heading
  const double step_m = std::max(0.005, std::min(res * config_.step_resolution_factor, config_.max_step_m));
  const int num_steps = (input.remaining_dist > 1e-4) ?
    std::max(1, static_cast<int>(std::ceil(input.remaining_dist / step_m))) : 0;

  const double heading_cos = std::cos(input.current_pose.yaw);
  const double heading_sin = std::sin(input.current_pose.yaw);

  std::vector<CellSample> fp_cells;
  std::vector<CellSample> guard_cells;
  fp_cells.reserve(128);
  guard_cells.reserve(128);

  size_t prev_overlap_count = 0;
  size_t init_overlap_count = 0;
  double init_penetration = 0.0;
  double init_min_x_rel = std::numeric_limits<double>::infinity();
  std::vector<geometry_msgs::msg::Point> candidate_front_world_pts;

  for (int step = 0; step <= num_steps; ++step) {
    const double s = (step == num_steps) ? input.remaining_dist :
      std::min(input.remaining_dist, static_cast<double>(step) * step_m);

    Pose2D sim_pose;
    sim_pose.x = input.current_pose.x - s * heading_cos;
    sim_pose.y = input.current_pose.y - s * heading_sin;
    sim_pose.yaw = input.current_pose.yaw;

    std::string bounds_err;
    if (!collectPoseCells(
        cm, grid, sim_pose, input.footprint_in_base,
        fp_min_x, fp_max_x, fp_min_y, fp_max_y, config_,
        fp_cells, guard_cells, bounds_err))
    {
      result.failure_code = BackupFailureCode::COSTMAP_INVALID;
      result.detail = bounds_err;
      return result;
    }
    ++result.samples_checked;

    // 6a. Rear stopping zone and rear corners MUST be completely clear of obstacles AND unknown space
    for (const auto & gc : guard_cells) {
      if (gc.cost == kNoInformation) {
        result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
        result.detail = "unknown_space_in_rear_corner_or_stop_margin";
        return result;
      }
      if (gc.cost >= config_.lethal_cost_threshold) {
        result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
        result.detail = "rear_corner_or_stop_margin_occupied";
        return result;
      }
    }

    // 6b. Inspect every cell touched by or inside the robot footprint at P(s)
    size_t step_overlap_count = 0;
    double step_min_x_rel = std::numeric_limits<double>::infinity();
    std::vector<geometry_msgs::msg::Point> step_world_pts;

    for (const auto & fc : fp_cells) {
      if (fc.cost == kNoInformation) {
        result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
        result.detail = "unknown_space_in_footprint_trajectory";
        return result;
      }
      if (fc.cost >= config_.lethal_cost_threshold) {
        // Must strictly belong to the shallow front zone in base_footprint frame
        if (fc.x_rel < config_.front_overlap_min_x) {
          std::ostringstream oss;
          oss << "non_front_collision_cell(x_rel=" << fc.x_rel << ",s=" << s << ")";
          result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
          result.detail = oss.str();
          return result;
        }

        if (is_start) {
          if (step == 0) {
            geometry_msgs::msg::Point pt;
            pt.x = fc.wx;
            pt.y = fc.wy;
            pt.z = 0.0;
            step_world_pts.push_back(pt);
          } else {
            if (candidate_front_world_pts.empty() ||
              !isWithinWhitelist(fc.wx, fc.wy, candidate_front_world_pts, config_.whitelist_match_radius))
            {
              std::ostringstream oss;
              oss << "new_collision_cell_along_start_trajectory(s=" << s << ")";
              result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
              result.detail = oss.str();
              return result;
            }
          }
        } else {
          if (!session.front_overlap_accepted || session.front_overlap_cleared ||
            !isWithinWhitelist(fc.wx, fc.wy, session.initial_front_world_pts, config_.whitelist_match_radius))
          {
            std::ostringstream oss;
            oss << "new_or_encroaching_collision_during_backup(s=" << s << ")";
            result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
            result.detail = oss.str();
            return result;
          }
        }

        ++step_overlap_count;
        step_min_x_rel = std::min(step_min_x_rel, fc.x_rel);
      }
    }

    if (step == 0) {
      if (is_start) {
        if (step_overlap_count > 0) {
          const double overlap_ratio = static_cast<double>(step_overlap_count) /
            static_cast<double>(std::max<size_t>(1, fp_cells.size()));
          init_penetration = fp_max_x - step_min_x_rel;
          if (init_penetration > config_.max_front_overlap_depth ||
            overlap_ratio > config_.max_front_overlap_ratio)
          {
            std::ostringstream oss;
            oss << "front_overlap_too_deep_or_wide(pen=" << init_penetration
                << ",ratio=" << overlap_ratio << ")";
            result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
            result.detail = oss.str();
            return result;
          }
          init_overlap_count = step_overlap_count;
          init_min_x_rel = step_min_x_rel;
          candidate_front_world_pts = std::move(step_world_pts);
        }
      } else if (session.front_overlap_accepted && !session.front_overlap_cleared) {
        if (step_overlap_count == 0) {
          ++session.consecutive_clear_cycles;
          if (session.consecutive_clear_cycles >= 2) {
            session.front_overlap_cleared = true;
          }
        } else {
          session.consecutive_clear_cycles = 0;
          if (step_overlap_count > session.initial_front_overlap_count ||
            step_min_x_rel < (session.initial_min_front_x_rel - 0.03))
          {
            result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
            result.detail = "front_overlap_expanding_or_moving_closer";
            return result;
          }
        }
      }
      prev_overlap_count = step_overlap_count;
    } else {
      // Backing away must not exceed initial overlap or grow along trajectory
      const size_t ceiling_overlap = is_start ?
        init_overlap_count : session.initial_front_overlap_count;
      if (step_overlap_count > ceiling_overlap ||
        (prev_overlap_count > 0 && step_overlap_count > prev_overlap_count && step_overlap_count > 1))
      {
        result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
        result.detail = "overlap_not_receding_along_trajectory";
        return result;
      }
      prev_overlap_count = step_overlap_count;
    }

    // At final backup pose (when backing up meaningful distance), all pre-existing overlap must clear
    if (step == num_steps && input.remaining_dist > (res * 0.5) && step_overlap_count > 0) {
      result.failure_code = BackupFailureCode::TRAJECTORY_BLOCKED;
      result.detail = "backup_does_not_clear_front_overlap";
      return result;
    }
  }

  if (is_start) {
    session.active = true;
    session.front_overlap_accepted = (init_overlap_count > 0);
    session.front_overlap_cleared = false;
    session.initial_front_overlap_count = init_overlap_count;
    session.initial_front_overlap_penetration = init_penetration;
    session.initial_min_front_x_rel = init_min_x_rel;
    session.initial_front_world_pts = std::move(candidate_front_world_pts);
    session.consecutive_clear_cycles = 0;
  }

  result.safe = true;
  result.failure_code = BackupFailureCode::NONE;
  result.front_overlap_accepted = session.front_overlap_accepted;
  result.front_overlap_count = session.initial_front_overlap_count;
  result.front_overlap_penetration = session.initial_front_overlap_penetration;
  result.detail = session.front_overlap_accepted ?
    "safe_with_receding_front_overlap" : "safe_clear_trajectory";
  return result;
}

}  // namespace nav2_custom_plugins
