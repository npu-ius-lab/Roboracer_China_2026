#include "f1tenth_dynamic_mpcc/v5_jubu_roi.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace f1tenth_dynamic_mpcc::v5_jubu {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double wrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double smootherstep(double value) {
  const double t = clamp(value, 0.0, 1.0);
  return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

std::int64_t cellKey(int x, int y) {
  const auto high = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
  const auto low = static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
  return static_cast<std::int64_t>((high << 32U) | low);
}

std::pair<int, int> decodeCell(std::int64_t key) {
  const auto raw = static_cast<std::uint64_t>(key);
  return {static_cast<std::int32_t>(raw >> 32U),
          static_cast<std::int32_t>(raw & 0xffffffffU)};
}

struct ProjectedPoint {
  RoiPoint point;
  double q{};
  double ey{};
};

CandidatePath emptyCandidate(const std::string& side,
                             const std::string& reason) {
  CandidatePath candidate;
  candidate.side = side;
  candidate.reason = reason;
  candidate.minimum_boundary_clearance = -kInfinity;
  candidate.maximum_curvature = kInfinity;
  candidate.score = -kInfinity;
  return candidate;
}

double pathMaximumCurvature(const std::vector<double>& x,
                            const std::vector<double>& y) {
  double maximum = 0.0;
  for (std::size_t i = 1; i + 1 < x.size(); ++i) {
    const double ax = x[i] - x[i - 1];
    const double ay = y[i] - y[i - 1];
    const double bx = x[i + 1] - x[i];
    const double by = y[i + 1] - y[i];
    const double cx = x[i + 1] - x[i - 1];
    const double cy = y[i + 1] - y[i - 1];
    const double denominator =
        std::hypot(ax, ay) * std::hypot(bx, by) * std::hypot(cx, cy);
    if (denominator > 1e-9) {
      maximum = std::max(maximum,
                         std::abs(2.0 * (ax * by - ay * bx) / denominator));
    }
  }
  return maximum;
}

CandidatePath buildCandidate(const std::string& side,
                             const std::vector<CorridorSample>& corridor,
                             double ego_ey,
                             double ego_speed,
                             double obstacle_speed,
                             const std::vector<RoiObstacle>& obstacles,
                             const CandidateConfig& config) {
  const RoiObstacle& primary = obstacles.front();
  const double observed_width =
      std::max(config.default_obstacle_width, primary.width);
  const double observed_length =
      std::max(config.default_obstacle_length, primary.length);
  const double lateral_separation =
      0.5 * (config.ego_width + observed_width) + config.lateral_clearance;
  const double longitudinal_separation =
      0.5 * (config.ego_length + observed_length) +
      config.longitudinal_clearance;
  const double entry_end = std::min(
      config.nominal_lane_change_length,
      primary.q - longitudinal_separation);
  if (entry_end < config.minimum_lane_change_length) {
    return emptyCandidate(side, "insufficient_lane_change_distance");
  }

  const double plan_speed = clamp(
      std::max({ego_speed, obstacle_speed + config.pass_speed_advantage,
                config.minimum_plan_speed}),
      config.minimum_plan_speed, config.maximum_plan_speed);
  const double relative_speed = plan_speed - obstacle_speed;
  if (relative_speed < config.minimum_closing_speed) {
    return emptyCandidate(side, "insufficient_pass_speed_advantage");
  }
  const double relative_progress_ratio = relative_speed / plan_speed;
  const double hold_end =
      (primary.q + longitudinal_separation) / relative_progress_ratio +
      config.pass_hold_after_obstacle;
  const double return_end = hold_end + config.return_length;
  if (return_end > config.maximum_candidate_length) {
    return emptyCandidate(side, "pass_exceeds_candidate_horizon");
  }

  const double direction = side == "left" ? 1.0 : -1.0;
  const double target_offset = primary.ey + direction * lateral_separation;
  CandidatePath candidate;
  candidate.side = side;
  candidate.reason = "feasible";
  candidate.target_offset = target_offset;
  candidate.minimum_boundary_clearance = kInfinity;

  double maximum_track_curvature = 0.0;
  double previous_track_yaw = 0.0;
  double previous_q = 0.0;
  bool have_previous = false;
  for (const auto& sample : corridor) {
    if (sample.q > config.maximum_candidate_length + 1e-9) break;
    double offset = 0.0;
    if (sample.q <= entry_end) {
      offset = ego_ey +
               smootherstep(sample.q / entry_end) * (target_offset - ego_ey);
    } else if (sample.q <= hold_end) {
      offset = target_offset;
    } else if (sample.q <= return_end) {
      offset = target_offset *
               (1.0 - smootherstep((sample.q - hold_end) /
                                    (return_end - hold_end)));
    }
    const double nx = -std::sin(sample.yaw);
    const double ny = std::cos(sample.yaw);
    candidate.q.push_back(sample.q);
    candidate.s.push_back(sample.s);
    candidate.ey.push_back(offset);
    candidate.x.push_back(sample.x + offset * nx);
    candidate.y.push_back(sample.y + offset * ny);
    const double left_limit = sample.width_left - 0.5 * config.ego_width -
                              config.boundary_margin;
    const double right_limit = -sample.width_right + 0.5 * config.ego_width +
                               config.boundary_margin;
    candidate.minimum_boundary_clearance = std::min(
        candidate.minimum_boundary_clearance,
        std::min(left_limit - offset, offset - right_limit));
    if (sample.q <= return_end && have_previous) {
      const double ds = sample.q - previous_q;
      if (ds > 1e-6) {
        maximum_track_curvature = std::max(
            maximum_track_curvature,
            std::abs(wrapAngle(sample.yaw - previous_track_yaw) / ds));
      }
    }
    previous_track_yaw = sample.yaw;
    previous_q = sample.q;
    have_previous = true;
  }
  if (candidate.x.size() < 3) {
    return emptyCandidate(side, "candidate_too_short");
  }
  candidate.yaw.resize(candidate.x.size());
  for (std::size_t i = 0; i < candidate.x.size(); ++i) {
    const std::size_t lo = i == 0 ? 0 : i - 1;
    const std::size_t hi = std::min(i + 1, candidate.x.size() - 1);
    candidate.yaw[i] = std::atan2(candidate.y[hi] - candidate.y[lo],
                                  candidate.x[hi] - candidate.x[lo]);
  }
  candidate.maximum_curvature =
      pathMaximumCurvature(candidate.x, candidate.y);

  if (maximum_track_curvature > config.maximum_track_curvature_for_pass) {
    candidate.reason = "track_curvature_outside_pass_zone";
  } else if (candidate.minimum_boundary_clearance + 1e-9 <
             config.minimum_boundary_clearance) {
    candidate.reason = "track_corridor_violation";
  } else if (candidate.maximum_curvature > config.maximum_path_curvature) {
    candidate.reason = "path_curvature_limit";
  }

  if (candidate.reason == "feasible") {
    for (const auto& obstacle : obstacles) {
      const double obstacle_width =
          std::max(config.default_obstacle_width, obstacle.width);
      const double obstacle_length =
          std::max(config.default_obstacle_length, obstacle.length);
      const double required_lateral =
          0.5 * (config.ego_width + obstacle_width) +
          config.lateral_clearance;
      const double required_longitudinal =
          0.5 * (config.ego_length + obstacle_length) +
          config.longitudinal_clearance;
      for (std::size_t i = 0; i < candidate.q.size(); ++i) {
        if (std::abs(candidate.q[i] - obstacle.q) < required_longitudinal &&
            std::abs(candidate.ey[i] - obstacle.ey) < required_lateral) {
          candidate.reason = "roi_obstacle_collision";
          break;
        }
      }
      if (candidate.reason != "feasible") break;
    }
  }

  candidate.feasible = candidate.reason == "feasible";
  candidate.score = candidate.feasible
                        ? candidate.minimum_boundary_clearance -
                              0.02 * candidate.maximum_curvature -
                              0.01 * std::abs(target_offset - ego_ey) +
                              ((config.preferred_side == side) ? 0.02 : 0.0)
                        : -kInfinity;
  return candidate;
}

}  // namespace

void RoiExtractionConfig::validate() const {
  if (!(minimum_q >= 0.0 && maximum_q > minimum_q &&
        boundary_strip >= 0.0 && voxel_size > 0.0 &&
        minimum_component_points > 0 && maximum_component_length > 0.0 &&
        maximum_component_width > 0.0 &&
        std::isfinite(minimum_obstacle_top_z) &&
        minimum_centroid_boundary_clearance >= 0.0 &&
        fragment_merge_longitudinal_gap >= 0.0 &&
        fragment_merge_lateral_gap >= 0.0)) {
    throw std::invalid_argument("invalid V5 Jubu ROI extraction configuration");
  }
}

std::vector<RoiObstacle> extractRoiObstacles(
    const std::vector<RoiPoint>& points,
    const std::vector<CorridorSample>& corridor,
    const RoiExtractionConfig& config) {
  config.validate();
  if (corridor.empty()) return {};

  std::vector<ProjectedPoint> projected;
  projected.reserve(points.size());
  std::unordered_map<std::int64_t, std::vector<std::size_t>> cells;
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    std::size_t nearest = 0;
    double nearest_distance2 = kInfinity;
    for (std::size_t i = 0; i < corridor.size(); ++i) {
      const double dx = point.x - corridor[i].x;
      const double dy = point.y - corridor[i].y;
      const double distance2 = dx * dx + dy * dy;
      if (distance2 < nearest_distance2) {
        nearest_distance2 = distance2;
        nearest = i;
      }
    }
    const auto& sample = corridor[nearest];
    if (sample.q < config.minimum_q || sample.q > config.maximum_q) continue;
    const double dx = point.x - sample.x;
    const double dy = point.y - sample.y;
    const double ey = -std::sin(sample.yaw) * dx +
                      std::cos(sample.yaw) * dy;
    if (ey > sample.width_left - config.boundary_strip ||
        ey < -sample.width_right + config.boundary_strip) {
      continue;
    }
    const std::size_t index = projected.size();
    projected.push_back({point, sample.q, ey});
    const int cell_x = static_cast<int>(std::floor(point.x / config.voxel_size));
    const int cell_y = static_cast<int>(std::floor(point.y / config.voxel_size));
    cells[cellKey(cell_x, cell_y)].push_back(index);
  }

  std::vector<RoiObstacle> obstacles;
  std::unordered_set<std::int64_t> visited;
  for (const auto& entry : cells) {
    if (visited.count(entry.first) != 0U) continue;
    std::queue<std::int64_t> pending;
    pending.push(entry.first);
    visited.insert(entry.first);
    std::vector<std::size_t> component;
    while (!pending.empty()) {
      const auto key = pending.front();
      pending.pop();
      const auto found = cells.find(key);
      if (found == cells.end()) continue;
      component.insert(component.end(), found->second.begin(), found->second.end());
      const auto [cx, cy] = decodeCell(key);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          const auto neighbor = cellKey(cx + dx, cy + dy);
          if (cells.count(neighbor) != 0U && visited.insert(neighbor).second) {
            pending.push(neighbor);
          }
        }
      }
    }
    if (component.size() <
        static_cast<std::size_t>(config.minimum_component_points)) {
      continue;
    }
    double q_sum = 0.0;
    double ey_sum = 0.0;
    double x_sum = 0.0;
    double y_sum = 0.0;
    double q_min = kInfinity;
    double q_max = -kInfinity;
    double ey_min = kInfinity;
    double ey_max = -kInfinity;
    double z_min = kInfinity;
    double z_max = -kInfinity;
    for (const auto index : component) {
      const auto& sample = projected[index];
      q_sum += sample.q;
      ey_sum += sample.ey;
      x_sum += sample.point.x;
      y_sum += sample.point.y;
      q_min = std::min(q_min, sample.q);
      q_max = std::max(q_max, sample.q);
      ey_min = std::min(ey_min, sample.ey);
      ey_max = std::max(ey_max, sample.ey);
      z_min = std::min(z_min, sample.point.z);
      z_max = std::max(z_max, sample.point.z);
    }
    const double length = q_max - q_min + config.voxel_size;
    const double width = ey_max - ey_min + config.voxel_size;
    if (length > config.maximum_component_length ||
        width > config.maximum_component_width ||
        z_max < config.minimum_obstacle_top_z) {
      continue;
    }
    const double scale = 1.0 / static_cast<double>(component.size());
    const double mean_q = q_sum * scale;
    const double mean_ey = ey_sum * scale;
    const auto corridor_sample = std::min_element(
        corridor.begin(), corridor.end(),
        [mean_q](const CorridorSample& left, const CorridorSample& right) {
          return std::abs(left.q - mean_q) < std::abs(right.q - mean_q);
        });
    const double centroid_boundary_clearance = std::min(
        corridor_sample->width_left - mean_ey,
        corridor_sample->width_right + mean_ey);
    if (centroid_boundary_clearance + 1e-9 <
        config.minimum_centroid_boundary_clearance) {
      continue;
    }
    obstacles.push_back(
        {mean_q, mean_ey, x_sum * scale, y_sum * scale,
         length, width, z_min, z_max, component.size()});
  }
  std::sort(obstacles.begin(), obstacles.end(),
            [](const RoiObstacle& lhs, const RoiObstacle& rhs) {
              return lhs.q < rhs.q;
            });
  if (config.fragment_merge_longitudinal_gap > 0.0 &&
      config.fragment_merge_lateral_gap > 0.0 && obstacles.size() > 1U) {
    std::vector<RoiObstacle> merged;
    merged.reserve(obstacles.size());
    for (const auto& obstacle : obstacles) {
      if (merged.empty()) {
        merged.push_back(obstacle);
        continue;
      }
      auto& previous = merged.back();
      const double previous_back = previous.q + 0.5 * previous.length;
      const double current_front = obstacle.q - 0.5 * obstacle.length;
      const double q_front = std::min(
          previous.q - 0.5 * previous.length, current_front);
      const double q_back = std::max(
          previous_back, obstacle.q + 0.5 * obstacle.length);
      const double ey_right = std::min(
          previous.ey - 0.5 * previous.width,
          obstacle.ey - 0.5 * obstacle.width);
      const double ey_left = std::max(
          previous.ey + 0.5 * previous.width,
          obstacle.ey + 0.5 * obstacle.width);
      const double combined_length = q_back - q_front;
      const double combined_width = ey_left - ey_right;
      const bool same_vehicle_fragment =
          current_front - previous_back <=
              config.fragment_merge_longitudinal_gap &&
          std::abs(obstacle.ey - previous.ey) <=
              config.fragment_merge_lateral_gap &&
          combined_length <= config.maximum_component_length &&
          combined_width <= config.maximum_component_width;
      if (!same_vehicle_fragment) {
        merged.push_back(obstacle);
        continue;
      }
      const std::size_t total_points =
          previous.point_count + obstacle.point_count;
      const double previous_weight =
          static_cast<double>(previous.point_count) /
          static_cast<double>(total_points);
      const double obstacle_weight = 1.0 - previous_weight;
      previous.q = 0.5 * (q_front + q_back);
      previous.ey = previous_weight * previous.ey +
                    obstacle_weight * obstacle.ey;
      previous.local_x = previous_weight * previous.local_x +
                         obstacle_weight * obstacle.local_x;
      previous.local_y = previous_weight * previous.local_y +
                         obstacle_weight * obstacle.local_y;
      previous.length = combined_length;
      previous.width = combined_width;
      previous.z_min = std::min(previous.z_min, obstacle.z_min);
      previous.z_max = std::max(previous.z_max, obstacle.z_max);
      previous.point_count = total_points;
    }
    obstacles = std::move(merged);
  }
  return obstacles;
}

void CandidateConfig::validate() const {
  const bool valid =
      ego_width > 0.0 && ego_length > 0.0 &&
      default_obstacle_width > 0.0 && default_obstacle_length > 0.0 &&
      boundary_margin >= 0.0 && minimum_boundary_clearance >= 0.0 &&
      lateral_clearance >= 0.0 && longitudinal_clearance >= 0.0 &&
      detection_distance > 0.0 && trigger_distance > 0.0 &&
      trigger_ttc > 0.0 && minimum_closing_speed > 0.0 &&
      minimum_lane_change_length > 0.0 &&
      nominal_lane_change_length >= minimum_lane_change_length &&
      pass_hold_after_obstacle >= 0.0 && return_length > 0.0 &&
      minimum_plan_speed > 0.0 && maximum_plan_speed >= minimum_plan_speed &&
      pass_speed_advantage >= 0.0 && maximum_path_curvature > 0.0 &&
      maximum_track_curvature_for_pass > 0.0 &&
      maximum_candidate_length > 0.0 &&
      (preferred_side == "left" || preferred_side == "right" ||
       preferred_side == "none");
  if (!valid) {
    throw std::invalid_argument("invalid V5 Jubu candidate configuration");
  }
}

PlanningDecision planAvoidance(
    const std::vector<CorridorSample>& corridor,
    double ego_ey,
    double ego_speed,
    double obstacle_speed,
    const std::vector<RoiObstacle>& obstacles,
    const CandidateConfig& config) {
  config.validate();
  PlanningDecision decision;
  decision.left = emptyCandidate("left", "no_obstacle");
  decision.right = emptyCandidate("right", "no_obstacle");
  decision.ttc = kInfinity;
  if (obstacles.empty()) {
    decision.reason = "roi_clear";
    return decision;
  }
  const auto& obstacle = obstacles.front();
  const double obstacle_length =
      std::max(config.default_obstacle_length, obstacle.length);
  decision.bumper_gap = obstacle.q -
                        0.5 * (config.ego_length + obstacle_length);
  decision.closing_speed = ego_speed - obstacle_speed;
  if (decision.closing_speed > config.minimum_closing_speed &&
      decision.bumper_gap > 0.0) {
    decision.ttc = decision.bumper_gap / decision.closing_speed;
  }
  decision.relevant = obstacle.q > 0.0 &&
                      obstacle.q <= config.detection_distance;
  decision.risk = decision.relevant &&
                  (decision.bumper_gap <= config.trigger_distance ||
                   decision.ttc <= config.trigger_ttc);
  if (!decision.relevant) {
    decision.reason = "obstacle_not_ahead";
    return decision;
  }
  decision.left = buildCandidate("left", corridor, ego_ey, ego_speed,
                                 obstacle_speed, obstacles, config);
  decision.right = buildCandidate("right", corridor, ego_ey, ego_speed,
                                  obstacle_speed, obstacles, config);
  if (!decision.risk) {
    decision.reason = "no_avoidance_trigger";
    return decision;
  }
  const CandidatePath* selected = nullptr;
  for (const auto* candidate : {&decision.left, &decision.right}) {
    if (candidate->feasible &&
        (selected == nullptr || candidate->score > selected->score)) {
      selected = candidate;
    }
  }
  if (selected == nullptr) {
    decision.reason = "no_feasible_corridor";
  } else {
    decision.selected_side = selected->side;
    decision.reason = "selected_" + selected->side;
  }
  return decision;
}

CandidatePath buildReturnPath(
    const std::vector<CorridorSample>& corridor,
    double ego_ey,
    const CandidateConfig& config,
    double hold_distance) {
  config.validate();
  CandidatePath candidate;
  candidate.side = "return";
  candidate.reason = "feasible";
  candidate.target_offset = 0.0;
  candidate.minimum_boundary_clearance = kInfinity;
  for (const auto& sample : corridor) {
    if (sample.q > config.maximum_candidate_length + 1e-9) break;
    const double progress =
        (sample.q - std::max(0.0, hold_distance)) / config.return_length;
    const double offset = ego_ey * (1.0 - smootherstep(progress));
    const double nx = -std::sin(sample.yaw);
    const double ny = std::cos(sample.yaw);
    candidate.q.push_back(sample.q);
    candidate.s.push_back(sample.s);
    candidate.ey.push_back(offset);
    candidate.x.push_back(sample.x + offset * nx);
    candidate.y.push_back(sample.y + offset * ny);
    const double left_limit = sample.width_left - 0.5 * config.ego_width -
                              config.boundary_margin;
    const double right_limit = -sample.width_right + 0.5 * config.ego_width +
                               config.boundary_margin;
    candidate.minimum_boundary_clearance = std::min(
        candidate.minimum_boundary_clearance,
        std::min(left_limit - offset, offset - right_limit));
  }
  if (candidate.x.size() < 3) {
    return emptyCandidate("return", "candidate_too_short");
  }
  candidate.yaw.resize(candidate.x.size());
  for (std::size_t i = 0; i < candidate.x.size(); ++i) {
    const std::size_t lo = i == 0 ? 0 : i - 1;
    const std::size_t hi = std::min(i + 1, candidate.x.size() - 1);
    candidate.yaw[i] = std::atan2(candidate.y[hi] - candidate.y[lo],
                                  candidate.x[hi] - candidate.x[lo]);
  }
  candidate.maximum_curvature =
      pathMaximumCurvature(candidate.x, candidate.y);
  if (candidate.minimum_boundary_clearance + 1e-9 <
      config.minimum_boundary_clearance) {
    candidate.reason = "track_corridor_violation";
  } else if (candidate.maximum_curvature > config.maximum_path_curvature) {
    candidate.reason = "path_curvature_limit";
  }
  candidate.feasible = candidate.reason == "feasible";
  candidate.score = candidate.feasible
                        ? candidate.minimum_boundary_clearance -
                              0.02 * candidate.maximum_curvature
                        : -kInfinity;
  return candidate;
}

}  // namespace f1tenth_dynamic_mpcc::v5_jubu
