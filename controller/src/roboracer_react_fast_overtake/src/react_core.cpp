#include "roboracer_react_fast_overtake/react_core.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace roboracer_react_fast_overtake {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double smoothStep(double value) {
  const double t = clamp(value, 0.0, 1.0);
  return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    while (!field.empty() &&
           std::isspace(static_cast<unsigned char>(field.front()))) {
      field.erase(field.begin());
    }
    while (!field.empty() &&
           std::isspace(static_cast<unsigned char>(field.back()))) {
      field.pop_back();
    }
    fields.push_back(field);
  }
  return fields;
}

std::int64_t cellKey(int q, int ey) {
  const auto hi = static_cast<std::uint64_t>(static_cast<std::uint32_t>(q));
  const auto lo = static_cast<std::uint64_t>(static_cast<std::uint32_t>(ey));
  return static_cast<std::int64_t>((hi << 32U) | lo);
}

std::pair<int, int> decodeCell(std::int64_t key) {
  const auto raw = static_cast<std::uint64_t>(key);
  return {static_cast<std::int32_t>(raw >> 32U),
          static_cast<std::int32_t>(raw & 0xffffffffU)};
}

double maximumCurvature(const std::vector<double>& x,
                        const std::vector<double>& y) {
  double result = 0.0;
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
      result = std::max(
          result, std::abs(2.0 * (ax * by - ay * bx) / denominator));
    }
  }
  return result;
}

CandidatePath rejected(const std::string& side, const std::string& reason) {
  CandidatePath path;
  path.side = side;
  path.reason = reason;
  path.minimum_clearance = -kInf;
  path.maximum_curvature = kInf;
  return path;
}

CandidatePath buildPassCandidate(
    const std::string& side,
    const std::vector<CorridorSample>& corridor,
    double ego_ey,
    const OccupancyComponent& primary,
    const std::vector<OccupancyComponent>& occupancy,
    const ReactConfig& config) {
  const double longitudinal_padding =
      0.5 * config.ego_length + config.longitudinal_clearance;
  const double entry_available = primary.q_min - longitudinal_padding;
  if (entry_available < config.minimum_lane_change_length) {
    return rejected(side, "insufficient_reaction_distance");
  }
  const double entry_end =
      std::min(config.nominal_lane_change_length, entry_available);
  const double lateral_padding =
      0.5 * config.ego_width + config.lateral_clearance;
  const double target_offset = side == "left"
      ? primary.ey_max + lateral_padding + config.candidate_occupancy_buffer
      : primary.ey_min - lateral_padding - config.candidate_occupancy_buffer;
  const double hold_end = primary.q_max + longitudinal_padding +
                          config.pass_hold_distance;
  const double return_end = hold_end + config.return_length;
  if (return_end > config.maximum_candidate_length) {
    return rejected(side, "candidate_horizon_too_short");
  }

  CandidatePath path;
  path.side = side;
  path.target_offset = target_offset;
  path.minimum_clearance = kInf;
  double maximum_track_curvature = 0.0;
  for (const auto& sample : corridor) {
    if (sample.q > config.maximum_candidate_length + 1e-9) break;
    double offset = 0.0;
    if (sample.q <= entry_end) {
      offset = ego_ey + smoothStep(sample.q / entry_end) *
                            (target_offset - ego_ey);
    } else if (sample.q <= hold_end) {
      offset = target_offset;
    } else if (sample.q <= return_end) {
      offset = target_offset *
          (1.0 - smoothStep((sample.q - hold_end) / config.return_length));
    }
    const double left_limit = sample.width_left - 0.5 * config.ego_width -
                              config.boundary_margin;
    const double right_limit = -sample.width_right + 0.5 * config.ego_width +
                               config.boundary_margin;
    path.minimum_clearance = std::min(
        path.minimum_clearance,
        std::min(left_limit - offset, offset - right_limit));
    if (sample.q <= return_end) {
      maximum_track_curvature = std::max(
          maximum_track_curvature, std::abs(sample.curvature));
    }
    const double nx = -std::sin(sample.yaw);
    const double ny = std::cos(sample.yaw);
    path.q.push_back(sample.q);
    path.x.push_back(sample.x + offset * nx);
    path.y.push_back(sample.y + offset * ny);
  }
  if (path.x.size() < 3) return rejected(side, "candidate_too_short");

  path.yaw.resize(path.x.size());
  for (std::size_t i = 0; i < path.x.size(); ++i) {
    const std::size_t lo = i == 0 ? 0 : i - 1;
    const std::size_t hi = std::min(i + 1, path.x.size() - 1);
    path.yaw[i] = std::atan2(path.y[hi] - path.y[lo],
                             path.x[hi] - path.x[lo]);
  }
  path.maximum_curvature = maximumCurvature(path.x, path.y);
  if (maximum_track_curvature > config.maximum_track_curvature_for_pass) {
    path.reason = "outside_reactive_pass_zone";
  } else if (path.minimum_clearance < config.minimum_boundary_clearance) {
    path.reason = "track_boundary_clearance";
  } else if (path.maximum_curvature > config.maximum_path_curvature) {
    path.reason = "path_curvature_limit";
  } else {
    const double half_length = 0.5 * config.ego_length;
    const double half_width = 0.5 * config.ego_width;
    path.reason = "feasible";
    for (const auto& obstacle : occupancy) {
      const double q_low = obstacle.q_min - half_length -
                           config.longitudinal_clearance;
      const double q_high = obstacle.q_max + half_length +
                            config.longitudinal_clearance;
      const double ey_low = obstacle.ey_min - half_width -
                            config.lateral_clearance;
      const double ey_high = obstacle.ey_max + half_width +
                             config.lateral_clearance;
      for (std::size_t i = 0; i < path.q.size(); ++i) {
        if (path.q[i] < q_low || path.q[i] > q_high) continue;
        const auto& sample = corridor[i];
        const double dx = path.x[i] - sample.x;
        const double dy = path.y[i] - sample.y;
        const double offset = -std::sin(sample.yaw) * dx +
                               std::cos(sample.yaw) * dy;
        if (offset > ey_low && offset < ey_high) {
          path.reason = "roi_occupancy_collision";
          break;
        }
      }
      if (path.reason != "feasible") break;
    }
  }
  path.feasible = path.reason == "feasible";
  return path;
}

}  // namespace

double wrapAngle(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

double clamp(double value, double minimum, double maximum) {
  return std::max(minimum, std::min(value, maximum));
}

PeriodicTrack::PeriodicTrack(const std::string& csv_path) {
  std::ifstream input(csv_path);
  if (!input) throw std::runtime_error("cannot open raceline: " + csv_path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty raceline");
  const auto header = splitCsv(line);
  std::unordered_map<std::string, std::size_t> columns;
  for (std::size_t i = 0; i < header.size(); ++i) columns[header[i]] = i;
  const char* required[] = {"s_m", "x_m", "y_m", "psi_rad",
                            "kappa_radpm", "vx_mps", "w_tr_right_m",
                            "w_tr_left_m"};
  for (const auto* name : required) {
    if (columns.count(name) == 0U) {
      throw std::runtime_error(std::string("raceline missing column ") + name);
    }
  }
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    auto value = [&](const char* name) {
      const auto index = columns.at(name);
      if (index >= fields.size()) throw std::runtime_error("short raceline row");
      return std::stod(fields[index]);
    };
    TrackSample sample;
    sample.s = value("s_m");
    sample.x = value("x_m");
    sample.y = value("y_m");
    sample.yaw = value("psi_rad");
    sample.curvature = value("kappa_radpm");
    sample.speed = value("vx_mps");
    sample.width_right = value("w_tr_right_m");
    sample.width_left = value("w_tr_left_m");
    if (!samples_.empty() && sample.s <= samples_.back().s) {
      throw std::runtime_error("raceline progress is not strictly increasing");
    }
    samples_.push_back(sample);
  }
  if (samples_.size() < 3) throw std::runtime_error("raceline is too short");
  length_ = samples_.back().s +
      std::hypot(samples_.front().x - samples_.back().x,
                 samples_.front().y - samples_.back().y);
  if (!(length_ > samples_.back().s)) {
    throw std::runtime_error("invalid raceline loop closure");
  }
}

double PeriodicTrack::wrapS(double s) const {
  double wrapped = std::fmod(s, length_);
  if (wrapped < 0.0) wrapped += length_;
  return wrapped;
}

TrackSample PeriodicTrack::sample(double s) const {
  const double wrapped = wrapS(s);
  auto upper = std::upper_bound(
      samples_.begin(), samples_.end(), wrapped,
      [](double value, const TrackSample& point) { return value < point.s; });
  std::size_t lo = 0;
  std::size_t hi = 0;
  double s0 = 0.0;
  double s1 = 0.0;
  if (upper == samples_.begin()) {
    lo = samples_.size() - 1;
    hi = 0;
    s0 = samples_[lo].s - length_;
    s1 = samples_[hi].s;
  } else if (upper == samples_.end()) {
    lo = samples_.size() - 1;
    hi = 0;
    s0 = samples_[lo].s;
    s1 = length_;
  } else {
    hi = static_cast<std::size_t>(upper - samples_.begin());
    lo = hi - 1;
    s0 = samples_[lo].s;
    s1 = samples_[hi].s;
  }
  const double query = upper == samples_.begin() ? wrapped - length_ : wrapped;
  const double ratio = clamp((query - s0) / (s1 - s0), 0.0, 1.0);
  const auto& a = samples_[lo];
  const auto& b = samples_[hi];
  TrackSample result;
  result.s = wrapped;
  result.x = a.x + ratio * (b.x - a.x);
  result.y = a.y + ratio * (b.y - a.y);
  result.yaw = wrapAngle(a.yaw + ratio * wrapAngle(b.yaw - a.yaw));
  result.curvature = a.curvature + ratio * (b.curvature - a.curvature);
  result.speed = a.speed + ratio * (b.speed - a.speed);
  result.width_right = a.width_right + ratio * (b.width_right - a.width_right);
  result.width_left = a.width_left + ratio * (b.width_left - a.width_left);
  return result;
}

TrackProjection PeriodicTrack::project(double x, double y) const {
  TrackProjection best{0.0, 0.0, kInf};
  for (std::size_t i = 0; i < samples_.size(); ++i) {
    const std::size_t next = (i + 1) % samples_.size();
    const double dx = samples_[next].x - samples_[i].x;
    const double dy = samples_[next].y - samples_[i].y;
    const double norm2 = dx * dx + dy * dy;
    if (norm2 < 1e-12) continue;
    const double ratio = clamp(
        ((x - samples_[i].x) * dx + (y - samples_[i].y) * dy) / norm2,
        0.0, 1.0);
    const double px = samples_[i].x + ratio * dx;
    const double py = samples_[i].y + ratio * dy;
    const double distance = std::hypot(x - px, y - py);
    if (distance >= best.distance) continue;
    const double next_s = next == 0 ? length_ : samples_[next].s;
    const double s = samples_[i].s + ratio * (next_s - samples_[i].s);
    const double yaw = wrapAngle(
        samples_[i].yaw + ratio *
        wrapAngle(samples_[next].yaw - samples_[i].yaw));
    best.s = wrapS(s);
    best.ey = -std::sin(yaw) * (x - px) + std::cos(yaw) * (y - py);
    best.distance = distance;
  }
  return best;
}

void ReactConfig::validate() const {
  if (!(minimum_q >= 0.0 && maximum_q > minimum_q &&
        trigger_distance > minimum_q && trigger_distance <= maximum_q &&
        boundary_strip >= 0.0 && voxel_size > 0.0 &&
        minimum_component_points > 0 && ego_width > 0.0 &&
        ego_length > 0.0 && lateral_clearance >= 0.0 &&
        candidate_occupancy_buffer >= 0.0 &&
        longitudinal_clearance >= 0.0 && boundary_margin >= 0.0 &&
        minimum_boundary_clearance >= 0.0 &&
        minimum_lane_change_length > 0.0 &&
        nominal_lane_change_length >= minimum_lane_change_length &&
        pass_hold_distance >= 0.0 && return_length > 0.0 &&
        maximum_candidate_length > return_length &&
        maximum_path_curvature > 0.0 &&
        maximum_track_curvature_for_pass > 0.0 &&
        (preferred_side == "left" || preferred_side == "right"))) {
    throw std::invalid_argument("invalid reactive overtake configuration");
  }
}

std::vector<OccupancyComponent> extractOccupancy(
    const std::vector<RoiPoint>& points,
    const std::vector<CorridorSample>& corridor,
    const ReactConfig& config) {
  config.validate();
  if (corridor.empty()) return {};
  struct Projected { double q; double ey; double z; };
  std::vector<Projected> projected;
  std::unordered_map<std::int64_t, std::vector<std::size_t>> cells;
  projected.reserve(points.size());
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) continue;
    const CorridorSample* nearest = nullptr;
    double nearest_distance2 = kInf;
    for (const auto& sample : corridor) {
      const double dx = point.x - sample.x;
      const double dy = point.y - sample.y;
      const double distance2 = dx * dx + dy * dy;
      if (distance2 < nearest_distance2) {
        nearest_distance2 = distance2;
        nearest = &sample;
      }
    }
    if (nearest == nullptr || nearest->q < config.minimum_q ||
        nearest->q > config.maximum_q) continue;
    const double dx = point.x - nearest->x;
    const double dy = point.y - nearest->y;
    const double ey = -std::sin(nearest->yaw) * dx +
                       std::cos(nearest->yaw) * dy;
    if (ey >= nearest->width_left - config.boundary_strip ||
        ey <= -nearest->width_right + config.boundary_strip) continue;
    const std::size_t index = projected.size();
    projected.push_back({nearest->q, ey, point.z});
    const int q_cell = static_cast<int>(std::floor(nearest->q / config.voxel_size));
    const int ey_cell = static_cast<int>(std::floor(ey / config.voxel_size));
    cells[cellKey(q_cell, ey_cell)].push_back(index);
  }

  std::vector<OccupancyComponent> result;
  std::unordered_set<std::int64_t> visited;
  for (const auto& entry : cells) {
    if (!visited.insert(entry.first).second) continue;
    std::queue<std::int64_t> pending;
    pending.push(entry.first);
    std::vector<std::size_t> component;
    while (!pending.empty()) {
      const auto key = pending.front();
      pending.pop();
      const auto found = cells.find(key);
      if (found == cells.end()) continue;
      component.insert(component.end(), found->second.begin(), found->second.end());
      const auto [q_cell, ey_cell] = decodeCell(key);
      for (int dq = -1; dq <= 1; ++dq) {
        for (int de = -1; de <= 1; ++de) {
          const auto neighbor = cellKey(q_cell + dq, ey_cell + de);
          if (cells.count(neighbor) != 0U && visited.insert(neighbor).second) {
            pending.push(neighbor);
          }
        }
      }
    }
    if (component.size() <
        static_cast<std::size_t>(config.minimum_component_points)) continue;
    OccupancyComponent obstacle;
    obstacle.q_min = obstacle.ey_min = obstacle.z_min = kInf;
    obstacle.q_max = obstacle.ey_max = obstacle.z_max = -kInf;
    double q_sum = 0.0;
    double ey_sum = 0.0;
    for (const auto index : component) {
      const auto& point = projected[index];
      q_sum += point.q;
      ey_sum += point.ey;
      obstacle.q_min = std::min(obstacle.q_min, point.q);
      obstacle.q_max = std::max(obstacle.q_max, point.q);
      obstacle.ey_min = std::min(obstacle.ey_min, point.ey);
      obstacle.ey_max = std::max(obstacle.ey_max, point.ey);
      obstacle.z_min = std::min(obstacle.z_min, point.z);
      obstacle.z_max = std::max(obstacle.z_max, point.z);
    }
    if (obstacle.z_max < config.minimum_obstacle_top_z) continue;
    obstacle.point_count = component.size();
    obstacle.q = q_sum / static_cast<double>(component.size());
    obstacle.ey = ey_sum / static_cast<double>(component.size());
    result.push_back(obstacle);
  }
  std::sort(result.begin(), result.end(),
            [](const auto& a, const auto& b) { return a.q_min < b.q_min; });
  return result;
}

ReactDecision planReactivePass(
    const std::vector<CorridorSample>& corridor,
    double ego_ey,
    const std::vector<OccupancyComponent>& occupancy,
    const ReactConfig& config) {
  config.validate();
  ReactDecision decision;
  decision.bumper_gap = kInf;
  const double lateral_padding =
      0.5 * config.ego_width + config.lateral_clearance;
  const OccupancyComponent* primary = nullptr;
  for (const auto& obstacle : occupancy) {
    if (obstacle.q_max < config.minimum_q ||
        obstacle.q_min > config.trigger_distance) continue;
    if (obstacle.ey_min - lateral_padding < 0.0 &&
        obstacle.ey_max + lateral_padding > 0.0) {
      if (primary == nullptr || obstacle.q_min < primary->q_min) {
        primary = &obstacle;
      }
    }
  }
  if (primary == nullptr) {
    decision.reason = "centerline_clear";
    return decision;
  }
  decision.centerline_blocked = true;
  decision.bumper_gap = primary->q_min - 0.5 * config.ego_length;
  decision.left = buildPassCandidate(
      "left", corridor, ego_ey, *primary, occupancy, config);
  decision.right = buildPassCandidate(
      "right", corridor, ego_ey, *primary, occupancy, config);
  if (decision.left.feasible && decision.right.feasible) {
    const double left_score = decision.left.minimum_clearance +
        (config.preferred_side == "left" ? 0.02 : 0.0);
    const double right_score = decision.right.minimum_clearance +
        (config.preferred_side == "right" ? 0.02 : 0.0);
    decision.selected_side = left_score >= right_score ? "left" : "right";
  } else if (decision.left.feasible) {
    decision.selected_side = "left";
  } else if (decision.right.feasible) {
    decision.selected_side = "right";
  }
  decision.reason = decision.selected_side == "none"
      ? "no_reactive_corridor" : "reactive_pass";
  return decision;
}

CandidatePath buildReturnPath(
    const std::vector<CorridorSample>& corridor,
    double ego_ey,
    double hold_distance,
    const ReactConfig& config) {
  config.validate();
  CandidatePath path;
  path.side = ego_ey >= 0.0 ? "left" : "right";
  path.target_offset = 0.0;
  path.minimum_clearance = kInf;
  const double return_end = hold_distance + config.return_length;
  for (const auto& sample : corridor) {
    if (sample.q > config.maximum_candidate_length + 1e-9) break;
    double offset = 0.0;
    if (sample.q <= hold_distance) {
      offset = ego_ey;
    } else if (sample.q <= return_end) {
      offset = ego_ey *
          (1.0 - smoothStep((sample.q - hold_distance) / config.return_length));
    }
    const double left_limit = sample.width_left - 0.5 * config.ego_width -
                              config.boundary_margin;
    const double right_limit = -sample.width_right + 0.5 * config.ego_width +
                               config.boundary_margin;
    path.minimum_clearance = std::min(
        path.minimum_clearance,
        std::min(left_limit - offset, offset - right_limit));
    const double nx = -std::sin(sample.yaw);
    const double ny = std::cos(sample.yaw);
    path.q.push_back(sample.q);
    path.x.push_back(sample.x + offset * nx);
    path.y.push_back(sample.y + offset * ny);
  }
  if (path.x.size() < 3) return rejected(path.side, "return_too_short");
  path.yaw.resize(path.x.size());
  for (std::size_t i = 0; i < path.x.size(); ++i) {
    const std::size_t lo = i == 0 ? 0 : i - 1;
    const std::size_t hi = std::min(i + 1, path.x.size() - 1);
    path.yaw[i] = std::atan2(path.y[hi] - path.y[lo],
                             path.x[hi] - path.x[lo]);
  }
  path.maximum_curvature = maximumCurvature(path.x, path.y);
  if (path.minimum_clearance < config.minimum_boundary_clearance) {
    path.reason = "return_boundary_clearance";
  } else if (path.maximum_curvature > config.maximum_path_curvature) {
    path.reason = "return_curvature_limit";
  } else {
    path.reason = "feasible";
    path.feasible = true;
  }
  return path;
}

double curvatureLimitedSpeedCap(
    const CandidatePath& path,
    double configured_speed_cap,
    double lateral_acceleration_limit,
    double minimum_speed_cap) {
  if (!std::isfinite(configured_speed_cap) || configured_speed_cap <= 0.0 ||
      !std::isfinite(lateral_acceleration_limit) ||
      lateral_acceleration_limit <= 0.0 ||
      !std::isfinite(minimum_speed_cap) || minimum_speed_cap < 0.0 ||
      minimum_speed_cap > configured_speed_cap) {
    throw std::invalid_argument("invalid curvature speed-limit configuration");
  }
  if (!std::isfinite(path.maximum_curvature) ||
      path.maximum_curvature <= 1e-6) {
    return configured_speed_cap;
  }
  const double curvature_cap =
      std::sqrt(lateral_acceleration_limit / path.maximum_curvature);
  return clamp(curvature_cap, minimum_speed_cap, configured_speed_cap);
}

}  // namespace roboracer_react_fast_overtake
