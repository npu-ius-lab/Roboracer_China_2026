#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace f1tenth_dynamic_mpcc {

struct TrackingQualitySpeedDecision {
  double speed_cap{};
  double scale{1.0};
  double lateral_scale{1.0};
  double heading_scale{1.0};
  double margin_scale{1.0};
};

// StableV5-only policy: extra speed above the ordinary-zone ceiling is earned
// continuously as lateral error, heading error and buffered corridor margin
// improve. Keeping this policy outside core.hpp preserves older frozen nodes.
class TrackingQualitySpeedGate {
 public:
  explicit TrackingQualitySpeedGate(const YAML::Node& config) {
    if (!config) return;
    enabled_ = value(config, "enabled", false);
    base_speed_ = value(config, "base_speed_mps", 3.2);
    lateral_full_ = value(config, "lateral_full_speed_m", 0.10);
    lateral_base_ = value(config, "lateral_base_speed_m", 0.30);
    heading_full_ = value(config, "heading_full_speed_rad", 0.04);
    heading_base_ = value(config, "heading_base_speed_rad", 0.12);
    margin_base_ = value(config, "margin_base_speed_m", 0.12);
    margin_full_ = value(config, "margin_full_speed_m", 0.30);
    if (!std::isfinite(base_speed_) || base_speed_ <= 0.0 ||
        !std::isfinite(lateral_full_) || lateral_full_ < 0.0 ||
        !std::isfinite(lateral_base_) || lateral_base_ <= lateral_full_ ||
        !std::isfinite(heading_full_) || heading_full_ < 0.0 ||
        !std::isfinite(heading_base_) || heading_base_ <= heading_full_ ||
        !std::isfinite(margin_base_) || !std::isfinite(margin_full_) ||
        margin_full_ <= margin_base_) {
      throw std::invalid_argument("invalid StableV5 high-speed tracking gate parameters");
    }
  }

  TrackingQualitySpeedDecision limit(double requested_speed_cap,
                                     double lateral_error,
                                     double heading_error,
                                     double current_control_margin) const {
    if (!std::isfinite(requested_speed_cap) || requested_speed_cap < 0.0 ||
        !std::isfinite(lateral_error) || !std::isfinite(heading_error) ||
        !std::isfinite(current_control_margin)) {
      throw std::invalid_argument("invalid StableV5 high-speed tracking gate sample");
    }
    TrackingQualitySpeedDecision out;
    out.speed_cap = requested_speed_cap;
    if (!enabled_ || requested_speed_cap <= base_speed_) return out;
    const auto decreasing = [](double magnitude, double full, double base) {
      return std::clamp((base - magnitude) / (base - full), 0.0, 1.0);
    };
    out.lateral_scale = decreasing(
        std::abs(lateral_error), lateral_full_, lateral_base_);
    out.heading_scale = decreasing(
        std::abs(heading_error), heading_full_, heading_base_);
    out.margin_scale = std::clamp(
        (current_control_margin - margin_base_) /
            (margin_full_ - margin_base_),
        0.0, 1.0);
    out.scale = std::min(
        out.lateral_scale, std::min(out.heading_scale, out.margin_scale));
    out.speed_cap = base_speed_ +
        out.scale * (requested_speed_cap - base_speed_);
    return out;
  }

  bool enabled() const { return enabled_; }

 private:
  template <typename T>
  static T value(const YAML::Node& node, const char* key, const T& fallback) {
    return node[key] ? node[key].as<T>() : fallback;
  }

  bool enabled_{false};
  double base_speed_{3.2};
  double lateral_full_{0.10}, lateral_base_{0.30};
  double heading_full_{0.04}, heading_base_{0.12};
  double margin_base_{0.12}, margin_full_{0.30};
};

}  // namespace f1tenth_dynamic_mpcc
