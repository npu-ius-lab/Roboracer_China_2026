#include "roboracer_react_fast_overtake/minimum_speed_gate.hpp"

#include <algorithm>
#include <cmath>

namespace roboracer_react_fast_overtake {

MinimumSpeedGateResult applyMinimumSpeedGate(
    double raw_speed, double minimum_speed,
    const MinimumSpeedGateInput& input) {
  MinimumSpeedGateResult result{raw_speed, false, "raw_passthrough"};
  if (!std::isfinite(raw_speed)) {
    result.reason = "non_finite_raw_speed";
    return result;
  }
  if (!std::isfinite(minimum_speed) || minimum_speed < 0.0) {
    result.reason = "invalid_minimum_speed";
    return result;
  }
  if (!input.planner_state_fresh) {
    result.reason = "planner_state_stale";
    return result;
  }
  if (!input.planner_health_fresh) {
    result.reason = "planner_health_stale";
    return result;
  }
  if (!input.planner_healthy) {
    result.reason = "planner_unhealthy";
    return result;
  }
  const bool driving_state = input.planner_state == "GLOBAL" ||
                             input.planner_state == "FOLLOW" ||
                             input.planner_state == "PASS" ||
                             input.planner_state == "RETURN";
  if (!driving_state) {
    result.reason = "not_driving";
    return result;
  }
  if (input.require_supervisor_running) {
    if (!input.supervisor_fresh) {
      result.reason = "supervisor_stale";
      return result;
    }
    if (!input.supervisor_running) {
      result.reason = "supervisor_not_running";
      return result;
    }
    if (!input.controller_enabled) {
      result.reason = "controller_disabled";
      return result;
    }
  }
  if (raw_speed < 0.0) {
    result.reason = "reverse_passthrough";
    return result;
  }
  result.speed = std::max(raw_speed, minimum_speed);
  result.floor_applied = result.speed > raw_speed;
  result.reason = result.floor_applied ? "react_minimum_speed"
                                       : "raw_above_minimum";
  return result;
}

}  // namespace roboracer_react_fast_overtake
