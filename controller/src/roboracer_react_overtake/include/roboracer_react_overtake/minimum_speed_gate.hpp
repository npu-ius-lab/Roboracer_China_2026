#pragma once

#include <string>

namespace roboracer_react_overtake {

struct MinimumSpeedGateInput {
  std::string planner_state;
  bool planner_healthy{false};
  bool planner_state_fresh{false};
  bool planner_health_fresh{false};
  bool require_supervisor_running{true};
  bool supervisor_fresh{false};
  bool supervisor_running{false};
  bool controller_enabled{false};
};

struct MinimumSpeedGateResult {
  double speed{};
  bool floor_applied{false};
  std::string reason;
};

MinimumSpeedGateResult applyMinimumSpeedGate(
    double raw_speed, double minimum_speed,
    const MinimumSpeedGateInput& input);

}  // namespace roboracer_react_overtake
