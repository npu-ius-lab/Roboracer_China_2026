#include "roboracer_react_fast_overtake/minimum_speed_gate.hpp"

#include <gtest/gtest.h>

namespace react = roboracer_react_fast_overtake;

namespace {

react::MinimumSpeedGateInput runningState(const std::string& state = "FOLLOW") {
  react::MinimumSpeedGateInput input;
  input.planner_state = state;
  input.planner_healthy = true;
  input.planner_state_fresh = true;
  input.planner_health_fresh = true;
  input.require_supervisor_running = true;
  input.supervisor_fresh = true;
  input.supervisor_running = true;
  input.controller_enabled = true;
  return input;
}

}  // namespace

TEST(MinimumSpeedGate, FloorsHealthyRunningFollowCommand) {
  const auto result = react::applyMinimumSpeedGate(0.0, 1.0, runningState());
  EXPECT_TRUE(result.floor_applied);
  EXPECT_DOUBLE_EQ(result.speed, 1.0);
}

TEST(MinimumSpeedGate, LeavesSpeedAboveFloorUnchanged) {
  const auto result = react::applyMinimumSpeedGate(1.7, 1.0, runningState());
  EXPECT_FALSE(result.floor_applied);
  EXPECT_DOUBLE_EQ(result.speed, 1.7);
}

TEST(MinimumSpeedGate, FloorsEveryHealthyDrivingState) {
  for (const auto& state : {"GLOBAL", "FOLLOW", "PASS", "RETURN"}) {
    const auto result =
        react::applyMinimumSpeedGate(0.0, 1.0, runningState(state));
    EXPECT_TRUE(result.floor_applied) << state;
    EXPECT_DOUBLE_EQ(result.speed, 1.0) << state;
  }
}

TEST(MinimumSpeedGate, PreservesAbortStop) {
  const auto result =
      react::applyMinimumSpeedGate(0.0, 1.0, runningState("ABORT"));
  EXPECT_FALSE(result.floor_applied);
  EXPECT_DOUBLE_EQ(result.speed, 0.0);
}

TEST(MinimumSpeedGate, PreservesStopWhenPlannerIsUnhealthyOrStale) {
  auto input = runningState();
  input.planner_healthy = false;
  EXPECT_DOUBLE_EQ(react::applyMinimumSpeedGate(0.0, 1.0, input).speed, 0.0);
  input = runningState();
  input.planner_health_fresh = false;
  EXPECT_DOUBLE_EQ(react::applyMinimumSpeedGate(0.0, 1.0, input).speed, 0.0);
}

TEST(MinimumSpeedGate, PreservesStopOutsideRunningSupervisorState) {
  auto input = runningState();
  input.supervisor_running = false;
  EXPECT_DOUBLE_EQ(react::applyMinimumSpeedGate(0.0, 1.0, input).speed, 0.0);
  input = runningState();
  input.controller_enabled = false;
  EXPECT_DOUBLE_EQ(react::applyMinimumSpeedGate(0.0, 1.0, input).speed, 0.0);
  input = runningState();
  input.supervisor_fresh = false;
  EXPECT_DOUBLE_EQ(react::applyMinimumSpeedGate(0.0, 1.0, input).speed, 0.0);
}

TEST(MinimumSpeedGate, ShadowModeDoesNotRequireSupervisor) {
  auto input = runningState();
  input.require_supervisor_running = false;
  input.supervisor_running = false;
  input.controller_enabled = false;
  input.supervisor_fresh = false;
  EXPECT_DOUBLE_EQ(react::applyMinimumSpeedGate(0.2, 1.0, input).speed, 1.0);
}

TEST(MinimumSpeedGate, ReverseCommandIsNeverTurnedIntoForwardMotion) {
  const auto result = react::applyMinimumSpeedGate(-0.2, 1.0, runningState());
  EXPECT_FALSE(result.floor_applied);
  EXPECT_DOUBLE_EQ(result.speed, -0.2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
