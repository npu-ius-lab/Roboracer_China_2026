#include "roboracer_react_fast_overtake/react_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

namespace react = roboracer_react_fast_overtake;

namespace {

std::vector<react::CorridorSample> corridor(double width = 0.65) {
  std::vector<react::CorridorSample> result;
  for (int i = 0; i <= 80; ++i) {
    const double q = 0.1 * i;
    result.push_back({q, q, q, 0.0, 0.0, 0.0, width, width});
  }
  return result;
}

std::vector<react::RoiPoint> box(double q, double ey) {
  std::vector<react::RoiPoint> result;
  for (int ix = -2; ix <= 2; ++ix) {
    for (int iy = -2; iy <= 2; ++iy) {
      result.push_back({q + 0.04 * ix, ey + 0.04 * iy, 0.12});
    }
  }
  return result;
}

}  // namespace

TEST(ReactCore, EmptyCloudLeavesCenterlineClear) {
  react::ReactConfig config;
  const auto occupancy = react::extractOccupancy({}, corridor(), config);
  const auto decision = react::planReactivePass(corridor(), 0.0, occupancy, config);
  EXPECT_TRUE(occupancy.empty());
  EXPECT_FALSE(decision.centerline_blocked);
  EXPECT_EQ(decision.selected_side, "none");
}

TEST(ReactCore, EveryCurrentRoiComponentIsHandledWithoutMotionInput) {
  react::ReactConfig config;
  const auto occupancy = react::extractOccupancy(box(2.5, 0.0), corridor(), config);
  ASSERT_EQ(occupancy.size(), 1U);
  const auto decision = react::planReactivePass(corridor(), 0.0, occupancy, config);
  EXPECT_TRUE(decision.centerline_blocked);
  EXPECT_NE(decision.selected_side, "none");
  EXPECT_TRUE(decision.left.feasible || decision.right.feasible);
}

TEST(ReactCore, CandidateKeepsExtraDistanceFromVoxelEnvelope) {
  react::ReactConfig config;
  const auto occupancy = react::extractOccupancy(box(2.5, 0.0), corridor(), config);
  ASSERT_EQ(occupancy.size(), 1U);
  const auto decision = react::planReactivePass(corridor(), 0.0, occupancy, config);
  ASSERT_TRUE(decision.left.feasible) << decision.left.reason;
  const double required = occupancy.front().ey_max + 0.5 * config.ego_width +
                          config.lateral_clearance +
                          config.candidate_occupancy_buffer;
  EXPECT_NEAR(decision.left.target_offset, required, 1e-9);
}

TEST(ReactCore, BoundaryStripReturnsAreNotDeclaredObstacles) {
  react::ReactConfig config;
  const auto occupancy = react::extractOccupancy(box(2.5, 0.66), corridor(), config);
  EXPECT_TRUE(occupancy.empty());
}

TEST(ReactCore, NarrowTrackProducesSafeBlockedDecision) {
  react::ReactConfig config;
  const auto narrow = corridor(0.34);
  const auto occupancy = react::extractOccupancy(box(2.5, 0.0), narrow, config);
  ASSERT_EQ(occupancy.size(), 1U);
  const auto decision = react::planReactivePass(narrow, 0.0, occupancy, config);
  EXPECT_TRUE(decision.centerline_blocked);
  EXPECT_EQ(decision.selected_side, "none");
}

TEST(ReactCore, ReturnPathRejoinsGlobalCenterline) {
  react::ReactConfig config;
  const auto path = react::buildReturnPath(corridor(), 0.30, 0.65, config);
  ASSERT_TRUE(path.feasible) << path.reason;
  ASSERT_FALSE(path.x.empty());
  EXPECT_NEAR(path.y.front(), 0.30, 1e-6);
  EXPECT_NEAR(path.y.back(), 0.0, 1e-6);
}

TEST(ReactCore, StraightLocalPathKeepsConfiguredSpeedCap) {
  react::CandidatePath path;
  path.maximum_curvature = 0.0;
  EXPECT_DOUBLE_EQ(
      react::curvatureLimitedSpeedCap(path, 3.2, 3.0, 1.0), 3.2);
}

TEST(ReactCore, CurvedLocalPathUsesLateralAccelerationLimit) {
  react::CandidatePath path;
  path.maximum_curvature = 0.75;
  EXPECT_NEAR(
      react::curvatureLimitedSpeedCap(path, 3.2, 3.0, 1.0), 2.0, 1e-9);
}

TEST(ReactCore, CurvatureCapNeverViolatesReactMinimumSpeed) {
  react::CandidatePath path;
  path.maximum_curvature = 6.0;
  EXPECT_DOUBLE_EQ(
      react::curvatureLimitedSpeedCap(path, 3.2, 3.0, 1.0), 1.0);
}

TEST(ReactCore, RacelineLoaderAcceptsCrLfFiles) {
  const std::string path = "/tmp/roboracer_react_fast_crlf_track.csv";
  {
    std::ofstream output(path, std::ios::binary);
    output << "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,w_tr_right_m,w_tr_left_m\r\n"
           << "0,0,0,0,0,2,0.6,0.6\r\n"
           << "1,1,0,0,0,2,0.6,0.6\r\n"
           << "2,1,1,1.57079632679,0,2,0.6,0.6\r\n";
  }
  EXPECT_NO_THROW({
    react::PeriodicTrack track(path);
    EXPECT_GT(track.length(), 2.0);
    EXPECT_NEAR(track.project(0.5, 0.1).s, 0.5, 0.15);
  });
  std::remove(path.c_str());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
