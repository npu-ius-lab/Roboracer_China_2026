#include "f1tenth_dynamic_mpcc/v5_jubu_roi.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace jubu = f1tenth_dynamic_mpcc::v5_jubu;

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

std::vector<jubu::CorridorSample> straightCorridor(double half_width = 0.8) {
  std::vector<jubu::CorridorSample> corridor;
  for (int i = 0; i <= 120; ++i) {
    const double q = 0.1 * static_cast<double>(i);
    corridor.push_back({q, q, q, 0.0, 0.0, half_width, half_width});
  }
  return corridor;
}

void appendBox(std::vector<jubu::RoiPoint>& points, double center_x,
               double center_y, double length, double width) {
  for (int ix = 0; ix < 7; ++ix) {
    for (int iy = 0; iy < 7; ++iy) {
      points.push_back({center_x - 0.5 * length + length * ix / 6.0,
                        center_y - 0.5 * width + width * iy / 6.0,
                        0.06 + 0.02 * ((ix + iy) % 4)});
    }
  }
}

}  // namespace

int main() {
  try {
    const auto corridor = straightCorridor();
    std::vector<jubu::RoiPoint> cloud;
    appendBox(cloud, 3.0, 0.0, 0.35, 0.28);
    // Dense boundary returns must not become obstacles.
    for (int i = 0; i < 80; ++i) {
      cloud.push_back({0.5 + 0.07 * i, 0.77, 0.15});
      cloud.push_back({0.5 + 0.07 * i, -0.77, 0.15});
    }
    // Sparse interior noise must not pass the component support gate.
    cloud.push_back({1.2, 0.1, 0.2});
    cloud.push_back({5.1, -0.2, 0.2});

    jubu::RoiExtractionConfig extraction;
    const auto obstacles = jubu::extractRoiObstacles(cloud, corridor, extraction);
    require(obstacles.size() == 1, "expected one interior obstacle");
    require(std::abs(obstacles.front().q - 3.0) < 0.15,
            "obstacle longitudinal location is wrong");
    require(std::abs(obstacles.front().ey) < 0.08,
            "obstacle lateral location is wrong");

    jubu::CandidateConfig planning;
    const auto decision =
        jubu::planAvoidance(corridor, 0.0, 2.0, 1.0, obstacles, planning);
    require(decision.relevant, "obstacle should be relevant");
    require(decision.risk, "obstacle should trigger avoidance");
    require(decision.left.feasible, decision.left.reason.c_str());
    require(decision.right.feasible, decision.right.reason.c_str());
    require(decision.selected_side == "left", "left preference was not applied");
    require(std::abs(decision.left.ey.back()) < 1e-6,
            "candidate must return to the raceline");

    auto blocked = obstacles;
    blocked.push_back({4.0, 0.43, 4.0, 0.43, 0.35, 0.28,
                       0.05, 0.20, 30});
    const auto blocked_decision =
        jubu::planAvoidance(corridor, 0.0, 2.0, 1.0, blocked, planning);
    require(!blocked_decision.left.feasible,
            "left candidate must reject another ROI obstacle");
    require(blocked_decision.right.feasible,
            "right candidate should remain available");
    require(blocked_decision.selected_side == "right",
            "planner should choose the unblocked side");

    const auto clear = jubu::planAvoidance(
        corridor, 0.0, 2.0, 0.0, {}, planning);
    require(!clear.relevant && clear.reason == "roi_clear",
            "empty ROI must remain clear");

    const auto return_path = jubu::buildReturnPath(corridor, 0.32, planning);
    require(return_path.feasible, "return-to-raceline path must be feasible");
    require(std::abs(return_path.ey.front() - 0.32) < 1e-9,
            "return path must start at the current lateral offset");
    require(std::abs(return_path.ey.back()) < 1e-9,
            "return path must converge to the raceline");
  } catch (const std::exception& error) {
    std::cerr << "v5_jubu_roi_test failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "v5_jubu_roi_test passed\n";
  return 0;
}
