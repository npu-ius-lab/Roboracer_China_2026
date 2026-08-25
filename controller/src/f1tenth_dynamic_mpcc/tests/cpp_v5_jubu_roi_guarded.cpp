#include "f1tenth_dynamic_mpcc/v5_jubu_roi.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace jubu = f1tenth_dynamic_mpcc::v5_jubu;

namespace {

std::vector<jubu::CorridorSample> straightCorridor() {
  std::vector<jubu::CorridorSample> result;
  for (int i = 0; i <= 75; ++i) {
    const double q = 0.08 * i;
    result.push_back({q, q, q, 0.0, 0.0, 0.60, 0.60});
  }
  return result;
}

void addPatch(std::vector<jubu::RoiPoint>& points, double x, double y,
              double z, int rows = 3, int columns = 3) {
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      points.push_back({x + 0.025 * column, y + 0.025 * row,
                        z + 0.01 * ((row + column) % 2)});
    }
  }
}

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

}  // namespace

int main() {
  const auto corridor = straightCorridor();
  jubu::RoiExtractionConfig guarded;
  guarded.minimum_q = 0.25;
  guarded.maximum_q = 6.0;
  guarded.boundary_strip = 0.10;
  guarded.voxel_size = 0.08;
  guarded.minimum_component_points = 8;
  guarded.maximum_component_length = 0.75;
  guarded.maximum_component_width = 0.55;
  guarded.minimum_obstacle_top_z = 0.005;
  guarded.minimum_centroid_boundary_clearance = 0.14;
  guarded.fragment_merge_longitudinal_gap = 0.20;
  guarded.fragment_merge_lateral_gap = 0.35;

  std::vector<jubu::RoiPoint> wall;
  addPatch(wall, 2.0, 0.50, 0.08, 3, 4);
  require(jubu::extractRoiObstacles(wall, corridor, guarded).empty(),
          "near-wall fragment was accepted as an opponent");

  std::vector<jubu::RoiPoint> opponent;
  addPatch(opponent, 2.0, -0.04, 0.10, 4, 4);
  const auto one = jubu::extractRoiObstacles(opponent, corridor, guarded);
  require(one.size() == 1U, "centered opponent was rejected");
  require(one.front().point_count >= 8U, "opponent point count is invalid");

  std::vector<jubu::RoiPoint> fragmented;
  addPatch(fragmented, 2.0, -0.05, 0.10, 3, 3);
  addPatch(fragmented, 2.30, -0.03, 0.11, 3, 3);
  auto unmerged = guarded;
  unmerged.fragment_merge_longitudinal_gap = 0.0;
  unmerged.fragment_merge_lateral_gap = 0.0;
  require(jubu::extractRoiObstacles(fragmented, corridor, unmerged).size() == 2U,
          "test fixture does not produce two vehicle fragments");
  const auto merged = jubu::extractRoiObstacles(fragmented, corridor, guarded);
  require(merged.size() == 1U, "one opponent remained multiply detected");
  require(merged.front().point_count == 18U,
          "merged opponent did not retain all returns");

  std::cout << "guarded ROI extraction passed" << std::endl;
  return 0;
}
