#include "f1tenth_dynamic_mpcc/acados_runtime_jubu.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace mpcc = f1tenth_dynamic_mpcc;

namespace {
mpcc::LocalReferencePoint point(double s, double x) {
  mpcc::TrackGeometry geometry;
  geometry.x = x;
  geometry.y = 0.0;
  geometry.yaw = 0.0;
  geometry.curvature = 0.0;
  geometry.width_left = 1.0;
  geometry.width_right = 1.0;
  geometry.speed_prior = 2.0;
  return {s, geometry};
}
}  // namespace

int main() {
  std::vector<mpcc::LocalReferencePoint> points{
      point(9.5, 0.0), point(9.8, 0.3), point(10.2, 0.7), point(10.5, 1.0)};
  mpcc::LocalReferenceProfile profile(points, 10.0);
  if (!profile.covers(9.7) || !profile.covers(0.2) || profile.covers(8.0)) {
    return 1;
  }

  bool rejected = false;
  try {
    std::vector<mpcc::LocalReferencePoint> invalid{
        point(1.0, 0.0), point(1.2, 0.2), point(1.1, 0.4), point(1.4, 0.6)};
    mpcc::LocalReferenceProfile ignored(invalid, 10.0);
    (void)ignored;
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  return rejected ? 0 : 2;
}
