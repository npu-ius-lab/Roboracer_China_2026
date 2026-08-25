#include "f1tenth_dynamic_mpcc/acados_runtime_chaoche.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace mpcc = f1tenth_dynamic_mpcc;

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  const auto controller = YAML::LoadFile(argv[1]);
  const auto vehicle_config = YAML::LoadFile(argv[2]);
  const auto vehicle = mpcc::VehicleParameters::fromYaml(vehicle_config, controller);
  auto speed = controller["speed_planning"];
  speed["wheelbase_m"] = vehicle.wheelbase;
  speed["max_steer_rad"] = vehicle.max_steer;
  speed["max_steer_rate_radps"] = vehicle.max_steer_rate;
  speed["max_accel_mps2"] = vehicle.max_accel;
  speed["max_decel_mps2"] = vehicle.max_decel;
  speed["lateral_accel_limit_mps2"] = vehicle.lateral_accel_limit;
  mpcc::PeriodicTrack track(argv[3], speed, vehicle);

  std::vector<mpcc::LocalReferencePoint> points;
  constexpr double offset = 0.10;
  for (double s : {1.0, 1.5, 2.0, 2.5, 3.0}) {
    auto geometry = track.geometry(s);
    geometry.x -= offset * std::sin(geometry.yaw);
    geometry.y += offset * std::cos(geometry.yaw);
    geometry.width_left -= offset;
    geometry.width_right += offset;
    points.push_back({s, geometry});
  }
  mpcc::LocalReferenceProfile profile(std::move(points), track.length());
  if (!profile.covers(2.0) || profile.covers(0.0)) return 10;
  const auto local = profile.geometry(track, 2.0);
  const auto global = track.geometry(2.0);
  const double nx = -std::sin(global.yaw), ny = std::cos(global.yaw);
  const double measured_offset =
      (local.x - global.x) * nx + (local.y - global.y) * ny;
  if (std::abs(measured_offset - offset) > 1.0e-6) return 11;
  if (std::abs(local.width_left - (global.width_left - offset)) > 1.0e-6 ||
      std::abs(local.width_right - (global.width_right + offset)) > 1.0e-6)
    return 12;
  const auto fallback = profile.geometry(track, 0.0);
  const auto expected_fallback = track.geometry(0.0);
  if (std::hypot(fallback.x - expected_fallback.x,
                 fallback.y - expected_fallback.y) > 1.0e-12)
    return 13;
  std::cout << "V5 Chaoche local reference shift/boundary/global-fallback OK\n";
  return 0;
}
