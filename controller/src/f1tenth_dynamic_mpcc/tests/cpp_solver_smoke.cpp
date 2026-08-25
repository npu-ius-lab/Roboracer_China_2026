#include "f1tenth_dynamic_mpcc/acados_runtime.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace mpcc = f1tenth_dynamic_mpcc;

int main(int argc, char** argv) {
  if (argc != 5 && argc != 11) return 2;
  auto controller = YAML::LoadFile(argv[1]);
  auto vehicle_config = YAML::LoadFile(argv[2]);
  auto vehicle = mpcc::VehicleParameters::fromYaml(vehicle_config, controller);
  auto speed = controller["speed_planning"];
  speed["wheelbase_m"] = vehicle.wheelbase;
  speed["max_steer_rad"] = vehicle.max_steer;
  speed["max_steer_rate_radps"] = vehicle.max_steer_rate;
  speed["max_accel_mps2"] = vehicle.max_accel;
  speed["max_decel_mps2"] = vehicle.max_decel;
  speed["lateral_accel_limit_mps2"] = vehicle.lateral_accel_limit;
  mpcc::PeriodicTrack track(argv[3], speed, vehicle);
  mpcc::AcadosRuntimeSolver solver(
      argc == 11 ? "mpcc" : "baseline", argv[4], track, vehicle, controller);
  const auto geometry = track.geometry(0.0);
  mpcc::State state = mpcc::State::Zero();
  double speed_cap = 1.0;
  if (argc == 11) {
    const double x = std::stod(argv[5]), y = std::stod(argv[6]);
    const auto projection = track.project(x, y);
    state << x, y, std::stod(argv[7]), std::stod(argv[8]),
             std::stod(argv[9]), std::stod(argv[10]), 0.03, 0.05, projection.s;
    speed_cap = 3.0;
  } else {
    state << geometry.x, geometry.y, geometry.yaw, 0.6, 0.0, 0.0,
             std::atan(vehicle.wheelbase * geometry.curvature), 0.0, 0.0;
  }
  mpcc::SolverResult result;
  const int attempts = argc == 11 ? 8 : 1;
  for (int attempt = 0; attempt < attempts; ++attempt) {
    result = solver.solve(state, speed_cap, 1.5, 1.0);
    std::cout << "attempt=" << attempt << " status=" << result.status
              << " success=" << result.success
              << " time=" << result.solve_time
              << " track_slack=" << result.track_slack
              << " near=" << result.near_track_slack
              << " far=" << result.far_track_slack
              << " stage=" << result.track_slack_stage
              << " cap=" << result.speed_cap_used
              << " retry=" << result.speed_retry
              << " reset=" << result.warm_start_reset
              << " tire_slack=" << result.tire_slack
              << " reason=" << result.reason << '\n';
    if (result.success) break;
  }
  if (!result.success || result.states.size() != 26 || result.controls.size() != 25 ||
      !std::isfinite(result.solve_time)) return 20;
  return 0;
}
