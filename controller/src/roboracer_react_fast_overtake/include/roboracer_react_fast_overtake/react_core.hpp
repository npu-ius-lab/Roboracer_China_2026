#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace roboracer_react_fast_overtake {

double wrapAngle(double value);
double clamp(double value, double minimum, double maximum);

struct TrackSample {
  double s{};
  double x{};
  double y{};
  double yaw{};
  double curvature{};
  double speed{};
  double width_right{};
  double width_left{};
};

struct TrackProjection {
  double s{};
  double ey{};
  double distance{};
};

class PeriodicTrack {
 public:
  explicit PeriodicTrack(const std::string& csv_path);

  double length() const { return length_; }
  double wrapS(double s) const;
  TrackSample sample(double s) const;
  TrackProjection project(double x, double y) const;

 private:
  std::vector<TrackSample> samples_;
  double length_{};
};

struct RoiPoint {
  double x{};
  double y{};
  double z{};
};

struct CorridorSample {
  double q{};
  double s{};
  double x{};
  double y{};
  double yaw{};
  double curvature{};
  double width_right{};
  double width_left{};
};

struct OccupancyComponent {
  double q{};
  double ey{};
  double q_min{};
  double q_max{};
  double ey_min{};
  double ey_max{};
  double z_min{};
  double z_max{};
  std::size_t point_count{};
};

struct ReactConfig {
  double minimum_q{0.35};
  double maximum_q{5.50};
  double trigger_distance{4.00};
  double boundary_strip{0.10};
  double voxel_size{0.08};
  int minimum_component_points{5};
  double minimum_obstacle_top_z{0.02};

  double ego_width{0.24};
  double ego_length{0.40};
  double lateral_clearance{0.08};
  double candidate_occupancy_buffer{0.03};
  double longitudinal_clearance{0.20};
  double boundary_margin{0.08};
  double minimum_boundary_clearance{0.03};
  double minimum_lane_change_length{0.85};
  double nominal_lane_change_length{1.60};
  double pass_hold_distance{0.70};
  double return_length{1.80};
  double maximum_candidate_length{8.00};
  double maximum_path_curvature{1.80};
  double maximum_track_curvature_for_pass{1.20};
  std::string preferred_side{"left"};

  void validate() const;
};

struct CandidatePath {
  std::string side{"none"};
  bool feasible{false};
  std::string reason{"not_generated"};
  double target_offset{};
  double minimum_clearance{};
  double maximum_curvature{};
  std::vector<double> q;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
};

struct ReactDecision {
  bool centerline_blocked{false};
  std::string reason{"clear"};
  double bumper_gap{};
  CandidatePath left;
  CandidatePath right;
  std::string selected_side{"none"};
};

std::vector<OccupancyComponent> extractOccupancy(
    const std::vector<RoiPoint>& points,
    const std::vector<CorridorSample>& local_corridor,
    const ReactConfig& config);

ReactDecision planReactivePass(
    const std::vector<CorridorSample>& global_corridor,
    double ego_ey,
    const std::vector<OccupancyComponent>& occupancy,
    const ReactConfig& config);

CandidatePath buildReturnPath(
    const std::vector<CorridorSample>& global_corridor,
    double ego_ey,
    double hold_distance,
    const ReactConfig& config);

double curvatureLimitedSpeedCap(
    const CandidatePath& path,
    double configured_speed_cap,
    double lateral_acceleration_limit,
    double minimum_speed_cap);

}  // namespace roboracer_react_fast_overtake
