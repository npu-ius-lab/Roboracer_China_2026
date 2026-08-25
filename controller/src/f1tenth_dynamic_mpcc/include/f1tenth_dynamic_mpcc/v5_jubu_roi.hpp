#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace f1tenth_dynamic_mpcc::v5_jubu {

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
  double width_left{};
  double width_right{};
};

struct RoiObstacle {
  double q{};
  double ey{};
  double local_x{};
  double local_y{};
  double length{};
  double width{};
  double z_min{};
  double z_max{};
  std::size_t point_count{};
};

struct RoiExtractionConfig {
  double minimum_q{0.30};
  double maximum_q{6.00};
  double boundary_strip{0.10};
  double voxel_size{0.10};
  int minimum_component_points{8};
  double maximum_component_length{0.80};
  double maximum_component_width{0.70};
  double minimum_obstacle_top_z{0.02};
  // Candidate-only guards; zero preserves the deployed extractor behavior.
  double minimum_centroid_boundary_clearance{0.0};
  double fragment_merge_longitudinal_gap{0.0};
  double fragment_merge_lateral_gap{0.0};

  void validate() const;
};

std::vector<RoiObstacle> extractRoiObstacles(
    const std::vector<RoiPoint>& points,
    const std::vector<CorridorSample>& corridor,
    const RoiExtractionConfig& config);

struct CandidateConfig {
  double ego_width{0.24};
  double ego_length{0.40};
  double default_obstacle_width{0.35};
  double default_obstacle_length{0.40};
  double boundary_margin{0.08};
  double minimum_boundary_clearance{0.03};
  double lateral_clearance{0.06};
  double longitudinal_clearance{0.20};
  double detection_distance{6.00};
  double trigger_distance{4.00};
  double trigger_ttc{4.00};
  double minimum_closing_speed{0.10};
  double minimum_lane_change_length{0.80};
  double nominal_lane_change_length{1.60};
  double pass_hold_after_obstacle{0.70};
  double return_length{1.60};
  double minimum_plan_speed{0.80};
  double maximum_plan_speed{2.20};
  double pass_speed_advantage{0.60};
  double maximum_path_curvature{1.70};
  double maximum_track_curvature_for_pass{1.20};
  double maximum_candidate_length{12.0};
  std::string preferred_side{"left"};

  void validate() const;
};

struct CandidatePath {
  std::string side;
  bool feasible{false};
  std::string reason;
  std::vector<double> q;
  std::vector<double> s;
  std::vector<double> ey;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  double target_offset{};
  double minimum_boundary_clearance{};
  double maximum_curvature{};
  double score{};
};

struct PlanningDecision {
  bool relevant{false};
  bool risk{false};
  std::string reason;
  double bumper_gap{};
  double closing_speed{};
  double ttc{};
  CandidatePath left;
  CandidatePath right;
  std::string selected_side{"none"};
};

PlanningDecision planAvoidance(
    const std::vector<CorridorSample>& corridor,
    double ego_ey,
    double ego_speed,
    double obstacle_speed,
    const std::vector<RoiObstacle>& obstacles,
    const CandidateConfig& config);

CandidatePath buildReturnPath(
    const std::vector<CorridorSample>& corridor,
    double ego_ey,
    const CandidateConfig& config,
    double hold_distance = 0.0);

}  // namespace f1tenth_dynamic_mpcc::v5_jubu
