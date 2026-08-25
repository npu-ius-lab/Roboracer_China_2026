#pragma once

#include "f1tenth_dynamic_mpcc/core.hpp"

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace f1tenth_dynamic_mpcc {

struct CostBreakdown {
  double contour{}, lag{}, heading{}, speed_prior{}, progress{};
  double control{}, delta_control{}, track_slack{}, tire_slack{};
};

struct SolverResult {
  bool success{false};
  int status{-1}, rti_iterations{};
  double solve_time{}, objective{}, track_slack{}, tire_slack{};
  double near_track_slack{}, far_track_slack{}, track_slack_time{};
  double trigger_track_slack{}, trigger_track_slack_time{};
  double speed_cap_used{}, speed_input_cap{};
  double geometry_theta_shift{}, geometry_heading_shift{};
  double geometry_curvature_shift{};
  double warm_start_advance_s{}, warm_start_shift_stages{};
  int track_slack_stage{-1}, track_slack_side{}, failure_kind{};
  int trigger_track_slack_stage{-1}, trigger_track_slack_side{};
  bool speed_retry{}, warm_start_reset{};
  CostBreakdown cost;
  std::string reason;
  std::vector<State> states;
  std::vector<Control> controls;
};

class AcadosRuntimeSolver {
 public:
  AcadosRuntimeSolver(const std::string& formulation, const std::string& generated_root,
                      const PeriodicTrack& track, const VehicleParameters& vehicle,
                      const YAML::Node& controller,
                      std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual = nullptr);
  ~AcadosRuntimeSolver();
  AcadosRuntimeSolver(const AcadosRuntimeSolver&) = delete;
  AcadosRuntimeSolver& operator=(const AcadosRuntimeSolver&) = delete;

  SolverResult solve(const State& initial, double speed_cap,
                     double steering_rate_cap, double progress_scale,
                     double warm_start_advance_s = -1.0);
  void resetWarmStart();

 private:
  void initializeWarmStart(const State& initial, double target_speed_cap,
                           double input_speed_cap, double rate_cap);
  void advanceWarmStart(double elapsed_s);
  void setInputBounds(double input_speed_cap, double rate_cap);
  void seedSolver(const State& initial, double target_speed_cap,
                  double input_speed_cap, double rate_cap,
                  double progress_scale);
  void updateStageParameters(const std::vector<State>& states,
                             const std::vector<Control>& controls,
                             double speed_cap, double progress_scale);
  void readSolution(std::vector<State>& states,
                    std::vector<Control>& controls) const;
  TrackSlackSummary readSlack(double near_horizon_s,
                              double& tire_slack) const;
  CostBreakdown diagnosticCost(const std::vector<State>& states,
                               const std::vector<Control>& controls,
                               double target_speed_cap,
                               double progress_scale) const;
  double feasibleInputSpeedCap(const State& initial,
                               double target_speed_cap) const;
  void* symbol(void* handle, const std::string& name) const;
  std::string formulation_, prefix_;
  const PeriodicTrack& track_;
  VehicleParameters vehicle_;
  YAML::Node controller_;
  std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual_;
  int horizon_{};
  double dt_{};
  void *acados_handle_{}, *solver_handle_{}, *capsule_{};
  void *config_{}, *dims_{}, *input_{}, *output_{}, *nlp_solver_{};
  using CapsuleCreate = void* (*)();
  using CapsuleFree = int (*)(void*);
  using Create = int (*)(void*);
  using Free = int (*)(void*);
  using Solve = int (*)(void*);
  using UpdateParams = int (*)(void*, int, double*, int);
  using GetObject = void* (*)(void*);
  CapsuleCreate create_capsule_{}; CapsuleFree free_capsule_{};
  Create create_{}; Free free_{}; Solve solve_{}; UpdateParams update_params_{};
  using ConstraintSet = int (*)(void*,void*,void*,void*,int,const char*,void*);
  using OutSet = void (*)(void*,void*,void*,void*,int,const char*,void*);
  using OutGet = void (*)(void*,void*,void*,int,const char*,void*);
  using NlpGet = void (*)(void*,const char*,void*);
  ConstraintSet constraint_set_{}; OutSet out_set_{}; OutGet out_get_{}; NlpGet nlp_get_{};
  std::vector<State> warm_states_;
  std::vector<Control> warm_controls_;
  Control previous_control_{Control::Zero()};
  bool warm_valid_{false};
};

}  // namespace f1tenth_dynamic_mpcc
