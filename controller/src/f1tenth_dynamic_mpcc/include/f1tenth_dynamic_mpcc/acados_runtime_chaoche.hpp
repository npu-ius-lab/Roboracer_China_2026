#pragma once

#include "f1tenth_dynamic_mpcc/acados_runtime.hpp"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace f1tenth_dynamic_mpcc {

// A short, forward-only reference expressed on the global raceline progress
// axis. It changes only the MPCC reference geometry; physical track
// boundaries are carried into the shifted frame so the acados constraints
// continue to describe the real corridor.
struct LocalReferencePoint {
  double s{};
  TrackGeometry geometry;
};

class LocalReferenceProfile {
 public:
  LocalReferenceProfile(std::vector<LocalReferencePoint> points,
                        double track_length);

  TrackGeometry geometry(const PeriodicTrack& global_track, double s) const;
  bool covers(double s) const;
  double startS() const { return points_.front().s; }
  double endS() const { return points_.back().s; }
  std::size_t size() const { return points_.size(); }

 private:
  double unwrapNear(double s, double reference) const;
  std::vector<LocalReferencePoint> points_;
  double track_length_{};
};

// Stable V5 remains linked to AcadosRuntimeSolver. Only the new Chaoche
// executable uses this independent solver wrapper.
class AcadosRuntimeSolverChaoche {
 public:
  AcadosRuntimeSolverChaoche(
      const std::string& formulation, const std::string& generated_root,
      const PeriodicTrack& track, const VehicleParameters& vehicle,
      const YAML::Node& controller,
      std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual = nullptr);
  ~AcadosRuntimeSolverChaoche();
  AcadosRuntimeSolverChaoche(const AcadosRuntimeSolverChaoche&) = delete;
  AcadosRuntimeSolverChaoche& operator=(const AcadosRuntimeSolverChaoche&) = delete;

  SolverResult solve(const State& initial, double speed_cap,
                     double steering_rate_cap, double progress_scale,
                     double warm_start_advance_s = -1.0);
  void resetWarmStart();
  void setLocalReference(std::shared_ptr<const LocalReferenceProfile> reference);
  void clearLocalReference();
  bool localReferenceActive() const;
  TrackGeometry referenceGeometry(double s) const;

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
  std::shared_ptr<const LocalReferenceProfile> local_reference_;
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
  CapsuleCreate create_capsule_{};
  CapsuleFree free_capsule_{};
  Create create_{};
  Free free_{};
  Solve solve_{};
  UpdateParams update_params_{};
  using ConstraintSet = int (*)(void*, void*, void*, void*, int,
                                const char*, void*);
  using OutSet = void (*)(void*, void*, void*, void*, int, const char*, void*);
  using OutGet = void (*)(void*, void*, void*, int, const char*, void*);
  using NlpGet = void (*)(void*, const char*, void*);
  ConstraintSet constraint_set_{};
  OutSet out_set_{};
  OutGet out_get_{};
  NlpGet nlp_get_{};
  std::vector<State> warm_states_;
  std::vector<Control> warm_controls_;
  Control previous_control_{Control::Zero()};
  bool warm_valid_{false};
};

}  // namespace f1tenth_dynamic_mpcc
