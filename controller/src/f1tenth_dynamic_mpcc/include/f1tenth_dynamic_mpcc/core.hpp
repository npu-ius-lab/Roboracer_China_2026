#pragma once

#include <Eigen/Core>
#include <f1tenth_residual_dynamics/residual_model.hpp>
#include <yaml-cpp/yaml.h>

#include <array>
#include <deque>
#include <limits>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace f1tenth_dynamic_mpcc {

constexpr int kNx = 9;
constexpr int kNu = 3;
constexpr int kNp = 14;
using State = Eigen::Matrix<double, kNx, 1>;
using Control = Eigen::Matrix<double, kNu, 1>;

double clamp(double value, double low, double high);
double wrapAngle(double angle);
double dynamicallyFeasibleSpeedCap(double vx, double target_cap,
                                   double speed_gain, double speed_tau,
                                   double max_decel, double margin = 0.02);

struct VehicleParameters {
  double wheelbase{}, body_width{}, mass{}, yaw_inertia{}, lf{}, lr{};
  double cf{}, cr{}, vx_regularization{};
  double speed_gain{}, speed_tau{}, speed_dead_time{};
  // Optional braking-side actuator parameters. Legacy profiles default these
  // to the acceleration-side values and therefore retain identical behavior.
  double braking_speed_tau{}, braking_speed_dead_time{};
  double steering_gain{}, steering_bias{}, steering_tau{}, steering_dead_time{};
  double max_speed{}, max_steer{}, max_accel{}, max_decel{};
  double max_steer_rate{}, lateral_accel_limit{};

  static VehicleParameters fromYaml(const YAML::Node& vehicle,
                                    const YAML::Node& controller);
  void validate() const;
};

struct LowSpeedConditioningResult {
  State state{State::Zero()};
  bool stationary{false};
  bool entered{false};
  bool exited{false};
  double dynamic_blend{1.0};
};

// Projects noisy Point-LIO body velocities onto the low-speed Ackermann
// manifold used by the controller. Pose and steering states are untouched.
class LowSpeedStateConditioner {
 public:
  LowSpeedStateConditioner(const YAML::Node& config, double wheelbase);
  LowSpeedConditioningResult updateAndCondition(const State& state,
                                                double commanded_speed);
  LowSpeedConditioningResult condition(const State& state) const;
  void reset();
  bool stationary() const { return stationary_; }

 private:
  LowSpeedConditioningResult conditionImpl(const State& state,
                                           bool entered,
                                           bool exited) const;
  bool enabled_{true};
  bool stationary_{false};
  double wheelbase_{};
  double enter_vx_{}, exit_vx_{}, command_max_{}, full_dynamic_vx_{};
};

class DynamicBicycleModel {
 public:
  explicit DynamicBicycleModel(
      VehicleParameters parameters, int substeps = 5,
      std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual = nullptr);
  State derivatives(const State& state, const Control& control) const;
  State step(const State& state, const Control& control, double dt) const;
  std::array<double, 2> slipAngles(const State& state) const;
  const VehicleParameters& parameters() const { return p_; }

 private:
  VehicleParameters p_;
  int substeps_;
  std::shared_ptr<const f1tenth_residual_dynamics::ResidualModel> residual_;
};

struct TrackGeometry {
  double x{}, y{}, yaw{}, curvature{}, width_left{}, width_right{}, speed_prior{};
};

struct TrackProjection {
  double s{}, s_wrapped{}, e_contour{}, e_lag{}, distance{};
  double x_ref{}, y_ref{}, psi_ref{};
};

struct TrackSlackSummary {
  double maximum{}, near{}, far{};
  int stage{-1};
  // +1 is the left boundary, -1 is the right boundary, 0 is unknown.
  int side{};
};

struct PredictiveSlackPolicy {
  double emergency_slack{}, far_hard_slack{};
  double retry_speed_scale{1.0}, retry_min_speed{};
};

struct TrackSlackDecision {
  bool retry{}, reject{}, speed_limited{};
  double speed_cap{};
  std::string reason;
};

TrackSlackDecision evaluateTrackSlack(const TrackSlackSummary& slack,
                                      const PredictiveSlackPolicy& policy,
                                      double requested_speed_cap,
                                      bool already_retried);

class PredictiveSpeedLimiter {
 public:
  PredictiveSpeedLimiter(double hold_s, double recovery_rate_mps2);
  void reset();
  void noteRisk(double speed_cap, double now);
  double limit(double requested_speed_cap, double now);
  bool active() const { return active_; }

 private:
  double hold_s_{}, recovery_rate_{};
  double cap_{std::numeric_limits<double>::infinity()};
  double hold_until_{}, last_update_{};
  bool active_{false};
};

class PeriodicSpline {
 public:
  void fit(const std::vector<double>& knots, const std::vector<double>& values,
           double period);
  double eval(double query, int derivative = 0) const;

 private:
  std::vector<double> x_, y_, second_;
  double period_{0.0};
};

class PeriodicTrack {
 public:
  PeriodicTrack(const std::string& csv_path, const YAML::Node& speed_planning,
                const VehicleParameters& vehicle);
  double length() const { return length_; }
  TrackGeometry geometry(double s) const;
  TrackProjection project(double x, double y,
                          std::optional<double> guess = std::nullopt) const;
  std::array<double, 2> position(double s) const;
  double tangent(double s) const;
  double curvature(double s) const;
  double widthLeft(double s) const;
  double widthRight(double s) const;
  double speedPrior(double s) const;
  double csvCurvature(double s) const;
  double wrapS(double s) const;
  const std::vector<double>& speedNodes() const { return speed_nodes_; }

 private:
  std::vector<double> planSpeed(const YAML::Node& cfg,
                                const VehicleParameters& vehicle) const;
  std::vector<double> profileAcceleration(
      const std::vector<double>& speed) const;
  std::vector<double> s_nodes_, x_nodes_, y_nodes_, kappa_nodes_;
  std::vector<double> speed_nodes_, accel_nodes_, speed_limit_nodes_;
  std::vector<double> left_nodes_, right_nodes_;
  PeriodicSpline x_, y_, speed_, accel_, left_, right_, csv_kappa_;
  double length_{0.0};
  std::vector<double> coarse_s_, coarse_x_, coarse_y_;
};

struct TrajectorySafety {
  bool finite{true}, safe{false};
  double minimum_margin{std::numeric_limits<double>::infinity()};
  double initial_margin{std::numeric_limits<double>::infinity()};
  double final_margin{std::numeric_limits<double>::infinity()};
  double maximum_violation{};
  double first_violation_time{-1.0};
  double first_safe_time{-1.0};
  int minimum_margin_stage{-1};
};

TrajectorySafety evaluateTrajectorySafety(
    const std::vector<State>& states, const PeriodicTrack& track,
    double body_width, double boundary_margin, double dt,
    double maximum_time = std::numeric_limits<double>::infinity());

struct PurePursuitCandidateResult {
  bool valid{false};
  double command_speed{}, command_steering{}, virtual_speed{};
  double lookahead{}, lateral_error{}, heading_error{};
  TrajectorySafety safety;
  std::vector<State> states;
  std::vector<Control> controls;
};

class PurePursuitCandidate {
 public:
  PurePursuitCandidate(const YAML::Node& config,
                       const PeriodicTrack& track,
                       DynamicBicycleModel model,
                       double command_acceleration_limit =
                           std::numeric_limits<double>::infinity(),
                       double command_deceleration_limit =
                           std::numeric_limits<double>::infinity());
  PurePursuitCandidateResult generate(const State& initial,
                                      double requested_speed_cap,
                                      bool out_of_bounds_recovery=false,
                                      double initial_command_speed=
                                          std::numeric_limits<double>::quiet_NaN()) const;
  bool enabled() const { return enabled_; }
  double triggerSlack() const { return trigger_slack_; }

 private:
  std::pair<double, double> command(const State& state,
                                    double requested_speed_cap,
                                    double& lookahead,
                                    double& lateral_error,
                                    double& heading_error,
                                    bool out_of_bounds_recovery=false) const;
  const PeriodicTrack& track_;
  DynamicBicycleModel model_;
  bool enabled_{};
  double horizon_s_{}, dt_{}, trigger_slack_{}, boundary_margin_{};
  double lookahead_base_{}, lookahead_speed_gain_{}, lookahead_lateral_gain_{};
  double lookahead_min_{}, lookahead_max_{}, steering_delay_{};
  double max_speed_{}, recovery_speed_min_{}, recovery_speed_max_{};
  double recovery_depth_{}, max_steering_rate_{};
  double lateral_accel_limit_{}, curvature_preview_{};
  double lateral_soft_{}, lateral_hard_{}, heading_soft_{}, heading_hard_{};
  double minimum_speed_scale_{};
  double command_acceleration_limit_{}, command_deceleration_limit_{};
};

struct CheckpointRecoveryDecision {
  bool active{}, activated{}, released{}, use_candidate{}, blocked{};
  std::string reason;
};

class CheckpointRecoveryManager {
 public:
  explicit CheckpointRecoveryManager(const YAML::Node& config);
  CheckpointRecoveryDecision update(
      double lateral_error, double heading_error, double current_margin,
      const TrajectorySafety& candidate, const TrajectorySafety& mpcc);
  void arm();
  void disarm();
  bool armed() const { return armed_; }
  bool active() const { return active_; }
  double speedCap() const { return speed_cap_; }

 private:
  bool candidateAcceptable(double current_margin,
                           const TrajectorySafety& candidate) const;
  bool enabled_{}, armed_{}, active_{};
  double speed_cap_{}, activation_lateral_{}, activation_heading_{};
  double maximum_outside_margin_{}, maximum_worsening_{};
  double reentry_time_{}, final_margin_{};
  double release_lateral_{}, release_heading_{}, release_mpcc_margin_{};
  int release_safe_cycles_{}, safe_cycles_{};
};

struct CandidateSelection {
  bool use_candidate{}, activated{}, released{}, recovery{};
  double improvement{};
  std::string reason;
};

class CandidateArbitrator {
 public:
  explicit CandidateArbitrator(const YAML::Node& config);
  CandidateSelection choose(bool prediction_risk,
                            const TrajectorySafety& mpcc,
                            const TrajectorySafety& candidate,
                            double now);
  void reset();
  bool active() const { return active_; }

 private:
  bool enabled_{};
  double minimum_improvement_{}, minimum_candidate_margin_{};
  double minimum_candidate_margin_tolerance_{};
  double recovery_minimum_initial_margin_{}, recovery_maximum_worsening_{};
  double recovery_reentry_time_{}, recovery_final_margin_{};
  double recovery_minimum_progress_{};
  double hold_s_{}, release_mpcc_margin_{};
  int release_safe_cycles_{}, safe_cycles_{};
  bool active_{}, active_recovery_{};
  double hold_until_{};
};

struct CommandSample {
  double stamp{}, speed{}, steering{}, virtual_speed{}, steering_rate{};
};

class CommandHistory {
 public:
  explicit CommandHistory(double retention = 3.0) : retention_(retention) {}
  void push(const CommandSample& sample);
  void clear();
  std::optional<CommandSample> commandAt(double stamp) const;
  std::optional<CommandSample> effectiveAt(double stamp, double speed_delay,
                                           double steering_delay = 0.0) const;

 private:
  double retention_;
  mutable std::mutex mutex_;
  std::deque<CommandSample> samples_;
};

struct ImuSample { double stamp{}, yaw_rate{}; };
struct WheelSample { double stamp{}, vx{}; };

class SensorHistory {
 public:
  explicit SensorHistory(double retention = 1.0) : retention_(retention) {}
  void pushImu(ImuSample sample);
  void pushWheel(WheelSample sample);
  std::optional<ImuSample> imuBefore(double stamp) const;
  std::optional<WheelSample> wheelBefore(double stamp) const;
  void clear();

 private:
  double retention_;
  mutable std::mutex mutex_;
  std::deque<ImuSample> imu_;
  std::deque<WheelSample> wheel_;
};

class StateHistory {
 public:
  explicit StateHistory(double retention = 1.0) : retention_(retention) {}
  void push(double stamp, const State& state);
  std::optional<State> at(double stamp) const;
  void truncateAfter(double stamp);
  void clear();

 private:
  struct Sample { double stamp; State state; };
  double retention_;
  mutable std::mutex mutex_;
  std::deque<Sample> samples_;
};

struct Prediction {
  State state{State::Zero()};
  double source_stamp{}, output_stamp{}, age{}, horizon{}, confidence{};
  std::string mode;
};

class LowLatencyPredictor {
 public:
  LowLatencyPredictor(DynamicBicycleModel model, CommandHistory& commands,
                      double integration_step = 0.01);
  void pushImu(double stamp, double yaw_rate);
  void pushWheel(double stamp, double vx);
  void clear();
  Prediction correctAndRepropagate(const State& measured, double measurement_stamp,
                                   double now, double max_age,
                                   const Control& fallback);
  Prediction committedHorizon(const State& now_state, double now,
                              const Control& fallback);
  State propagate(const State& state, double start, double end,
                  const Control& fallback, bool record_history);

 private:
  Control controlAt(double stamp, const State& state,
                    const Control& fallback) const;
  double speedDelayFor(const State& state, double stamp,
                       const Control& fallback) const;
  DynamicBicycleModel model_;
  CommandHistory& commands_;
  SensorHistory sensors_;
  StateHistory states_;
  double step_;
  double gyro_bias_{0.0}, imu_max_age_{0.03};
  double wheel_gain_{0.20}, wheel_max_age_{0.06}, wheel_innovation_limit_{0.75};
  std::optional<double> last_pointlio_stamp_, current_stamp_;
  std::optional<State> current_state_;
  std::mutex mutex_;
};

enum class ObserverMode { INVALID, MODEL_ONLY, KINEMATIC, DYNAMIC };
struct ObserverEstimate {
  double stamp{std::numeric_limits<double>::quiet_NaN()};
  double delta{}, target{}, prediction{}, innovation{}, variance{0.04}, confidence{};
  bool valid{false};
  ObserverMode mode{ObserverMode::INVALID};
};

class SteeringObserver {
 public:
  SteeringObserver(const YAML::Node& vehicle_config, const VehicleParameters& vehicle);
  void pushCommand(double stamp, double steering);
  ObserverEstimate update(double stamp, double vx, double vy, double yaw_rate);
  ObserverEstimate estimate() const;
  void invalidate();

 private:
  double commandAt(double stamp) const;
  double target(double command) const;
  double propagate(double delta, double command, double dt) const;
  VehicleParameters vehicle_;
  bool enabled_{};
  double gain_{}, bias_{}, tau_{}, delay_{}, delta_min_{}, delta_max_{};
  double model_only_speed_{}, dynamic_speed_{}, kinematic_gain_{}, dynamic_gain_{};
  double vy_cutoff_{}, r_cutoff_{}, pseudo_cutoff_{}, innovation_limit_{}, timeout_{};
  mutable std::mutex mutex_;
  std::deque<std::pair<double, double>> commands_;
  ObserverEstimate estimate_;
  std::optional<double> last_stamp_, last_vy_, last_r_, pseudo_filtered_;
  double vy_dot_filtered_{}, r_dot_filtered_{};
  int valid_samples_{};
};

enum class StartupMode { BOOT, STEER_SETTLE, CRAWL_OBSERVE, MPCC_RAMP,
                         RACE, SAFE_DECEL, STOPPED };
std::string startupModeName(StartupMode mode);
struct StartupProfile {
  StartupMode mode{StartupMode::BOOT};
  double ramp{}, speed_cap{}, steering_rate_cap{}, progress_scale{};
  double steering_limit{std::numeric_limits<double>::infinity()};
};

class StartupStateMachine {
 public:
  StartupStateMachine(const YAML::Node& controller, double race_speed_cap,
                      double race_steering_rate);
  void reset(double now);
  void updateObserver(double now, bool valid, double confidence, bool pointlio_valid);
  void noteSolver(bool success, double now);
  void forceSafeDecel(double now);
  StartupProfile profile(double now) const;
  StartupMode mode() const;
  int failureCount() const;

 private:
  void enter(StartupMode mode, double now);
  YAML::Node startup_, fallback_;
  double race_speed_cap_, race_steering_rate_;
  mutable std::mutex mutex_;
  StartupMode mode_{StartupMode::BOOT};
  double entered_at_{};
  int observer_samples_{}, recovery_solutions_{}, failures_{};
};

struct PublishedCommand {
  double speed{}, steering{}, virtual_speed{}, dt{};
  bool limited{};
  std::string source;
};

class CommandManager {
 public:
  CommandManager(const YAML::Node& controller, double max_steer);
  void setSolution(const std::vector<State>& states,
                   const std::vector<Control>& controls, double now,
                   bool emergency_deceleration = false);
  void setCandidate(double speed, double steering, double virtual_speed,
                    double now, const std::string& source);
  void noteFailure();
  PublishedCommand tick(double now, const StartupProfile& profile);
  Control published() const;

 private:
  static std::pair<double, bool> slew(double current, double target,
                                      double rate, double dt);
  YAML::Node publisher_, startup_, fallback_;
  double max_steer_;
  mutable std::mutex mutex_;
  Control desired_{Control::Zero()}, shifted_{Control::Zero()}, published_{Control::Zero()};
  double desired_stamp_{-std::numeric_limits<double>::infinity()};
  std::optional<double> last_publish_;
  bool solution_valid_{false}, shifted_available_{false};
  bool emergency_deceleration_{false};
  int failures_{};
  std::string desired_source_{"mpcc"};
};

}  // namespace f1tenth_dynamic_mpcc
