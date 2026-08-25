#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

double age(const ros::WallTime& stamp) {
  if (stamp.toSec() <= 0.0) return 1e99;
  return std::max(0.0, (ros::WallTime::now() - stamp).toSec());
}

double wrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) fields.push_back(field);
  return fields;
}

std::optional<std::size_t> jsonValueBegin(const std::string& input,
                                          const std::string& key) {
  const std::string token = "\"" + key + "\"";
  const auto key_begin = input.find(token);
  if (key_begin == std::string::npos) return std::nullopt;
  const auto colon = input.find(':', key_begin + token.size());
  if (colon == std::string::npos) return std::nullopt;
  std::size_t value_begin = colon + 1;
  while (value_begin < input.size() &&
         std::isspace(static_cast<unsigned char>(input[value_begin]))) {
    ++value_begin;
  }
  return value_begin;
}

bool jsonTrue(const std::string& input, const std::string& key) {
  const auto begin = jsonValueBegin(input, key);
  return begin.has_value() && input.compare(*begin, 4, "true") == 0;
}

std::optional<double> jsonNumber(const std::string& input,
                                 const std::string& key) {
  const auto value_begin = jsonValueBegin(input, key);
  if (!value_begin.has_value()) return std::nullopt;
  if (input.compare(*value_begin, 4, "null") == 0) return std::nullopt;
  try {
    std::size_t consumed = 0;
    const double value = std::stod(input.substr(*value_begin), &consumed);
    if (consumed == 0 || !std::isfinite(value)) return std::nullopt;
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

bool jsonState(const std::string& input, const std::string& state) {
  const auto begin = jsonValueBegin(input, "state");
  const std::string expected = "\"" + state + "\"";
  return begin.has_value() &&
      input.compare(*begin, expected.size(), expected) == 0;
}

struct TrackPoint {
  double x{};
  double y{};
  double psi{};
  double right{};
  double left{};
};

struct TrackState {
  bool valid{false};
  double lateral{};
  double margin{-1e9};
  double heading_error{};
};

class FollowOnlyFloorGate {
 public:
  FollowOnlyFloorGate() : private_node_("~") {
    bool hardware_allowed = false;
    private_node_.param("allow_real_hardware", hardware_allowed, false);
    if (!hardware_allowed) {
      throw std::runtime_error("allow_real_hardware must be true");
    }

    private_node_.param("input_topic", input_topic_,
                        std::string("/roboracer_h2h_faststart/supervised_command"));
    private_node_.param("output_topic", output_topic_,
                        std::string("/tianracer/ackermann_cmd"));
    private_node_.param("state_topic", state_topic_,
                        std::string("/roboracer_h2h_faststart/follow_gate_state"));
    private_node_.param("planner_state_topic", planner_state_topic_,
                        std::string("/roboracer_h2h_faststart/state"));
    private_node_.param("planner_status_topic", planner_status_topic_,
                        std::string("/roboracer_h2h_faststart/status"));
    private_node_.param("planner_diagnostics_topic", planner_diagnostics_topic_,
                        std::string("/roboracer_h2h_faststart/candidate_diagnostics"));
    private_node_.param("planner_health_topic", planner_health_topic_,
                        std::string("/roboracer_h2h_faststart/health"));
    private_node_.param("supervisor_state_topic", supervisor_state_topic_,
                        std::string("/automatic_relaunch_supervisor_roboracer_h2h_faststart/state"));
    private_node_.param("telemetry_topic", telemetry_topic_,
                        std::string("/roboracer_h2h_faststart/mpcc/telemetry"));
    private_node_.param("vehicle_odom_topic", vehicle_odom_topic_,
                        std::string("/localization/vehicle_odom"));
    private_node_.param("wheel_odom_topic", wheel_odom_topic_,
                        std::string("/tianracer/odom"));
    private_node_.param("track_csv", track_csv_, std::string());

    private_node_.param("minimum_follow_speed_mps", minimum_follow_speed_, 1.0);
    private_node_.param("minimum_floor_input_speed_mps", minimum_floor_input_speed_, 0.05);
    private_node_.param("follow_confirmation_s", follow_confirmation_, 0.18);
    private_node_.param("minimum_follow_gap_m", minimum_follow_gap_, 0.45);
    private_node_.param("minimum_normal_track_margin_m", minimum_track_margin_, 0.05);
    private_node_.param("maximum_normal_heading_error_rad", maximum_heading_error_, 0.35);
    private_node_.param("body_width_m", body_width_, 0.24);
    private_node_.param("boundary_margin_m", boundary_margin_, 0.10);
    private_node_.param("stalled_speed_max_mps", stalled_speed_max_, 0.15);
    private_node_.param("stall_command_min_mps", stall_command_min_, 0.80);
    private_node_.param("stall_confirmation_s", stall_confirmation_, 0.18);
    private_node_.param("input_timeout_s", input_timeout_, 0.25);
    private_node_.param("state_timeout_s", state_timeout_, 0.35);
    private_node_.param("odom_timeout_s", odom_timeout_, 0.25);
    double publish_rate = 60.0;
    private_node_.param("publish_rate_hz", publish_rate, 60.0);

    validate(publish_rate);
    loadTrack();

    command_publisher_ =
        node_.advertise<ackermann_msgs::AckermannDrive>(output_topic_, 1);
    state_publisher_ = node_.advertise<std_msgs::String>(state_topic_, 5);
    command_subscriber_ = node_.subscribe(
        input_topic_, 1, &FollowOnlyFloorGate::commandCallback, this,
        ros::TransportHints().tcpNoDelay());
    planner_state_subscriber_ = node_.subscribe(
        planner_state_topic_, 5, &FollowOnlyFloorGate::plannerStateCallback, this,
        ros::TransportHints().tcpNoDelay());
    planner_status_subscriber_ = node_.subscribe(
        planner_status_topic_, 5, &FollowOnlyFloorGate::plannerStatusCallback, this,
        ros::TransportHints().tcpNoDelay());
    planner_diagnostics_subscriber_ = node_.subscribe(
        planner_diagnostics_topic_, 5,
        &FollowOnlyFloorGate::plannerDiagnosticsCallback, this,
        ros::TransportHints().tcpNoDelay());
    planner_health_subscriber_ = node_.subscribe(
        planner_health_topic_, 5, &FollowOnlyFloorGate::plannerHealthCallback, this,
        ros::TransportHints().tcpNoDelay());
    supervisor_state_subscriber_ = node_.subscribe(
        supervisor_state_topic_, 5,
        &FollowOnlyFloorGate::supervisorStateCallback, this,
        ros::TransportHints().tcpNoDelay());
    telemetry_subscriber_ = node_.subscribe(
        telemetry_topic_, 5, &FollowOnlyFloorGate::telemetryCallback, this,
        ros::TransportHints().tcpNoDelay());
    vehicle_odom_subscriber_ = node_.subscribe(
        vehicle_odom_topic_, 5, &FollowOnlyFloorGate::vehicleOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
    wheel_odom_subscriber_ = node_.subscribe(
        wheel_odom_topic_, 5, &FollowOnlyFloorGate::wheelOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
    timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate),
                               &FollowOnlyFloorGate::timerCallback, this);
    publishZero();
    ROS_WARN_STREAM("ROBORACER H2H follow-only floor gate ready; floor="
                    << minimum_follow_speed_
                    << "m/s only for confirmed healthy FOLLOW");
  }

  ~FollowOnlyFloorGate() { publishZero(); }

 private:
  void validate(double publish_rate) const {
    const bool valid = publish_rate > 0.0 && minimum_follow_speed_ > 0.0 &&
        minimum_floor_input_speed_ >= 0.0 &&
        minimum_floor_input_speed_ < minimum_follow_speed_ &&
        follow_confirmation_ >= 0.0 && minimum_follow_gap_ >= 0.0 &&
        minimum_track_margin_ >= 0.0 && maximum_heading_error_ > 0.0 &&
        body_width_ > 0.0 && boundary_margin_ >= 0.0 &&
        stalled_speed_max_ >= 0.0 && stall_command_min_ > 0.0 &&
        stall_confirmation_ > 0.0 && input_timeout_ > 0.0 &&
        state_timeout_ > 0.0 && odom_timeout_ > 0.0;
    if (!valid || track_csv_.empty()) {
      throw std::runtime_error("invalid follow-only floor gate parameters");
    }
  }

  void loadTrack() {
    std::ifstream stream(track_csv_);
    if (!stream) throw std::runtime_error("cannot open track_csv: " + track_csv_);
    std::string line;
    std::getline(stream, line);
    while (std::getline(stream, line)) {
      const auto fields = splitCsv(line);
      if (fields.size() < 11) continue;
      try {
        TrackPoint point;
        point.x = std::stod(fields[1]);
        point.y = std::stod(fields[2]);
        point.psi = std::stod(fields[3]);
        point.right = std::stod(fields[9]);
        point.left = std::stod(fields[10]);
        track_.push_back(point);
      } catch (const std::exception&) {
      }
    }
    if (track_.size() < 20) {
      throw std::runtime_error("track_csv has too few valid points");
    }
  }

  void commandCallback(const ackermann_msgs::AckermannDrive::ConstPtr& message) {
    if (!std::isfinite(message->speed) ||
        !std::isfinite(message->steering_angle)) {
      command_.reset();
      return;
    }
    command_ = *message;
    command_wall_ = ros::WallTime::now();
  }

  void plannerStateCallback(const std_msgs::String::ConstPtr& message) {
    planner_follow_ = message->data == "FOLLOW";
    planner_state_text_ = message->data;
    planner_state_wall_ = ros::WallTime::now();
  }

  void plannerStatusCallback(const std_msgs::String::ConstPtr& message) {
    planner_status_healthy_ = jsonTrue(message->data, "healthy");
    planner_status_follow_ = jsonState(message->data, "FOLLOW");
    planner_status_wall_ = ros::WallTime::now();
  }

  void plannerDiagnosticsCallback(const std_msgs::String::ConstPtr& message) {
    obstacle_relevant_ = jsonTrue(message->data, "relevant");
    bumper_gap_ = jsonNumber(message->data, "bumper_gap_m");
    planner_diagnostics_wall_ = ros::WallTime::now();
  }

  void plannerHealthCallback(const std_msgs::Bool::ConstPtr& message) {
    planner_healthy_ = message->data;
    planner_health_wall_ = ros::WallTime::now();
  }

  void supervisorStateCallback(const std_msgs::String::ConstPtr& message) {
    supervisor_running_ = jsonState(message->data, "RUNNING");
    supervisor_state_text_ = message->data;
    supervisor_state_wall_ = ros::WallTime::now();
  }

  void telemetryCallback(
      const std_msgs::Float32MultiArray::ConstPtr& message) {
    telemetry_valid_ = message->data.size() > 72;
    if (telemetry_valid_) {
      solver_failures_ = message->data[10];
      solver_success_ = message->data[12] > 0.5f;
      prediction_risk_ = message->data[52] > 0.5f;
      candidate_reason_ = static_cast<int>(std::lround(message->data[59]));
      pp_risk_ = message->data[72] > 0.5f;
    }
    telemetry_wall_ = ros::WallTime::now();
  }

  void vehicleOdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const auto& p = message->pose.pose.position;
    const auto& q = message->pose.pose.orientation;
    vehicle_x_ = p.x;
    vehicle_y_ = p.y;
    vehicle_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    vehicle_speed_ = message->twist.twist.linear.x;
    vehicle_odom_wall_ = ros::WallTime::now();
    track_state_ = projectTrack();
  }

  void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    wheel_speed_ = message->twist.twist.linear.x;
    wheel_odom_wall_ = ros::WallTime::now();
  }

  TrackState projectTrack() const {
    TrackState output;
    if (track_.empty() || !std::isfinite(vehicle_x_) ||
        !std::isfinite(vehicle_y_) || !std::isfinite(vehicle_yaw_)) {
      return output;
    }
    std::size_t nearest = 0;
    double best_score = std::numeric_limits<double>::infinity();
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < track_.size(); ++i) {
      const double dx = vehicle_x_ - track_[i].x;
      const double dy = vehicle_y_ - track_[i].y;
      const double distance = dx * dx + dy * dy;
      const double heading = std::abs(wrapAngle(vehicle_yaw_ - track_[i].psi));
      const double score = distance + 0.10 * heading * heading;
      if (score < best_score) {
        best_score = score;
        best_distance = distance;
        nearest = i;
      }
    }
    const auto& point = track_[nearest];
    const double dx = vehicle_x_ - point.x;
    const double dy = vehicle_y_ - point.y;
    output.lateral = -std::sin(point.psi) * dx + std::cos(point.psi) * dy;
    const double footprint = body_width_ / 2.0 + boundary_margin_;
    output.margin = std::min(point.left - footprint - output.lateral,
                             point.right - footprint + output.lateral);
    output.heading_error = wrapAngle(vehicle_yaw_ - point.psi);
    output.valid = std::isfinite(output.margin) && best_distance < 4.0;
    return output;
  }

  bool fresh(const ros::WallTime& stamp, double timeout) const {
    return age(stamp) <= timeout;
  }

  bool supervisorRunning() const {
    return supervisor_running_ && fresh(supervisor_state_wall_, state_timeout_);
  }

  bool confirmedFollowSignal() const {
    return planner_follow_ && planner_status_follow_ &&
        planner_status_healthy_ && obstacle_relevant_ &&
        bumper_gap_.has_value() && *bumper_gap_ >= minimum_follow_gap_ &&
        fresh(planner_state_wall_, state_timeout_) &&
        fresh(planner_status_wall_, state_timeout_) &&
        fresh(planner_diagnostics_wall_, state_timeout_);
  }

  bool normalForFollowFloor() const {
    const bool healthy = planner_healthy_ &&
        fresh(planner_health_wall_, state_timeout_);
    const bool telemetry_normal = telemetry_valid_ && solver_success_ &&
        solver_failures_ < 0.5 && !prediction_risk_ && !pp_risk_ &&
        candidate_reason_ != 4 && candidate_reason_ != 5 &&
        candidate_reason_ != 10 && fresh(telemetry_wall_, state_timeout_);
    const bool track_normal = track_state_.valid &&
        track_state_.margin >= minimum_track_margin_ &&
        std::abs(track_state_.heading_error) <= maximum_heading_error_;
    const bool odom_fresh = fresh(vehicle_odom_wall_, odom_timeout_) &&
        fresh(wheel_odom_wall_, odom_timeout_);
    return supervisorRunning() && healthy && telemetry_normal && track_normal &&
        odom_fresh && !stuck_latched_;
  }

  bool followConfirmed() const {
    return follow_since_.toSec() > 0.0 &&
        age(follow_since_) >= follow_confirmation_;
  }

  void updateConfirmation() {
    const ros::WallTime now = ros::WallTime::now();
    if (confirmedFollowSignal()) {
      if (follow_since_.toSec() <= 0.0) follow_since_ = now;
    } else {
      follow_since_ = ros::WallTime();
    }

    const bool stall_candidate = supervisorRunning() && command_.has_value() &&
        command_->speed >= stall_command_min_ &&
        std::abs(vehicle_speed_) <= stalled_speed_max_ &&
        std::abs(wheel_speed_) <= stalled_speed_max_ &&
        fresh(vehicle_odom_wall_, odom_timeout_) &&
        fresh(wheel_odom_wall_, odom_timeout_);
    if (stall_candidate) {
      if (stall_since_.toSec() <= 0.0) stall_since_ = now;
      if (age(stall_since_) >= stall_confirmation_) stuck_latched_ = true;
    } else {
      stall_since_ = ros::WallTime();
      if (std::abs(vehicle_speed_) > 0.40 && std::abs(wheel_speed_) > 0.40) {
        stuck_latched_ = false;
      }
    }
    if (!supervisorRunning()) {
      stuck_latched_ = false;
      stall_since_ = ros::WallTime();
    }
  }

  void publishZero() {
    command_publisher_.publish(ackermann_msgs::AckermannDrive{});
  }

  void publishState(const std::string& state, bool floor_applied,
                    bool normal) {
    std_msgs::String message;
    std::ostringstream text;
    text << "{\"state\":\"" << state << "\""
         << ",\"floor_applied\":" << (floor_applied ? "true" : "false")
         << ",\"normal\":" << (normal ? "true" : "false")
         << ",\"confirmed_follow\":"
         << (followConfirmed() ? "true" : "false")
         << ",\"planner_state\":\"" << planner_state_text_ << "\""
         << ",\"supervisor_running\":"
         << (supervisorRunning() ? "true" : "false")
         << ",\"planner_healthy\":"
         << ((planner_healthy_ && fresh(planner_health_wall_, state_timeout_))
                 ? "true" : "false")
         << ",\"obstacle_relevant\":"
         << (obstacle_relevant_ ? "true" : "false")
         << ",\"bumper_gap_m\":";
    if (bumper_gap_) text << *bumper_gap_;
    else text << "null";
    text << ",\"track_margin_m\":" << track_state_.margin
         << ",\"heading_error_rad\":" << track_state_.heading_error
         << ",\"pp_risk\":" << (pp_risk_ ? "true" : "false")
         << ",\"prediction_risk\":"
         << (prediction_risk_ ? "true" : "false")
         << ",\"candidate_reason\":" << candidate_reason_
         << ",\"stuck_latched\":"
         << (stuck_latched_ ? "true" : "false") << "}";
    message.data = text.str();
    state_publisher_.publish(message);
  }

  void timerCallback(const ros::TimerEvent&) {
    updateConfirmation();
    if (!command_.has_value() || !fresh(command_wall_, input_timeout_)) {
      publishZero();
      publishState("WAIT_INPUT", false, false);
      return;
    }

    // The relaunch supervisor owns launch, carry recovery, handoff, and its
    // exact zero states. This gate passes every one of those commands without
    // modification. The only mutation below is the narrowly-scoped FOLLOW
    // floor while the supervisor is in normal RUNNING mode.
    auto output = *command_;
    const bool normal = normalForFollowFloor();
    const bool confirmed_follow = followConfirmed();
    const bool positive_intent = output.speed > minimum_floor_input_speed_;
    const bool floor_applied = normal && confirmed_follow && positive_intent &&
        output.speed < minimum_follow_speed_;
    if (floor_applied) output.speed = minimum_follow_speed_;
    command_publisher_.publish(output);
    publishState(floor_applied ? "FOLLOW_FLOOR" : "TRANSPARENT",
                 floor_applied, normal);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher command_publisher_;
  ros::Publisher state_publisher_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber planner_state_subscriber_;
  ros::Subscriber planner_status_subscriber_;
  ros::Subscriber planner_diagnostics_subscriber_;
  ros::Subscriber planner_health_subscriber_;
  ros::Subscriber supervisor_state_subscriber_;
  ros::Subscriber telemetry_subscriber_;
  ros::Subscriber vehicle_odom_subscriber_;
  ros::Subscriber wheel_odom_subscriber_;
  ros::Timer timer_;

  std::optional<ackermann_msgs::AckermannDrive> command_;
  std::optional<double> bumper_gap_;
  std::vector<TrackPoint> track_;
  TrackState track_state_;
  ros::WallTime command_wall_;
  ros::WallTime planner_state_wall_;
  ros::WallTime planner_status_wall_;
  ros::WallTime planner_diagnostics_wall_;
  ros::WallTime planner_health_wall_;
  ros::WallTime supervisor_state_wall_;
  ros::WallTime telemetry_wall_;
  ros::WallTime vehicle_odom_wall_;
  ros::WallTime wheel_odom_wall_;
  ros::WallTime follow_since_;
  ros::WallTime stall_since_;

  std::string input_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string planner_state_topic_;
  std::string planner_status_topic_;
  std::string planner_diagnostics_topic_;
  std::string planner_health_topic_;
  std::string supervisor_state_topic_;
  std::string telemetry_topic_;
  std::string vehicle_odom_topic_;
  std::string wheel_odom_topic_;
  std::string track_csv_;
  std::string planner_state_text_{"UNKNOWN"};
  std::string supervisor_state_text_;

  double minimum_follow_speed_{};
  double minimum_floor_input_speed_{};
  double follow_confirmation_{};
  double minimum_follow_gap_{};
  double minimum_track_margin_{};
  double maximum_heading_error_{};
  double body_width_{};
  double boundary_margin_{};
  double stalled_speed_max_{};
  double stall_command_min_{};
  double stall_confirmation_{};
  double input_timeout_{};
  double state_timeout_{};
  double odom_timeout_{};
  double vehicle_x_{std::numeric_limits<double>::quiet_NaN()};
  double vehicle_y_{std::numeric_limits<double>::quiet_NaN()};
  double vehicle_yaw_{std::numeric_limits<double>::quiet_NaN()};
  double vehicle_speed_{};
  double wheel_speed_{};
  double solver_failures_{1e9};
  int candidate_reason_{};
  bool planner_follow_{false};
  bool planner_status_follow_{false};
  bool planner_status_healthy_{false};
  bool obstacle_relevant_{false};
  bool planner_healthy_{false};
  bool supervisor_running_{false};
  bool telemetry_valid_{false};
  bool solver_success_{false};
  bool prediction_risk_{false};
  bool pp_risk_{false};
  bool stuck_latched_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "roboracer_h2h_follow_gate_cpp");
  try {
    FollowOnlyFloorGate gate;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("ROBORACER H2H follow-only floor gate failed: "
                     << error.what());
    return 1;
  }
  return 0;
}
