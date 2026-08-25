#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
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

bool jsonState(const std::string& input, const std::string& state) {
  const auto begin = jsonValueBegin(input, "state");
  const std::string expected = "\"" + state + "\"";
  return begin.has_value() &&
      input.compare(*begin, expected.size(), expected) == 0;
}

std::optional<double> jsonNumber(const std::string& input,
                                 const std::string& key) {
  const auto begin = jsonValueBegin(input, key);
  if (!begin.has_value() || input.compare(*begin, 4, "null") == 0) {
    return std::nullopt;
  }
  try {
    std::size_t consumed = 0;
    const double value = std::stod(input.substr(*begin), &consumed);
    if (consumed == 0 || !std::isfinite(value)) return std::nullopt;
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

struct TrackPoint {
  double x{};
  double y{};
  double psi{};
  double right{};
  double left{};
};

struct RawTrackState {
  bool valid{false};
  double lateral{};
  double margin{-1e9};
  double heading_error{};
};

enum class EscapeState {
  kForwarding,
  kBrake,
  kReverseDwell,
  kReverse,
  kForwardDwell,
  kForwardRecovery,
  kCooldown,
  kRearBlocked,
  kRecoveryFailed,
};

const char* stateName(EscapeState state) {
  switch (state) {
    case EscapeState::kForwarding: return "FORWARDING";
    case EscapeState::kBrake: return "CONTACT_BRAKE";
    case EscapeState::kReverseDwell: return "REVERSE_DWELL";
    case EscapeState::kReverse: return "REVERSE_ESCAPE";
    case EscapeState::kForwardDwell: return "FORWARD_DWELL";
    case EscapeState::kForwardRecovery: return "FORWARD_RECOVERY";
    case EscapeState::kCooldown: return "RACE_COOLDOWN";
    case EscapeState::kRearBlocked: return "REAR_BLOCKED";
    case EscapeState::kRecoveryFailed: return "RECOVERY_FAILED";
  }
  return "UNKNOWN";
}

class ContactEscapeGate {
 public:
  ContactEscapeGate() : private_node_("~") {
    bool hardware_allowed = false;
    private_node_.param("allow_real_hardware", hardware_allowed, false);
    if (!hardware_allowed) {
      throw std::runtime_error("allow_real_hardware must be true");
    }
    private_node_.param("input_topic", input_topic_,
                        std::string("/roboracer_eth_h2h/pre_escape_command"));
    private_node_.param("output_topic", output_topic_,
                        std::string("/tianracer/ackermann_cmd"));
    private_node_.param("state_topic", state_topic_,
                        std::string("/roboracer_eth_h2h/contact_escape_state"));
    private_node_.param("raw_mpcc_command_topic", raw_mpcc_command_topic_,
                        std::string("/roboracer_eth_h2h/mpcc/ackermann_cmd_stamped"));
    private_node_.param("vehicle_odom_topic", vehicle_odom_topic_,
                        std::string("/localization/vehicle_odom"));
    private_node_.param("wheel_odom_topic", wheel_odom_topic_,
                        std::string("/tianracer/odom"));
    private_node_.param("imu_topic", imu_topic_,
                        std::string("/tianracer/imu"));
    private_node_.param("roi_cloud_topic", roi_cloud_topic_,
                        std::string("/raw_cloud_perception/roi_cloud"));
    private_node_.param("telemetry_topic", telemetry_topic_,
                        std::string("/roboracer_eth_h2h/mpcc/telemetry"));
    private_node_.param("integration_guard_enabled", integration_guard_enabled_,
                        false);
    private_node_.param("supervisor_state_topic", supervisor_state_topic_,
                        std::string("/automatic_relaunch_supervisor/state"));
    private_node_.param("planner_state_topic", planner_state_topic_,
                        std::string("/roboracer_eth_h2h/state"));
    private_node_.param("planner_status_topic", planner_status_topic_,
                        std::string("/roboracer_eth_h2h/status"));
    private_node_.param("planner_diagnostics_topic", planner_diagnostics_topic_,
                        std::string("/roboracer_eth_h2h/candidate_diagnostics"));
    private_node_.param("track_csv", track_csv_, std::string());

    private_node_.param("body_width_m", body_width_, 0.24);
    private_node_.param("boundary_margin_m", boundary_margin_, 0.10);
    private_node_.param("command_timeout_s", command_timeout_, 0.25);
    private_node_.param("odom_timeout_s", odom_timeout_, 0.25);
    private_node_.param("cloud_timeout_s", cloud_timeout_, 0.25);
    private_node_.param("contact_command_min_mps", contact_command_min_, 0.80);
    private_node_.param("stalled_speed_max_mps", stalled_speed_max_, 0.15);
    private_node_.param("stall_confirmation_s", stall_confirmation_, 0.18);
    private_node_.param("impact_ax_threshold_mps2", impact_ax_threshold_, -8.0);
    private_node_.param("impact_evidence_hold_s", impact_hold_, 1.0);
    private_node_.param("brake_minimum_s", brake_minimum_, 0.18);
    private_node_.param("brake_maximum_s", brake_maximum_, 0.75);
    private_node_.param("direction_change_dwell_s", direction_dwell_, 0.20);
    private_node_.param("reverse_speed_mps", reverse_speed_, 0.55);
    private_node_.param("reverse_steering_rad", reverse_steering_, 0.32);
    private_node_.param("reverse_maximum_s", reverse_maximum_time_, 0.70);
    private_node_.param("reverse_maximum_distance_m", reverse_maximum_distance_, 0.35);
    private_node_.param("rear_clearance_m", rear_clearance_, 0.90);
    private_node_.param("rear_half_width_m", rear_half_width_, 0.38);
    private_node_.param("rear_block_point_count", rear_block_point_count_, 3);
    private_node_.param("forward_recovery_speed_mps", forward_recovery_speed_, 1.0);
    private_node_.param("forward_recovery_steering_rad", forward_recovery_steering_, 0.32);
    private_node_.param("forward_recovery_maximum_s", forward_recovery_maximum_, 1.50);
    private_node_.param("release_raw_margin_m", release_margin_, 0.05);
    private_node_.param("release_heading_error_rad", release_heading_, 0.30);
    private_node_.param("release_confirmation_s", release_confirmation_, 0.20);
    private_node_.param("cooldown_s", cooldown_time_, 0.45);
    private_node_.param("cooldown_speed_cap_mps", cooldown_speed_cap_, 2.0);
    private_node_.param("integration_state_timeout_s", integration_state_timeout_,
                        0.35);
    private_node_.param("follow_obstacle_minimum_gap_m",
                        follow_obstacle_minimum_gap_, 0.15);
    double publish_rate = 60.0;
    private_node_.param("publish_rate_hz", publish_rate, 60.0);

    validateParameters(publish_rate);
    loadTrack();

    command_publisher_ =
        node_.advertise<ackermann_msgs::AckermannDrive>(output_topic_, 1);
    state_publisher_ = node_.advertise<std_msgs::String>(state_topic_, 5);
    command_subscriber_ = node_.subscribe(
        input_topic_, 1, &ContactEscapeGate::commandCallback, this,
        ros::TransportHints().tcpNoDelay());
    raw_mpcc_command_subscriber_ = node_.subscribe(
        raw_mpcc_command_topic_, 1,
        &ContactEscapeGate::rawMpccCommandCallback, this,
        ros::TransportHints().tcpNoDelay());
    vehicle_odom_subscriber_ = node_.subscribe(
        vehicle_odom_topic_, 5, &ContactEscapeGate::vehicleOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
    wheel_odom_subscriber_ = node_.subscribe(
        wheel_odom_topic_, 5, &ContactEscapeGate::wheelOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
    imu_subscriber_ = node_.subscribe(
        imu_topic_, 20, &ContactEscapeGate::imuCallback, this,
        ros::TransportHints().tcpNoDelay());
    roi_subscriber_ = node_.subscribe(
        roi_cloud_topic_, 1, &ContactEscapeGate::roiCallback, this,
        ros::TransportHints().tcpNoDelay());
    telemetry_subscriber_ = node_.subscribe(
        telemetry_topic_, 5, &ContactEscapeGate::telemetryCallback, this,
        ros::TransportHints().tcpNoDelay());
    if (integration_guard_enabled_) {
      supervisor_state_subscriber_ = node_.subscribe(
          supervisor_state_topic_, 5,
          &ContactEscapeGate::supervisorStateCallback, this,
          ros::TransportHints().tcpNoDelay());
      planner_state_subscriber_ = node_.subscribe(
          planner_state_topic_, 5,
          &ContactEscapeGate::plannerStateCallback, this,
          ros::TransportHints().tcpNoDelay());
      planner_status_subscriber_ = node_.subscribe(
          planner_status_topic_, 5,
          &ContactEscapeGate::plannerStatusCallback, this,
          ros::TransportHints().tcpNoDelay());
      planner_diagnostics_subscriber_ = node_.subscribe(
          planner_diagnostics_topic_, 5,
          &ContactEscapeGate::plannerDiagnosticsCallback, this,
          ros::TransportHints().tcpNoDelay());
    }
    timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate),
                               &ContactEscapeGate::timerCallback, this);
    publishZero();
    ROS_WARN_STREAM("ROBORACER contact escape gate ready input=" << input_topic_
                    << " output=" << output_topic_
                    << " reverse=" << reverse_speed_ << "m/s max="
                    << reverse_maximum_distance_ << "m integration_guard="
                    << (integration_guard_enabled_ ? "true" : "false")
                    << "; normal H2H commands pass unchanged");
  }

  ~ContactEscapeGate() { publishZero(); }

 private:
  void validateParameters(double publish_rate) const {
    const bool valid = publish_rate > 0.0 && command_timeout_ > 0.0 &&
        odom_timeout_ > 0.0 && cloud_timeout_ > 0.0 &&
        body_width_ > 0.0 && boundary_margin_ >= 0.0 &&
        contact_command_min_ > 0.0 && stalled_speed_max_ > 0.0 &&
        stall_confirmation_ > 0.0 && impact_ax_threshold_ < 0.0 &&
        brake_minimum_ >= 0.0 && brake_maximum_ >= brake_minimum_ &&
        direction_dwell_ >= 0.0 && reverse_speed_ > 0.0 &&
        reverse_maximum_time_ > 0.0 && reverse_maximum_distance_ > 0.0 &&
        rear_clearance_ > 0.0 && rear_half_width_ > 0.0 &&
        rear_block_point_count_ > 0 && forward_recovery_speed_ > 0.0 &&
        forward_recovery_maximum_ > 0.0 && release_confirmation_ >= 0.0 &&
        cooldown_time_ >= 0.0 && cooldown_speed_cap_ > 0.0 &&
        integration_state_timeout_ > 0.0 &&
        follow_obstacle_minimum_gap_ >= 0.0;
    if (!valid || track_csv_.empty()) {
      throw std::runtime_error("invalid contact escape parameters");
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

  void rawMpccCommandCallback(
      const ackermann_msgs::AckermannDriveStamped::ConstPtr& message) {
    if (!std::isfinite(message->drive.speed)) {
      raw_mpcc_command_.reset();
      return;
    }
    raw_mpcc_command_ = message->drive;
    raw_mpcc_command_wall_ = ros::WallTime::now();
  }

  void vehicleOdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const auto& p = message->pose.pose.position;
    const auto& q = message->pose.pose.orientation;
    vehicle_x_ = p.x;
    vehicle_y_ = p.y;
    vehicle_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    raw_speed_ = message->twist.twist.linear.x;
    vehicle_odom_wall_ = ros::WallTime::now();
    raw_track_ = projectRawTrack();
  }

  void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    wheel_speed_ = message->twist.twist.linear.x;
    wheel_odom_wall_ = ros::WallTime::now();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    if (std::isfinite(message->linear_acceleration.x) &&
        message->linear_acceleration.x <= impact_ax_threshold_) {
      impact_wall_ = ros::WallTime::now();
      impact_ax_ = message->linear_acceleration.x;
    }
  }

  void telemetryCallback(const std_msgs::Float32MultiArray::ConstPtr& message) {
    if (message->data.size() > 72) {
      pp_risk_ = message->data[72] > 0.5f;
    }
    telemetry_wall_ = ros::WallTime::now();
  }

  void supervisorStateCallback(const std_msgs::String::ConstPtr& message) {
    supervisor_running_ = jsonState(message->data, "RUNNING");
    supervisor_state_wall_ = ros::WallTime::now();
  }

  void plannerStateCallback(const std_msgs::String::ConstPtr& message) {
    planner_follow_ = message->data == "FOLLOW";
    planner_state_wall_ = ros::WallTime::now();
  }

  void plannerStatusCallback(const std_msgs::String::ConstPtr& message) {
    planner_status_follow_ = jsonState(message->data, "FOLLOW");
    planner_status_healthy_ = jsonTrue(message->data, "healthy");
    planner_status_wall_ = ros::WallTime::now();
  }

  void plannerDiagnosticsCallback(
      const std_msgs::String::ConstPtr& message) {
    planner_obstacle_relevant_ = jsonTrue(message->data, "relevant");
    planner_bumper_gap_ = jsonNumber(message->data, "bumper_gap_m");
    planner_diagnostics_wall_ = ros::WallTime::now();
  }

  void roiCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    int count = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      for (; x != x.end(); ++x, ++y) {
        if (!std::isfinite(*x) || !std::isfinite(*y)) continue;
        if (*x <= -0.15f && *x >= -static_cast<float>(rear_clearance_) &&
            std::abs(*y) <= rear_half_width_) {
          ++count;
          if (count >= rear_block_point_count_) break;
        }
      }
    } catch (const std::runtime_error&) {
      count = rear_block_point_count_;
    }
    rear_block_points_ = count;
    roi_wall_ = ros::WallTime::now();
  }

  RawTrackState projectRawTrack() const {
    RawTrackState output;
    if (track_.empty() || !std::isfinite(vehicle_x_) ||
        !std::isfinite(vehicle_y_) || !std::isfinite(vehicle_yaw_)) {
      return output;
    }
    std::size_t nearest = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < track_.size(); ++i) {
      const double dx = vehicle_x_ - track_[i].x;
      const double dy = vehicle_y_ - track_[i].y;
      const double distance = dx * dx + dy * dy;
      if (distance < best_distance) {
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
    output.valid = std::isfinite(output.margin) && best_distance < 9.0;
    return output;
  }

  bool inputsFresh() const {
    return command_.has_value() && age(command_wall_) <= command_timeout_ &&
        age(vehicle_odom_wall_) <= odom_timeout_ &&
        age(wheel_odom_wall_) <= odom_timeout_ && raw_track_.valid;
  }

  bool rearClear() const {
    return age(roi_wall_) <= cloud_timeout_ &&
        rear_block_points_ < rear_block_point_count_;
  }

  bool supervisorRunning() const {
    return !integration_guard_enabled_ ||
        (supervisor_running_ &&
         age(supervisor_state_wall_) <= integration_state_timeout_);
  }

  bool confirmedFollowObstacle() const {
    if (!integration_guard_enabled_) return false;
    const bool streams_fresh =
        age(planner_state_wall_) <= integration_state_timeout_ &&
        age(planner_status_wall_) <= integration_state_timeout_ &&
        age(planner_diagnostics_wall_) <= integration_state_timeout_;
    return streams_fresh && planner_follow_ && planner_status_follow_ &&
        planner_status_healthy_ && planner_obstacle_relevant_ &&
        planner_bumper_gap_.has_value() &&
        *planner_bumper_gap_ >= follow_obstacle_minimum_gap_ &&
        raw_track_.valid && raw_track_.margin >= 0.0;
  }

  bool contactEvidence() const {
    const bool impact = age(impact_wall_) <= impact_hold_;
    const bool telemetry_risk = age(telemetry_wall_) <= command_timeout_ && pp_risk_;
    const bool outside_track = raw_track_.valid && raw_track_.margin < 0.0;
    // A healthy FOLLOW behind a detected opponent can legitimately be slow.
    // In the integrated fast-start candidate, PP risk by itself must not turn
    // that situation into a reverse maneuver. A hard impact or an actually
    // negative raw track margin remains sufficient evidence.
    return impact || outside_track ||
        (telemetry_risk && !confirmedFollowObstacle());
  }

  bool safetyStopRequested() const {
    const bool raw_command_fresh = raw_mpcc_command_.has_value() &&
        age(raw_mpcc_command_wall_) <= command_timeout_;
    const bool track_risk = (age(telemetry_wall_) <= command_timeout_ && pp_risk_) ||
        (raw_track_.valid && raw_track_.margin < 0.0);
    return raw_command_fresh && raw_mpcc_command_->speed <= 0.05f && track_risk;
  }

  bool stalled() const {
    return command_.has_value() && command_->speed >= contact_command_min_ &&
        std::abs(raw_speed_) <= stalled_speed_max_ &&
        std::abs(wheel_speed_) <= stalled_speed_max_;
  }

  void transition(EscapeState next) {
    state_ = next;
    state_wall_ = ros::WallTime::now();
    release_wall_ = ros::WallTime();
    if (next == EscapeState::kReverse) {
      reverse_start_x_ = vehicle_x_;
      reverse_start_y_ = vehicle_y_;
    }
    ROS_WARN("ROBORACER contact escape transition -> %s raw_margin=%.3f raw_e=%.3f raw_v=%.3f wheel_v=%.3f rear_clear=%s",
             stateName(next), raw_track_.margin, raw_track_.lateral,
             raw_speed_, wheel_speed_, rearClear() ? "true" : "false");
  }

  ackermann_msgs::AckermannDrive zeroCommand() const {
    return ackermann_msgs::AckermannDrive{};
  }

  double reverseSteering() const {
    if (!raw_track_.valid || std::abs(raw_track_.lateral) < 0.15) return 0.0;
    return std::copysign(reverse_steering_, raw_track_.lateral);
  }

  double forwardRecoverySteering() const {
    if (!raw_track_.valid || std::abs(raw_track_.lateral) < 0.08) {
      return command_.has_value() ? command_->steering_angle : 0.0;
    }
    return -std::copysign(forward_recovery_steering_, raw_track_.lateral);
  }

  void publishZero() { command_publisher_.publish(zeroCommand()); }

  void publishState() {
    std_msgs::String message;
    std::ostringstream stream;
    stream << "{\"state\":\"" << stateName(state_)
           << "\",\"intervening\":"
           << (state_ == EscapeState::kForwarding ? "false" : "true")
           << ",\"raw_margin_m\":" << raw_track_.margin
           << ",\"raw_lateral_m\":" << raw_track_.lateral
           << ",\"raw_speed_mps\":" << raw_speed_
           << ",\"wheel_speed_mps\":" << wheel_speed_
           << ",\"rear_clear\":" << (rearClear() ? "true" : "false")
           << ",\"safety_zero_override\":"
           << (safetyStopRequested() ? "true" : "false")
           << ",\"supervisor_running\":"
           << (supervisorRunning() ? "true" : "false")
           << ",\"confirmed_follow_obstacle\":"
           << (confirmedFollowObstacle() ? "true" : "false")
           << ",\"rear_block_points\":" << rear_block_points_
           << ",\"impact_ax_mps2\":" << impact_ax_ << "}";
    message.data = stream.str();
    state_publisher_.publish(message);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!inputsFresh()) {
      stall_wall_ = ros::WallTime();
      publishZero();
      publishState();
      return;
    }

    const ros::WallTime now = ros::WallTime::now();
    bool supervisor_abort = false;
    if (integration_guard_enabled_ &&
        state_ != EscapeState::kForwarding && !supervisorRunning()) {
      ROS_ERROR("ROBORACER contact escape aborted: fast-start supervisor left RUNNING");
      transition(EscapeState::kForwarding);
      stall_wall_ = ros::WallTime();
      supervisor_abort = true;
    }
    if (state_ == EscapeState::kForwarding) {
      if (supervisorRunning() && stalled() && contactEvidence()) {
        if (stall_wall_.toSec() <= 0.0) stall_wall_ = now;
        if ((now - stall_wall_).toSec() >= stall_confirmation_) {
          transition(EscapeState::kBrake);
          stall_wall_ = ros::WallTime();
        }
      } else {
        stall_wall_ = ros::WallTime();
      }
    }

    ackermann_msgs::AckermannDrive output = *command_;
    const double state_age = age(state_wall_);
    if (supervisor_abort) output = zeroCommand();
    switch (state_) {
      case EscapeState::kForwarding:
        // The upstream command gate provides the accepted racing minimum
        // speed, but it cannot distinguish Follow from an MPCC/OOB emergency
        // zero. Preserve the floor everywhere except a fresh, explicit MPCC
        // safety stop with independent track-risk evidence.
        if (supervisor_abort || safetyStopRequested()) {
          output = zeroCommand();
          ROS_ERROR_THROTTLE(0.5,
              "ROBORACER contact gate preserving MPCC/OOB safety zero; bypassing 1m/s Follow floor raw_margin=%.3f",
              raw_track_.margin);
        }
        break;
      case EscapeState::kBrake:
        output = zeroCommand();
        if ((state_age >= brake_minimum_ &&
             std::abs(raw_speed_) < 0.10 && std::abs(wheel_speed_) < 0.10) ||
            state_age >= brake_maximum_) {
          transition(EscapeState::kReverseDwell);
        }
        break;
      case EscapeState::kReverseDwell:
        output = zeroCommand();
        if (state_age >= direction_dwell_) {
          transition(rearClear() ? EscapeState::kReverse
                                 : EscapeState::kRearBlocked);
        }
        break;
      case EscapeState::kRearBlocked:
        output = zeroCommand();
        if (rearClear()) transition(EscapeState::kReverse);
        break;
      case EscapeState::kReverse: {
        output = zeroCommand();
        output.speed = -reverse_speed_;
        output.steering_angle = reverseSteering();
        const double distance = std::hypot(vehicle_x_ - reverse_start_x_,
                                           vehicle_y_ - reverse_start_y_);
        if (!rearClear() || distance >= reverse_maximum_distance_ ||
            state_age >= reverse_maximum_time_) {
          transition(EscapeState::kForwardDwell);
          output = zeroCommand();
        }
        break;
      }
      case EscapeState::kForwardDwell:
        output = zeroCommand();
        if (state_age >= direction_dwell_) {
          transition(EscapeState::kForwardRecovery);
        }
        break;
      case EscapeState::kForwardRecovery: {
        output = zeroCommand();
        output.speed = static_cast<float>(std::min(
            forward_recovery_speed_,
            static_cast<double>(std::max(0.0f, command_->speed))));
        output.steering_angle = forwardRecoverySteering();
        const bool release_safe = raw_track_.margin >= release_margin_ &&
            std::abs(raw_track_.heading_error) <= release_heading_;
        if (release_safe) {
          if (release_wall_.toSec() <= 0.0) release_wall_ = now;
          if ((now - release_wall_).toSec() >= release_confirmation_) {
            transition(EscapeState::kCooldown);
          }
        } else {
          release_wall_ = ros::WallTime();
        }
        if (state_age >= forward_recovery_maximum_ && !release_safe) {
          transition(EscapeState::kRecoveryFailed);
          output = zeroCommand();
        }
        break;
      }
      case EscapeState::kCooldown:
        output.speed = std::min(output.speed,
                                static_cast<float>(cooldown_speed_cap_));
        if (state_age >= cooldown_time_) {
          transition(EscapeState::kForwarding);
          impact_wall_ = ros::WallTime();
        }
        break;
      case EscapeState::kRecoveryFailed:
        output = zeroCommand();
        break;
    }
    command_publisher_.publish(output);
    publishState();
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher command_publisher_;
  ros::Publisher state_publisher_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber raw_mpcc_command_subscriber_;
  ros::Subscriber vehicle_odom_subscriber_;
  ros::Subscriber wheel_odom_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Subscriber roi_subscriber_;
  ros::Subscriber telemetry_subscriber_;
  ros::Subscriber supervisor_state_subscriber_;
  ros::Subscriber planner_state_subscriber_;
  ros::Subscriber planner_status_subscriber_;
  ros::Subscriber planner_diagnostics_subscriber_;
  ros::Timer timer_;

  std::optional<ackermann_msgs::AckermannDrive> command_;
  std::optional<ackermann_msgs::AckermannDrive> raw_mpcc_command_;
  std::optional<double> planner_bumper_gap_;
  std::vector<TrackPoint> track_;
  RawTrackState raw_track_;
  EscapeState state_{EscapeState::kForwarding};
  ros::WallTime command_wall_;
  ros::WallTime raw_mpcc_command_wall_;
  ros::WallTime vehicle_odom_wall_;
  ros::WallTime wheel_odom_wall_;
  ros::WallTime roi_wall_;
  ros::WallTime telemetry_wall_;
  ros::WallTime impact_wall_;
  ros::WallTime supervisor_state_wall_;
  ros::WallTime planner_state_wall_;
  ros::WallTime planner_status_wall_;
  ros::WallTime planner_diagnostics_wall_;
  ros::WallTime stall_wall_;
  ros::WallTime state_wall_{ros::WallTime::now()};
  ros::WallTime release_wall_;

  std::string input_topic_;
  std::string output_topic_;
  std::string state_topic_;
  std::string raw_mpcc_command_topic_;
  std::string vehicle_odom_topic_;
  std::string wheel_odom_topic_;
  std::string imu_topic_;
  std::string roi_cloud_topic_;
  std::string telemetry_topic_;
  std::string supervisor_state_topic_;
  std::string planner_state_topic_;
  std::string planner_status_topic_;
  std::string planner_diagnostics_topic_;
  std::string track_csv_;
  double vehicle_x_{std::numeric_limits<double>::quiet_NaN()};
  double vehicle_y_{std::numeric_limits<double>::quiet_NaN()};
  double vehicle_yaw_{std::numeric_limits<double>::quiet_NaN()};
  double raw_speed_{};
  double wheel_speed_{};
  double reverse_start_x_{};
  double reverse_start_y_{};
  double impact_ax_{};
  int rear_block_points_{};
  bool pp_risk_{false};
  bool integration_guard_enabled_{false};
  bool supervisor_running_{false};
  bool planner_follow_{false};
  bool planner_status_follow_{false};
  bool planner_status_healthy_{false};
  bool planner_obstacle_relevant_{false};

  double body_width_{};
  double boundary_margin_{};
  double command_timeout_{};
  double odom_timeout_{};
  double cloud_timeout_{};
  double contact_command_min_{};
  double stalled_speed_max_{};
  double stall_confirmation_{};
  double impact_ax_threshold_{};
  double impact_hold_{};
  double brake_minimum_{};
  double brake_maximum_{};
  double direction_dwell_{};
  double reverse_speed_{};
  double reverse_steering_{};
  double reverse_maximum_time_{};
  double reverse_maximum_distance_{};
  double rear_clearance_{};
  double rear_half_width_{};
  int rear_block_point_count_{};
  double forward_recovery_speed_{};
  double forward_recovery_steering_{};
  double forward_recovery_maximum_{};
  double release_margin_{};
  double release_heading_{};
  double release_confirmation_{};
  double cooldown_time_{};
  double cooldown_speed_cap_{};
  double integration_state_timeout_{};
  double follow_obstacle_minimum_gap_{};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "roboracer_contact_escape_gate_cpp");
  try {
    ContactEscapeGate gate;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("ROBORACER contact escape gate failed: " << error.what());
    return 1;
  }
  return 0;
}
