#include "roboracer_react_overtake/react_core.hpp"

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace roboracer_react_overtake {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

std::string normalizedFrame(std::string frame) {
  while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  return frame;
}

double wallAge(const ros::WallTime& stamp) {
  if (stamp.isZero()) return kInf;
  return std::max(0.0, (ros::WallTime::now() - stamp).toSec());
}

double rosAge(const ros::Time& stamp) {
  const auto now = ros::Time::now();
  if (stamp.isZero() || now.isZero()) return kInf;
  return (now - stamp).toSec();
}

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

enum class State { kGlobal, kFollow, kPass, kReturn, kAbort };

const char* stateName(State state) {
  switch (state) {
    case State::kGlobal: return "GLOBAL";
    case State::kFollow: return "FOLLOW";
    case State::kPass: return "PASS";
    case State::kReturn: return "RETURN";
    case State::kAbort: return "ABORT";
  }
  return "ABORT";
}

struct OdomState {
  ros::Time stamp;
  double x{};
  double y{};
  double yaw{};
  double vx{};
};

struct StoredOccupancy {
  double s{};
  double ey{};
  double half_q{};
  double half_ey{};
  double z_min{};
  double z_max{};
  std::size_t point_count{};
};

class ReactOvertakeNode {
 public:
  ReactOvertakeNode() : private_node_("~") {
    loadParameters();
    track_ = std::make_unique<PeriodicTrack>(raceline_csv_);
    odom_subscriber_ = node_.subscribe(
        odom_topic_, 100, &ReactOvertakeNode::odomCallback, this,
        ros::TransportHints().tcpNoDelay());
    cloud_subscriber_ = node_.subscribe(
        cloud_topic_, 2, &ReactOvertakeNode::cloudCallback, this,
        ros::TransportHints().tcpNoDelay());
    local_reference_publisher_ =
        node_.advertise<nav_msgs::Path>(local_reference_topic_, 2);
    red_path_publisher_ = node_.advertise<nav_msgs::Path>(red_path_topic_, 2);
    selected_path_publisher_ =
        node_.advertise<nav_msgs::Path>(selected_path_topic_, 2);
    left_path_publisher_ = node_.advertise<nav_msgs::Path>(left_path_topic_, 2);
    right_path_publisher_ = node_.advertise<nav_msgs::Path>(right_path_topic_, 2);
    valid_area_publisher_ =
        node_.advertise<geometry_msgs::PolygonStamped>(valid_area_topic_, 2);
    occupancy_marker_publisher_ =
        node_.advertise<visualization_msgs::MarkerArray>(occupancy_marker_topic_, 2);
    speed_cap_publisher_ = node_.advertise<std_msgs::Float32>(speed_cap_topic_, 5);
    state_publisher_ = node_.advertise<std_msgs::String>(state_topic_, 5);
    health_publisher_ = node_.advertise<std_msgs::Bool>(health_topic_, 5);
    status_publisher_ = node_.advertise<std_msgs::String>(status_topic_, 5);
    diagnostics_publisher_ =
        node_.advertise<std_msgs::String>(diagnostics_topic_, 5);
    timer_ = node_.createTimer(ros::Duration(1.0 / update_rate_hz_),
                               &ReactOvertakeNode::timerCallback, this);
    ROS_INFO_STREAM("V3 Pro Max React planner ready: ROI=" << cloud_topic_
                    << " odom=" << odom_topic_
                    << " motion/static classification=disabled");
  }

 private:
  template <typename T>
  void parameter(const std::string& name, T& value, const T& fallback) {
    private_node_.param(name, value, fallback);
  }

  void loadParameters() {
    if (!private_node_.getParam("raceline_csv", raceline_csv_)) {
      throw std::runtime_error("raceline_csv is required");
    }
    parameter("topics/odom", odom_topic_,
              std::string("/localization/odom"));
    parameter("topics/roi_cloud", cloud_topic_,
              std::string("/raw_cloud_perception/roi_cloud"));
    parameter("topics/local_reference", local_reference_topic_,
              std::string("/roboracer_react/local_reference"));
    parameter("topics/local_plan_red", red_path_topic_,
              std::string("/roboracer_react/local_plan_red"));
    parameter("topics/selected_path", selected_path_topic_,
              std::string("/roboracer_react/selected_path"));
    parameter("topics/candidate_left", left_path_topic_,
              std::string("/roboracer_react/candidate_left"));
    parameter("topics/candidate_right", right_path_topic_,
              std::string("/roboracer_react/candidate_right"));
    parameter("topics/valid_area", valid_area_topic_,
              std::string("/roboracer_react/valid_area"));
    parameter("topics/occupancy_markers", occupancy_marker_topic_,
              std::string("/roboracer_react/occupancy_markers"));
    parameter("topics/speed_cap", speed_cap_topic_,
              std::string("/roboracer_react/local_speed_cap"));
    parameter("topics/state", state_topic_,
              std::string("/roboracer_react/state"));
    parameter("topics/health", health_topic_,
              std::string("/roboracer_react/health"));
    parameter("topics/status", status_topic_,
              std::string("/roboracer_react/status"));
    parameter("topics/diagnostics", diagnostics_topic_,
              std::string("/roboracer_react/diagnostics"));
    parameter("frames/world", world_frame_, std::string("map"));
    if (!private_node_.getParam("frames/allowed_roi", allowed_roi_frames_)) {
      allowed_roi_frames_ = {"raw_body_leveled"};
    }
    for (auto& frame : allowed_roi_frames_) frame = normalizedFrame(frame);

    parameter("input/odom_timeout_s", odom_timeout_s_, 0.30);
    parameter("input/cloud_timeout_s", cloud_timeout_s_, 0.50);
    parameter("input/cloud_measurement_max_age_s",
              cloud_measurement_max_age_s_, 0.80);
    parameter("input/cloud_future_tolerance_s", cloud_future_tolerance_s_, 0.05);
    parameter("input/odom_history_duration_s", odom_history_duration_s_, 2.0);
    parameter("input/maximum_odom_bracket_gap_s",
              maximum_odom_bracket_gap_s_, 0.30);
    parameter("input/maximum_odom_extrapolation_s",
              maximum_odom_extrapolation_s_, 0.12);

    parameter("roi/minimum_q", config_.minimum_q, 0.35);
    parameter("roi/maximum_q", config_.maximum_q, 5.50);
    parameter("roi/trigger_distance", config_.trigger_distance, 4.00);
    parameter("roi/boundary_strip", config_.boundary_strip, 0.10);
    parameter("roi/voxel_size", config_.voxel_size, 0.08);
    parameter("roi/minimum_component_points",
              config_.minimum_component_points, 5);
    parameter("roi/minimum_obstacle_top_z",
              config_.minimum_obstacle_top_z, 0.02);
    parameter("planner/ego_width", config_.ego_width, 0.24);
    parameter("planner/ego_length", config_.ego_length, 0.40);
    parameter("planner/lateral_clearance", config_.lateral_clearance, 0.08);
    parameter("planner/candidate_occupancy_buffer",
              config_.candidate_occupancy_buffer, 0.03);
    parameter("planner/longitudinal_clearance",
              config_.longitudinal_clearance, 0.20);
    parameter("planner/boundary_margin", config_.boundary_margin, 0.08);
    parameter("planner/minimum_boundary_clearance",
              config_.minimum_boundary_clearance, 0.03);
    parameter("planner/minimum_lane_change_length",
              config_.minimum_lane_change_length, 0.85);
    parameter("planner/nominal_lane_change_length",
              config_.nominal_lane_change_length, 1.60);
    parameter("planner/pass_hold_distance",
              config_.pass_hold_distance, 0.70);
    parameter("planner/return_length", config_.return_length, 1.80);
    parameter("planner/maximum_candidate_length",
              config_.maximum_candidate_length, 8.00);
    parameter("planner/maximum_path_curvature",
              config_.maximum_path_curvature, 1.80);
    parameter("planner/maximum_track_curvature_for_pass",
              config_.maximum_track_curvature_for_pass, 1.20);
    parameter("planner/preferred_side", config_.preferred_side,
              std::string("left"));
    parameter("planner/sample_ds", sample_ds_, 0.10);
    parameter("planner/post_disappearance_clearance_m",
              post_disappearance_clearance_m_, 0.65);
    parameter("planner/committed_path_hold_s", committed_path_hold_s_, 0.60);
    parameter("planner/return_start_ey_m", return_start_ey_m_, 0.12);
    parameter("planner/return_finish_ey_m", return_finish_ey_m_, 0.06);
    parameter("control/update_rate_hz", update_rate_hz_, 20.0);
    parameter("control/cruise_speed_cap_mps", cruise_speed_cap_mps_, 4.0);
    parameter("control/pass_speed_cap_mps", pass_speed_cap_mps_, 2.20);
    parameter("control/return_speed_cap_mps", return_speed_cap_mps_, 2.00);
    parameter("control/blocked_minimum_speed_mps",
              blocked_minimum_speed_mps_, 1.00);
    parameter("control/blocked_maximum_speed_mps",
              blocked_maximum_speed_mps_, 1.00);
    parameter("control/blocked_stop_gap_m", blocked_stop_gap_m_, 0.45);
    parameter("control/blocked_gap_gain", blocked_gap_gain_, 0.80);

    config_.validate();
    if (allowed_roi_frames_.empty() || update_rate_hz_ <= 0.0 ||
        sample_ds_ <= 0.0 || odom_timeout_s_ <= 0.0 ||
        cloud_timeout_s_ <= 0.0 || cloud_measurement_max_age_s_ <= 0.0 ||
        maximum_odom_bracket_gap_s_ <= 0.0 ||
        maximum_odom_extrapolation_s_ < 0.0 ||
        post_disappearance_clearance_m_ < 0.0 || committed_path_hold_s_ < 0.0 ||
        return_finish_ey_m_ < 0.0 || return_start_ey_m_ < return_finish_ey_m_ ||
        blocked_minimum_speed_mps_ < 0.0 ||
        blocked_maximum_speed_mps_ < blocked_minimum_speed_mps_) {
      throw std::runtime_error("invalid V3 React runtime configuration");
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    if (message->header.stamp.isZero()) {
      last_input_error_ = "zero_odom_stamp";
      return;
    }
    OdomState state;
    state.stamp = message->header.stamp;
    state.x = message->pose.pose.position.x;
    state.y = message->pose.pose.position.y;
    state.yaw = yawFromQuaternion(message->pose.pose.orientation);
    state.vx = message->twist.twist.linear.x;
    if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.yaw) || !std::isfinite(state.vx)) {
      last_input_error_ = "non_finite_odom";
      return;
    }
    if (!odom_history_.empty() && state.stamp < odom_history_.back().stamp &&
        (odom_history_.back().stamp - state.stamp).toSec() > 0.001) {
      last_input_error_ = "out_of_order_odom";
      return;
    }
    odom_history_.push_back(state);
    const ros::Time cutoff = state.stamp - ros::Duration(odom_history_duration_s_);
    while (odom_history_.size() > 2 && odom_history_.front().stamp < cutoff) {
      odom_history_.pop_front();
    }
    latest_odom_ = state;
    latest_odom_wall_ = ros::WallTime::now();
  }

  std::optional<OdomState> odomAt(const ros::Time& stamp) const {
    if (odom_history_.empty()) return std::nullopt;
    if (stamp <= odom_history_.front().stamp) {
      if ((odom_history_.front().stamp - stamp).toSec() <=
          maximum_odom_extrapolation_s_) return odom_history_.front();
      return std::nullopt;
    }
    if (stamp >= odom_history_.back().stamp) {
      const double dt = (stamp - odom_history_.back().stamp).toSec();
      if (dt > maximum_odom_extrapolation_s_) return std::nullopt;
      OdomState result = odom_history_.back();
      result.stamp = stamp;
      result.x += dt * std::cos(result.yaw) * result.vx;
      result.y += dt * std::sin(result.yaw) * result.vx;
      return result;
    }
    for (std::size_t i = 1; i < odom_history_.size(); ++i) {
      if (odom_history_[i].stamp < stamp) continue;
      const auto& lower = odom_history_[i - 1];
      const auto& upper = odom_history_[i];
      const double gap = (upper.stamp - lower.stamp).toSec();
      if (gap <= 0.0 || gap > maximum_odom_bracket_gap_s_) return std::nullopt;
      const double ratio = (stamp - lower.stamp).toSec() / gap;
      OdomState result;
      result.stamp = stamp;
      result.x = lower.x + ratio * (upper.x - lower.x);
      result.y = lower.y + ratio * (upper.y - lower.y);
      result.yaw = wrapAngle(
          lower.yaw + ratio * wrapAngle(upper.yaw - lower.yaw));
      result.vx = lower.vx + ratio * (upper.vx - lower.vx);
      return result;
    }
    return std::nullopt;
  }

  std::vector<CorridorSample> corridorAt(
      const OdomState& odom, bool local_frame) const {
    const auto projection = track_->project(odom.x, odom.y);
    const int count = static_cast<int>(
        std::ceil(config_.maximum_candidate_length / sample_ds_));
    std::vector<CorridorSample> result;
    result.reserve(static_cast<std::size_t>(count + 1));
    for (int i = 0; i <= count; ++i) {
      const double q = std::min(config_.maximum_candidate_length,
                                static_cast<double>(i) * sample_ds_);
      const auto geometry = track_->sample(projection.s + q);
      double x = geometry.x;
      double y = geometry.y;
      double yaw = geometry.yaw;
      if (local_frame) {
        const double dx = geometry.x - odom.x;
        const double dy = geometry.y - odom.y;
        x = std::cos(odom.yaw) * dx + std::sin(odom.yaw) * dy;
        y = -std::sin(odom.yaw) * dx + std::cos(odom.yaw) * dy;
        yaw = wrapAngle(geometry.yaw - odom.yaw);
      }
      result.push_back({q, geometry.s, x, y, yaw, geometry.curvature,
                        geometry.width_right, geometry.width_left});
    }
    return result;
  }

  bool frameAllowed(const std::string& frame) const {
    return std::find(allowed_roi_frames_.begin(), allowed_roi_frames_.end(),
                     normalizedFrame(frame)) != allowed_roi_frames_.end();
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    last_cloud_received_wall_ = ros::WallTime::now();
    if (!frameAllowed(message->header.frame_id)) {
      last_input_error_ = "unexpected_roi_frame:" +
                          normalizedFrame(message->header.frame_id);
      return;
    }
    const double age = rosAge(message->header.stamp);
    if (!std::isfinite(age) || age > cloud_measurement_max_age_s_ ||
        age < -cloud_future_tolerance_s_) {
      last_input_error_ = "stale_or_future_roi_cloud";
      return;
    }
    const auto measurement_odom = odomAt(message->header.stamp);
    if (!measurement_odom.has_value()) {
      last_input_error_ = "roi_cloud_without_synchronized_odom";
      return;
    }
    try {
      std::vector<RoiPoint> points;
      points.reserve(message->width * message->height);
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        points.push_back({static_cast<double>(*x), static_cast<double>(*y),
                          static_cast<double>(*z)});
      }
      const auto local_corridor = corridorAt(*measurement_odom, true);
      const auto extracted = extractOccupancy(points, local_corridor, config_);
      const auto projection = track_->project(
          measurement_odom->x, measurement_odom->y);
      stored_occupancy_.clear();
      stored_occupancy_.reserve(extracted.size());
      for (const auto& obstacle : extracted) {
        stored_occupancy_.push_back({
            track_->wrapS(projection.s + obstacle.q), obstacle.ey,
            std::max(0.5 * config_.voxel_size,
                     0.5 * (obstacle.q_max - obstacle.q_min)),
            std::max(0.5 * config_.voxel_size,
                     0.5 * (obstacle.ey_max - obstacle.ey_min)),
            obstacle.z_min, obstacle.z_max, obstacle.point_count});
      }
      latest_cloud_stamp_ = message->header.stamp;
      latest_cloud_wall_ = ros::WallTime::now();
      ++accepted_cloud_sequence_;
      last_input_error_ = "none";
    } catch (const std::exception& error) {
      last_input_error_ = std::string("roi_decode_error:") + error.what();
    }
  }

  double signedTrackDelta(double target, double origin) const {
    double delta = track_->wrapS(target - origin);
    if (delta > 0.5 * track_->length()) delta -= track_->length();
    return delta;
  }

  std::vector<OccupancyComponent> currentOccupancy(double current_s) const {
    std::vector<OccupancyComponent> result;
    result.reserve(stored_occupancy_.size());
    for (const auto& stored : stored_occupancy_) {
      OccupancyComponent obstacle;
      obstacle.q = signedTrackDelta(stored.s, current_s);
      obstacle.ey = stored.ey;
      obstacle.q_min = obstacle.q - stored.half_q;
      obstacle.q_max = obstacle.q + stored.half_q;
      obstacle.ey_min = obstacle.ey - stored.half_ey;
      obstacle.ey_max = obstacle.ey + stored.half_ey;
      obstacle.z_min = stored.z_min;
      obstacle.z_max = stored.z_max;
      obstacle.point_count = stored.point_count;
      result.push_back(obstacle);
    }
    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b) { return a.q_min < b.q_min; });
    return result;
  }

  bool inputsHealthy() const {
    if (!latest_odom_.has_value() || latest_cloud_stamp_.isZero()) return false;
    if (wallAge(latest_odom_wall_) > odom_timeout_s_ ||
        wallAge(latest_cloud_wall_) > cloud_timeout_s_) return false;
    const double age = rosAge(latest_cloud_stamp_);
    return std::isfinite(age) && age <= cloud_measurement_max_age_s_ &&
           age >= -cloud_future_tolerance_s_;
  }

  bool currentLaneClear(const std::vector<OccupancyComponent>& occupancy,
                        double ego_ey) const {
    const double lateral_padding =
        0.5 * config_.ego_width + config_.lateral_clearance;
    const double longitudinal_padding =
        0.5 * config_.ego_length + config_.longitudinal_clearance;
    for (const auto& obstacle : occupancy) {
      if (obstacle.q_max < -longitudinal_padding ||
          obstacle.q_min > config_.trigger_distance) continue;
      if (ego_ey > obstacle.ey_min - lateral_padding &&
          ego_ey < obstacle.ey_max + lateral_padding) return false;
    }
    return true;
  }

  double holdDistanceForOccupancy(
      const std::vector<OccupancyComponent>& occupancy) const {
    double hold = 0.0;
    const double longitudinal_padding =
        0.5 * config_.ego_length + config_.longitudinal_clearance;
    for (const auto& obstacle : occupancy) {
      hold = std::max(hold, obstacle.q_max + longitudinal_padding +
                             config_.pass_hold_distance);
    }
    return clamp(hold, 0.0, config_.maximum_candidate_length -
                             config_.return_length);
  }

  void transition(State next, const std::string& reason) {
    if (state_ == next && state_reason_ == reason) return;
    if (state_ != next) {
      ROS_WARN_STREAM("V3 React " << stateName(state_) << " -> "
                      << stateName(next) << ": " << reason);
    }
    state_ = next;
    state_reason_ = reason;
  }

  double blockedSpeedCap(double bumper_gap) const {
    if (!std::isfinite(bumper_gap) || bumper_gap <= blocked_stop_gap_m_) {
      return blocked_minimum_speed_mps_;
    }
    return clamp(blocked_gap_gain_ * (bumper_gap - blocked_stop_gap_m_),
                 blocked_minimum_speed_mps_, blocked_maximum_speed_mps_);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!latest_odom_.has_value()) {
      transition(State::kAbort, "waiting_for_odometry");
      publishOutputs(false, 0.0, {}, {}, {}, nullptr, 0.0);
      return;
    }
    const OdomState odom = *latest_odom_;
    const auto projection = track_->project(odom.x, odom.y);
    const auto corridor = corridorAt(odom, false);
    const auto occupancy = currentOccupancy(projection.s);
    const auto decision = planReactivePass(
        corridor, projection.ey, occupancy, config_);
    if (!inputsHealthy()) {
      transition(State::kAbort, "roi_or_odom_stream_stale");
      selected_side_ = "none";
      clearance_active_ = false;
      publishOutputs(false, 0.0, decision, corridor, occupancy, nullptr,
                     projection.ey);
      return;
    }
    if (state_ == State::kAbort) transition(State::kGlobal, "inputs_recovered");

    CandidatePath output;
    const CandidatePath* output_path = nullptr;
    double speed_cap = cruise_speed_cap_mps_;
    if (decision.centerline_blocked) {
      clearance_active_ = false;
      const CandidatePath* selected = nullptr;
      if (state_ == State::kPass) {
        // Once lateral motion has started, never switch sides because a
        // single ROI frame scores the opposite corridor slightly better.
        if (selected_side_ == "left" && decision.left.feasible) {
          selected = &decision.left;
        } else if (selected_side_ == "right" && decision.right.feasible) {
          selected = &decision.right;
        }
      } else {
        if (decision.selected_side == "left") selected = &decision.left;
        if (decision.selected_side == "right") selected = &decision.right;
      }
      if (selected != nullptr) {
        selected_side_ = selected->side;
        output = *selected;
        accepted_path_ = output;
        accepted_path_wall_ = ros::WallTime::now();
        output_path = &output;
        speed_cap = pass_speed_cap_mps_;
        transition(State::kPass, "current_roi_reactive_pass");
      } else if (state_ == State::kPass && accepted_path_.feasible &&
                 wallAge(accepted_path_wall_) <= committed_path_hold_s_) {
        output = accepted_path_;
        output_path = &output;
        speed_cap = pass_speed_cap_mps_;
        transition(State::kPass, "committed_reactive_path_hold");
      } else if (state_ == State::kPass &&
                 currentLaneClear(occupancy, projection.ey)) {
        output = buildReturnPath(corridor, projection.ey,
                                 holdDistanceForOccupancy(occupancy), config_);
        if (output.feasible) {
          output_path = &output;
          speed_cap = pass_speed_cap_mps_;
          transition(State::kPass, "hold_current_free_corridor");
        } else {
          transition(State::kAbort, "committed_corridor_invalid");
          speed_cap = 0.0;
        }
      } else {
        selected_side_ = "none";
        transition(State::kFollow, "no_reactive_corridor");
        speed_cap = blockedSpeedCap(decision.bumper_gap);
      }
    } else if (state_ == State::kPass || clearance_active_) {
      if (!clearance_active_) {
        clearance_active_ = true;
        clearance_start_s_ = projection.s;
      }
      const double travelled = std::max(
          0.0, signedTrackDelta(projection.s, clearance_start_s_));
      const double remaining = std::max(
          0.0, post_disappearance_clearance_m_ - travelled);
      output = buildReturnPath(corridor, projection.ey, remaining, config_);
      if (!output.feasible) {
        // Once the current ROI is clear, an invalid local return must not
        // recreate the old stop-on-disappearance failure.  Drop the local
        // reference and hand recovery back to the frozen global V3 MPCC.
        clearance_active_ = false;
        selected_side_ = "none";
        transition(State::kGlobal,
                   "global_recovery_for_invalid_post_pass_return");
        speed_cap = cruise_speed_cap_mps_;
      } else if (remaining <= 1e-3 &&
                 std::abs(projection.ey) <= return_finish_ey_m_) {
        clearance_active_ = false;
        selected_side_ = "none";
        transition(State::kGlobal, "raceline_rejoined");
      } else {
        output_path = &output;
        speed_cap = remaining > 1e-3 ? pass_speed_cap_mps_
                                     : return_speed_cap_mps_;
        transition(remaining > 1e-3 ? State::kPass : State::kReturn,
                   remaining > 1e-3 ? "post_disappearance_clearance"
                                     : "reactive_return");
      }
    } else if (std::abs(projection.ey) > return_start_ey_m_) {
      output = buildReturnPath(corridor, projection.ey, 0.0, config_);
      if (output.feasible) {
        output_path = &output;
        speed_cap = return_speed_cap_mps_;
        transition(State::kReturn, "off_raceline_without_occupancy");
      } else {
        // A failed optional return must not reproduce the old stop-on-loss
        // behaviour.  With no current ROI blockage the frozen global V3
        // controller remains the safer recovery authority.
        transition(State::kGlobal, "global_recovery_for_invalid_return");
        speed_cap = cruise_speed_cap_mps_;
      }
    } else {
      selected_side_ = "none";
      transition(State::kGlobal, "current_roi_clear");
    }
    publishOutputs(state_ != State::kAbort, speed_cap, decision, corridor,
                   occupancy, output_path, projection.ey);
  }

  nav_msgs::Path makePath(const CandidatePath& candidate) const {
    nav_msgs::Path message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = world_frame_;
    message.poses.reserve(candidate.x.size());
    for (std::size_t i = 0; i < candidate.x.size(); ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = candidate.x[i];
      pose.pose.position.y = candidate.y[i];
      pose.pose.orientation.z = std::sin(0.5 * candidate.yaw[i]);
      pose.pose.orientation.w = std::cos(0.5 * candidate.yaw[i]);
      message.poses.push_back(pose);
    }
    return message;
  }

  void publishValidArea(const std::vector<CorridorSample>& corridor) {
    geometry_msgs::PolygonStamped polygon;
    polygon.header.stamp = ros::Time::now();
    polygon.header.frame_id = world_frame_;
    for (const auto& sample : corridor) {
      geometry_msgs::Point32 point;
      const double offset = sample.width_left - 0.5 * config_.ego_width -
                            config_.boundary_margin;
      point.x = sample.x - std::sin(sample.yaw) * offset;
      point.y = sample.y + std::cos(sample.yaw) * offset;
      polygon.polygon.points.push_back(point);
    }
    for (auto it = corridor.rbegin(); it != corridor.rend(); ++it) {
      geometry_msgs::Point32 point;
      const double offset = it->width_right - 0.5 * config_.ego_width -
                            config_.boundary_margin;
      point.x = it->x + std::sin(it->yaw) * offset;
      point.y = it->y - std::cos(it->yaw) * offset;
      polygon.polygon.points.push_back(point);
    }
    valid_area_publisher_.publish(polygon);
  }

  void publishOccupancy(const std::vector<OccupancyComponent>& occupancy,
                        double current_s) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);
    int id = 0;
    for (const auto& obstacle : occupancy) {
      const auto geometry = track_->sample(current_s + obstacle.q);
      visualization_msgs::Marker marker;
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = world_frame_;
      marker.ns = "roi_occupancy_all";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = geometry.x - std::sin(geometry.yaw) * obstacle.ey;
      marker.pose.position.y = geometry.y + std::cos(geometry.yaw) * obstacle.ey;
      marker.pose.position.z = 0.5 * (obstacle.z_min + obstacle.z_max);
      marker.pose.orientation.z = std::sin(0.5 * geometry.yaw);
      marker.pose.orientation.w = std::cos(0.5 * geometry.yaw);
      marker.scale.x = std::max(config_.voxel_size,
                                obstacle.q_max - obstacle.q_min);
      marker.scale.y = std::max(config_.voxel_size,
                                obstacle.ey_max - obstacle.ey_min);
      marker.scale.z = std::max(0.05, obstacle.z_max - obstacle.z_min);
      marker.color.r = 1.0;
      marker.color.g = 0.20;
      marker.color.b = 0.05;
      marker.color.a = 0.85;
      marker.lifetime = ros::Duration(0.25);
      array.markers.push_back(marker);
    }
    occupancy_marker_publisher_.publish(array);
  }

  void publishOutputs(bool healthy, double speed_cap,
                      const ReactDecision& decision,
                      const std::vector<CorridorSample>& corridor,
                      const std::vector<OccupancyComponent>& occupancy,
                      const CandidatePath* output,
                      double ego_ey) {
    std_msgs::Bool health_message;
    health_message.data = healthy;
    health_publisher_.publish(health_message);
    std_msgs::Float32 speed_message;
    speed_message.data = static_cast<float>(std::max(0.0, speed_cap));
    speed_cap_publisher_.publish(speed_message);
    std_msgs::String state_message;
    state_message.data = stateName(state_);
    state_publisher_.publish(state_message);

    if (!corridor.empty()) {
      left_path_publisher_.publish(makePath(decision.left));
      right_path_publisher_.publish(makePath(decision.right));
      publishValidArea(corridor);
      const auto projection = track_->project(latest_odom_->x, latest_odom_->y);
      publishOccupancy(occupancy, projection.s);
    }
    nav_msgs::Path local;
    local.header.stamp = ros::Time::now();
    local.header.frame_id = world_frame_;
    if (output != nullptr && output->feasible) {
      local = makePath(*output);
      selected_path_publisher_.publish(local);
      red_path_publisher_.publish(local);
    } else {
      selected_path_publisher_.publish(local);
      red_path_publisher_.publish(local);
    }
    local_reference_publisher_.publish(local);

    std::ostringstream status;
    status << "{\"state\":\"" << stateName(state_)
           << "\",\"healthy\":" << (healthy ? "true" : "false")
           << ",\"reason\":\"" << state_reason_
           << "\",\"input_error\":\"" << last_input_error_
           << "\",\"motion_classification\":\"disabled\""
           << ",\"target_id_used\":false"
           << ",\"roi_cloud_age_s\":" << jsonNumber(wallAge(latest_cloud_wall_))
           << ",\"ego_ey_m\":" << jsonNumber(ego_ey)
           << ",\"occupancy_count\":" << occupancy.size()
           << ",\"selected_side\":\"" << selected_side_ << "\"}";
    std_msgs::String status_message;
    status_message.data = status.str();
    status_publisher_.publish(status_message);

    std::ostringstream diagnostics;
    diagnostics << "{\"reason\":\"" << decision.reason
                << "\",\"centerline_blocked\":"
                << (decision.centerline_blocked ? "true" : "false")
                << ",\"bumper_gap_m\":" << jsonNumber(decision.bumper_gap)
                << ",\"left\":\"" << decision.left.reason
                << "\",\"right\":\"" << decision.right.reason
                << "\",\"selected\":\"" << decision.selected_side
                << "\",\"accepted_cloud_sequence\":"
                << accepted_cloud_sequence_ << "}";
    std_msgs::String diagnostics_message;
    diagnostics_message.data = diagnostics.str();
    diagnostics_publisher_.publish(diagnostics_message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher local_reference_publisher_;
  ros::Publisher red_path_publisher_;
  ros::Publisher selected_path_publisher_;
  ros::Publisher left_path_publisher_;
  ros::Publisher right_path_publisher_;
  ros::Publisher valid_area_publisher_;
  ros::Publisher occupancy_marker_publisher_;
  ros::Publisher speed_cap_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher health_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::Timer timer_;

  std::unique_ptr<PeriodicTrack> track_;
  ReactConfig config_;
  std::deque<OdomState> odom_history_;
  std::optional<OdomState> latest_odom_;
  std::vector<StoredOccupancy> stored_occupancy_;
  ros::Time latest_cloud_stamp_;
  ros::WallTime latest_odom_wall_;
  ros::WallTime latest_cloud_wall_;
  ros::WallTime last_cloud_received_wall_;
  std::uint64_t accepted_cloud_sequence_{};
  CandidatePath accepted_path_;
  ros::WallTime accepted_path_wall_;

  State state_{State::kAbort};
  std::string state_reason_{"startup"};
  std::string selected_side_{"none"};
  std::string last_input_error_{"waiting_for_inputs"};
  bool clearance_active_{false};
  double clearance_start_s_{};

  std::string raceline_csv_;
  std::string odom_topic_;
  std::string cloud_topic_;
  std::string local_reference_topic_;
  std::string red_path_topic_;
  std::string selected_path_topic_;
  std::string left_path_topic_;
  std::string right_path_topic_;
  std::string valid_area_topic_;
  std::string occupancy_marker_topic_;
  std::string speed_cap_topic_;
  std::string state_topic_;
  std::string health_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string world_frame_;
  std::vector<std::string> allowed_roi_frames_;

  double odom_timeout_s_{};
  double cloud_timeout_s_{};
  double cloud_measurement_max_age_s_{};
  double cloud_future_tolerance_s_{};
  double odom_history_duration_s_{};
  double maximum_odom_bracket_gap_s_{};
  double maximum_odom_extrapolation_s_{};
  double sample_ds_{};
  double post_disappearance_clearance_m_{};
  double committed_path_hold_s_{};
  double return_start_ey_m_{};
  double return_finish_ey_m_{};
  double update_rate_hz_{};
  double cruise_speed_cap_mps_{};
  double pass_speed_cap_mps_{};
  double return_speed_cap_mps_{};
  double blocked_minimum_speed_mps_{};
  double blocked_maximum_speed_mps_{};
  double blocked_stop_gap_m_{};
  double blocked_gap_gain_{};
};

}  // namespace
}  // namespace roboracer_react_overtake

int main(int argc, char** argv) {
  ros::init(argc, argv, "roboracer_react_overtake");
  try {
    roboracer_react_overtake::ReactOvertakeNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("V3 React planner startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
