#include "f1tenth_dynamic_mpcc/core.hpp"
#include "f1tenth_dynamic_mpcc/v5_jubu_roi.hpp"

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
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
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

namespace f1tenth_dynamic_mpcc::v5_jubu {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

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
  if (stamp.toSec() <= 0.0) return kInfinity;
  return std::max(0.0, (ros::WallTime::now() - stamp).toSec());
}

double rosAge(const ros::Time& stamp) {
  const ros::Time now = ros::Time::now();
  if (stamp.isZero() || now.isZero()) return kInfinity;
  return (now - stamp).toSec();
}

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) return "null";
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

enum class State { kFree, kFollow, kPrepare, kPass, kReturn, kAbort };

const char* stateName(State state) {
  switch (state) {
    case State::kFree: return "FREE";
    case State::kFollow: return "FOLLOW";
    case State::kPrepare: return "PREPARE";
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
  double vy{};
};

struct AbsoluteObstacle {
  double s{};
  double ey{};
  double length{};
  double width{};
  double z_min{};
  double z_max{};
  std::size_t point_count{};
};

struct ObstacleMemory {
  bool valid{false};
  double s{};
  double ey{};
  double speed{};
  double length{};
  double width{};
  double z_min{};
  double z_max{};
  std::size_t point_count{};
  ros::Time measurement_stamp;
  ros::WallTime seen_wall;
};

class V5JubuLocalPlanner {
 public:
  V5JubuLocalPlanner() : private_node_("~") {
    loadParameters();
    const YAML::Node controller = YAML::LoadFile(controller_config_);
    const YAML::Node vehicle = YAML::LoadFile(vehicle_config_);
    const auto parameters = VehicleParameters::fromYaml(vehicle, controller);
    track_ = std::make_unique<PeriodicTrack>(
        track_csv_, controller["speed_planning"], parameters);

    odom_subscriber_ = node_.subscribe(
        odom_topic_, 100, &V5JubuLocalPlanner::odomCallback, this);
    cloud_subscriber_ = node_.subscribe(
        cloud_topic_, 2, &V5JubuLocalPlanner::cloudCallback, this,
        ros::TransportHints().tcpNoDelay());
    local_reference_publisher_ =
        node_.advertise<nav_msgs::Path>(local_reference_topic_, 3);
    left_path_publisher_ = node_.advertise<nav_msgs::Path>(left_path_topic_, 2);
    right_path_publisher_ = node_.advertise<nav_msgs::Path>(right_path_topic_, 2);
    selected_path_publisher_ =
        node_.advertise<nav_msgs::Path>(selected_path_topic_, 2);
    valid_area_publisher_ =
        node_.advertise<geometry_msgs::PolygonStamped>(valid_area_topic_, 2);
    health_publisher_ = node_.advertise<std_msgs::Bool>(health_topic_, 5);
    speed_cap_publisher_ =
        node_.advertise<std_msgs::Float32>(speed_cap_topic_, 5);
    state_publisher_ = node_.advertise<std_msgs::String>(state_topic_, 5);
    status_publisher_ = node_.advertise<std_msgs::String>(status_topic_, 5);
    diagnostics_publisher_ =
        node_.advertise<std_msgs::String>(diagnostics_topic_, 5);
    reset_service_ = private_node_.advertiseService(
        "reset_abort", &V5JubuLocalPlanner::resetAbort, this);
    timer_ = node_.createTimer(
        ros::Duration(1.0 / update_rate_hz_),
        &V5JubuLocalPlanner::timerCallback, this);
    state_started_wall_ = ros::WallTime::now();
    ROS_INFO_STREAM("V5 Jubu C++ planner ready: cloud=" << cloud_topic_
                    << " odom=" << odom_topic_
                    << " track_length=" << track_->length());
  }

 private:
  template <typename T>
  void parameter(const std::string& name, T& target, const T& fallback) {
    private_node_.param(name, target, fallback);
  }

  void loadParameters() {
    if (!private_node_.getParam("controller_config", controller_config_) ||
        !private_node_.getParam("vehicle_config", vehicle_config_) ||
        !private_node_.getParam("track_csv", track_csv_)) {
      throw std::runtime_error(
          "controller_config, vehicle_config and track_csv are required");
    }
    parameter("topics/odom", odom_topic_,
              std::string("/localization/odom"));
    parameter("topics/roi_cloud", cloud_topic_,
              std::string("/perception/roi_cloud"));
    parameter("topics/local_reference", local_reference_topic_,
              std::string("/v5_jubu/local_reference"));
    parameter("topics/candidate_left", left_path_topic_,
              std::string("/v5_jubu/candidate_left"));
    parameter("topics/candidate_right", right_path_topic_,
              std::string("/v5_jubu/candidate_right"));
    parameter("topics/selected_path", selected_path_topic_,
              std::string("/v5_jubu/selected_path"));
    parameter("topics/valid_area", valid_area_topic_,
              std::string("/v5_jubu/valid_area"));
    parameter("topics/health", health_topic_,
              std::string("/v5_jubu/health"));
    parameter("topics/speed_cap", speed_cap_topic_,
              std::string("/v5_jubu/local_speed_cap"));
    parameter("topics/state", state_topic_,
              std::string("/v5_jubu/state"));
    parameter("topics/status", status_topic_,
              std::string("/v5_jubu/status"));
    parameter("topics/diagnostics", diagnostics_topic_,
              std::string("/v5_jubu/candidate_diagnostics"));
    parameter("frames/world", world_frame_, std::string("map"));
    parameter("frames/roi_primary", roi_primary_frame_,
              std::string("body"));
    parameter("frames/roi_secondary", roi_secondary_frame_,
              std::string("body_leveled"));

    parameter("input/odom_timeout_s", odom_timeout_s_, 0.30);
    parameter("input/cloud_timeout_s", cloud_timeout_s_, 0.50);
    parameter("input/cloud_measurement_max_age_s",
              cloud_measurement_max_age_s_, 0.80);
    parameter("input/cloud_future_tolerance_s", cloud_future_tolerance_s_,
              0.05);
    parameter("input/odom_history_duration_s", odom_history_duration_s_, 2.0);
    parameter("input/maximum_odom_bracket_gap_s", maximum_odom_bracket_gap_s_,
              0.30);
    parameter("input/maximum_odom_extrapolation_s",
              maximum_odom_extrapolation_s_, 0.10);
    parameter("input/obstacle_memory_timeout_s", obstacle_memory_timeout_s_,
              2.0);
    parameter("input/unknown_obstacle_speed_mps",
              unknown_obstacle_speed_mps_, 1.0);
    parameter("input/obstacle_speed_min_mps", obstacle_speed_min_mps_, -0.5);
    parameter("input/obstacle_speed_max_mps", obstacle_speed_max_mps_, 3.0);
    parameter("input/obstacle_speed_filter_alpha",
              obstacle_speed_filter_alpha_, 0.35);
    parameter("input/obstacle_association_distance_m",
              obstacle_association_distance_m_, 1.5);
    parameter("input/single_target_mode", single_target_mode_, false);

    parameter("roi/minimum_q", extraction_config_.minimum_q, 0.30);
    parameter("roi/maximum_q", extraction_config_.maximum_q, 6.0);
    parameter("roi/boundary_strip", extraction_config_.boundary_strip, 0.10);
    parameter("roi/voxel_size", extraction_config_.voxel_size, 0.10);
    parameter("roi/minimum_component_points",
              extraction_config_.minimum_component_points, 8);
    parameter("roi/maximum_component_length",
              extraction_config_.maximum_component_length, 0.80);
    parameter("roi/maximum_component_width",
              extraction_config_.maximum_component_width, 0.70);
    parameter("roi/minimum_obstacle_top_z",
              extraction_config_.minimum_obstacle_top_z, 0.02);
    parameter("roi/minimum_centroid_boundary_clearance",
              extraction_config_.minimum_centroid_boundary_clearance, 0.0);
    parameter("roi/fragment_merge_longitudinal_gap",
              extraction_config_.fragment_merge_longitudinal_gap, 0.0);
    parameter("roi/fragment_merge_lateral_gap",
              extraction_config_.fragment_merge_lateral_gap, 0.0);

    parameter("planner/ego_width", candidate_config_.ego_width, 0.24);
    parameter("planner/ego_length", candidate_config_.ego_length, 0.40);
    parameter("planner/opponent_width",
              candidate_config_.default_obstacle_width, 0.35);
    parameter("planner/opponent_length",
              candidate_config_.default_obstacle_length, 0.40);
    parameter("planner/boundary_margin", candidate_config_.boundary_margin,
              0.08);
    parameter("planner/minimum_boundary_clearance_for_pass",
              candidate_config_.minimum_boundary_clearance, 0.03);
    parameter("planner/lateral_clearance",
              candidate_config_.lateral_clearance, 0.06);
    parameter("planner/longitudinal_clearance",
              candidate_config_.longitudinal_clearance, 0.20);
    parameter("planner/detection_distance",
              candidate_config_.detection_distance, 6.0);
    parameter("planner/trigger_distance",
              candidate_config_.trigger_distance, 4.0);
    parameter("planner/trigger_ttc", candidate_config_.trigger_ttc, 4.0);
    parameter("planner/minimum_closing_speed",
              candidate_config_.minimum_closing_speed, 0.10);
    parameter("planner/minimum_lane_change_length",
              candidate_config_.minimum_lane_change_length, 0.80);
    parameter("planner/nominal_lane_change_length",
              candidate_config_.nominal_lane_change_length, 1.60);
    parameter("planner/pass_hold_after_opponent",
              candidate_config_.pass_hold_after_obstacle, 0.70);
    parameter("planner/return_length", candidate_config_.return_length, 1.60);
    parameter("planner/minimum_plan_speed",
              candidate_config_.minimum_plan_speed, 0.80);
    parameter("planner/maximum_plan_speed",
              candidate_config_.maximum_plan_speed, 2.20);
    parameter("planner/pass_speed_advantage",
              candidate_config_.pass_speed_advantage, 0.60);
    parameter("planner/maximum_path_curvature",
              candidate_config_.maximum_path_curvature, 1.70);
    parameter("planner/maximum_track_curvature_for_pass",
              candidate_config_.maximum_track_curvature_for_pass, 1.20);
    parameter("planner/maximum_candidate_length",
              candidate_config_.maximum_candidate_length, 12.0);
    parameter("planner/preferred_side", candidate_config_.preferred_side,
              std::string("left"));
    parameter("planner/sample_ds", sample_ds_, 0.10);

    parameter("state_machine/prepare_time_s", prepare_time_s_, 0.40);
    parameter("state_machine/follow_target_loss_timeout_s",
              follow_target_loss_timeout_s_, 1.00);
    parameter("state_machine/return_confirm_gap_m", return_confirm_gap_m_,
              0.45);
    parameter("state_machine/return_ey_tolerance_m", return_ey_tolerance_m_,
              0.10);
    parameter("state_machine/abort_recovery_distance_m",
              abort_recovery_distance_m_, 0.55);
    parameter("state_machine/graceful_pass_target_loss_enabled",
              graceful_pass_target_loss_enabled_, false);
    parameter("state_machine/pass_target_loss_hold_s",
              pass_target_loss_hold_s_, 0.35);
    parameter("state_machine/pass_target_loss_return_retry_timeout_s",
              pass_target_loss_return_retry_timeout_s_, 1.50);
    parameter("state_machine/pass_target_loss_speed_cap_mps",
              pass_target_loss_speed_cap_mps_, 1.50);
    parameter("confirmation/minimum_frames", confirmation_minimum_frames_, 2);
    parameter("confirmation/minimum_duration_s",
              confirmation_minimum_duration_s_, 0.15);
    parameter("confirmation/maximum_gap_s", confirmation_maximum_gap_s_,
              0.80);

    parameter("control/update_rate_hz", update_rate_hz_, 20.0);
    parameter("control/cruise_speed_cap_mps", cruise_speed_cap_mps_, 3.0);
    parameter("control/pass_speed_cap_mps", pass_speed_cap_mps_, 2.20);
    parameter("control/follow_minimum_speed_mps", follow_minimum_speed_mps_,
              0.40);
    parameter("control/follow_gap_m", follow_gap_m_, 0.75);
    parameter("control/follow_gap_gain", follow_gap_gain_, 0.55);

    extraction_config_.validate();
    candidate_config_.validate();
    if (update_rate_hz_ <= 0.0 || sample_ds_ <= 0.0 ||
        odom_timeout_s_ <= 0.0 || cloud_timeout_s_ <= 0.0 ||
        cloud_measurement_max_age_s_ <= 0.0 ||
        maximum_odom_bracket_gap_s_ <= 0.0 ||
        maximum_odom_extrapolation_s_ < 0.0 ||
        confirmation_minimum_frames_ <= 0 ||
        (graceful_pass_target_loss_enabled_ &&
         (pass_target_loss_hold_s_ < 0.0 ||
          pass_target_loss_return_retry_timeout_s_ <
              pass_target_loss_hold_s_ ||
          pass_target_loss_speed_cap_mps_ <= 0.0 ||
          pass_target_loss_speed_cap_mps_ > pass_speed_cap_mps_))) {
      throw std::runtime_error("invalid V5 Jubu runtime configuration");
    }
    roi_primary_frame_ = normalizedFrame(roi_primary_frame_);
    roi_secondary_frame_ = normalizedFrame(roi_secondary_frame_);
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
    state.vy = message->twist.twist.linear.y;
    if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
        !std::isfinite(state.yaw) || !std::isfinite(state.vx) ||
        !std::isfinite(state.vy)) {
      last_input_error_ = "non_finite_odom";
      return;
    }
    if (!odom_history_.empty() &&
        state.stamp <= odom_history_.back().stamp) {
      const double reorder =
          (odom_history_.back().stamp - state.stamp).toSec();
      if (reorder > 0.001) last_input_error_ = "out_of_order_odom";
      // Even sub-millisecond reordering or duplicate stamps must not enter the
      // interpolation deque: a non-positive bracket makes odomAt() reject an
      // otherwise valid cloud measurement.
      return;
    }
    odom_history_.push_back(state);
    const ros::Time cutoff =
        state.stamp - ros::Duration(odom_history_duration_s_);
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
          maximum_odom_extrapolation_s_) {
        return odom_history_.front();
      }
      return std::nullopt;
    }
    if (stamp >= odom_history_.back().stamp) {
      const double dt = (stamp - odom_history_.back().stamp).toSec();
      if (dt > maximum_odom_extrapolation_s_) return std::nullopt;
      OdomState result = odom_history_.back();
      result.stamp = stamp;
      result.x += dt * (std::cos(result.yaw) * result.vx -
                        std::sin(result.yaw) * result.vy);
      result.y += dt * (std::sin(result.yaw) * result.vx +
                        std::cos(result.yaw) * result.vy);
      return result;
    }
    for (std::size_t i = 1; i < odom_history_.size(); ++i) {
      const auto& upper = odom_history_[i];
      if (upper.stamp < stamp) continue;
      const auto& lower = odom_history_[i - 1];
      const double gap = (upper.stamp - lower.stamp).toSec();
      if (gap <= 0.0 || gap > maximum_odom_bracket_gap_s_) {
        return std::nullopt;
      }
      const double ratio = (stamp - lower.stamp).toSec() / gap;
      OdomState result;
      result.stamp = stamp;
      result.x = lower.x + ratio * (upper.x - lower.x);
      result.y = lower.y + ratio * (upper.y - lower.y);
      result.yaw = wrapAngle(
          lower.yaw + ratio * wrapAngle(upper.yaw - lower.yaw));
      result.vx = lower.vx + ratio * (upper.vx - lower.vx);
      result.vy = lower.vy + ratio * (upper.vy - lower.vy);
      return result;
    }
    return std::nullopt;
  }

  double signedTrackDelta(double target_s, double origin_s) const {
    double delta = track_->wrapS(target_s - origin_s);
    if (delta > 0.5 * track_->length()) delta -= track_->length();
    return delta;
  }

  std::vector<CorridorSample> corridorAt(const OdomState& odom,
                                         bool local_frame,
                                         double maximum_q) const {
    const auto projection = track_->project(odom.x, odom.y);
    std::vector<CorridorSample> corridor;
    const int count =
        static_cast<int>(std::ceil(maximum_q / sample_ds_));
    corridor.reserve(static_cast<std::size_t>(count + 1));
    for (int i = 0; i <= count; ++i) {
      const double q = std::min(maximum_q, sample_ds_ * i);
      const double s = track_->wrapS(projection.s_wrapped + q);
      const auto geometry = track_->geometry(s);
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
      corridor.push_back(
          {q, s, x, y, yaw, geometry.width_left, geometry.width_right});
    }
    return corridor;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    last_cloud_received_wall_ = ros::WallTime::now();
    const std::string frame = normalizedFrame(message->header.frame_id);
    if (frame != roi_primary_frame_ && frame != roi_secondary_frame_) {
      last_input_error_ = "unexpected_roi_frame:" + frame;
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
      const auto local_corridor = corridorAt(
          *measurement_odom, true, extraction_config_.maximum_q);
      const auto extracted =
          extractRoiObstacles(points, local_corridor, extraction_config_);
      const auto projection =
          track_->project(measurement_odom->x, measurement_odom->y);
      std::vector<AbsoluteObstacle> absolute;
      absolute.reserve(extracted.size());
      for (const auto& obstacle : extracted) {
        absolute.push_back(
            {track_->wrapS(projection.s_wrapped + obstacle.q), obstacle.ey,
             obstacle.length, obstacle.width, obstacle.z_min, obstacle.z_max,
             obstacle.point_count});
      }
      updateObstacleMemory(absolute, message->header.stamp);
      latest_absolute_obstacles_ = std::move(absolute);
      latest_cloud_stamp_ = message->header.stamp;
      latest_cloud_wall_ = ros::WallTime::now();
      ++accepted_cloud_sequence_;
      last_input_error_ = "none";
    } catch (const std::exception& error) {
      last_input_error_ = std::string("roi_decode_or_extract_error:") +
                          error.what();
    }
  }

  void updateObstacleMemory(const std::vector<AbsoluteObstacle>& obstacles,
                            const ros::Time& measurement_stamp) {
    if (obstacles.empty()) return;
    const auto* primary = &obstacles.front();
    if (obstacle_memory_.valid) {
      for (const auto& obstacle : obstacles) {
        if (std::abs(signedTrackDelta(obstacle.s, obstacle_memory_.s)) <
            std::abs(signedTrackDelta(primary->s, obstacle_memory_.s))) {
          primary = &obstacle;
        }
      }
    }
    double speed = unknown_obstacle_speed_mps_;
    if (obstacle_memory_.valid) {
      const double dt =
          (measurement_stamp - obstacle_memory_.measurement_stamp).toSec();
      const double ds = signedTrackDelta(primary->s, obstacle_memory_.s);
      if (dt >= 0.05 && dt <= 1.0 &&
          std::abs(ds) <= obstacle_association_distance_m_) {
        const double measured_speed =
            clamp(ds / dt, obstacle_speed_min_mps_, obstacle_speed_max_mps_);
        speed = (1.0 - obstacle_speed_filter_alpha_) * obstacle_memory_.speed +
                obstacle_speed_filter_alpha_ * measured_speed;
      }
    }
    obstacle_memory_ = {true,
                        primary->s,
                        primary->ey,
                        speed,
                        primary->length,
                        primary->width,
                        primary->z_min,
                        primary->z_max,
                        primary->point_count,
                        measurement_stamp,
                        ros::WallTime::now()};
    last_nonempty_cloud_wall_ = obstacle_memory_.seen_wall;
  }

  bool baseInputsHealthy() const {
    if (!latest_odom_.has_value() || latest_cloud_stamp_.isZero()) return false;
    if (wallAge(latest_odom_wall_) > odom_timeout_s_ ||
        wallAge(latest_cloud_wall_) > cloud_timeout_s_) {
      return false;
    }
    const double age = rosAge(latest_cloud_stamp_);
    return std::isfinite(age) && age <= cloud_measurement_max_age_s_ &&
           age >= -cloud_future_tolerance_s_;
  }

  std::vector<RoiObstacle> currentObstacles(
      double current_s, bool allow_memory) const {
    std::vector<RoiObstacle> result;
    const double elapsed = std::max(0.0, rosAge(latest_cloud_stamp_));
    if (!latest_absolute_obstacles_.empty() &&
        wallAge(latest_cloud_wall_) <= cloud_timeout_s_) {
      result.reserve(latest_absolute_obstacles_.size());
      for (const auto& obstacle : latest_absolute_obstacles_) {
        const double propagated_s = track_->wrapS(
            obstacle.s + obstacle_memory_.speed * elapsed);
        result.push_back(
            {signedTrackDelta(propagated_s, current_s), obstacle.ey, 0.0, 0.0,
             obstacle.length, obstacle.width, obstacle.z_min, obstacle.z_max,
             obstacle.point_count});
      }
    } else if (allow_memory && obstacle_memory_.valid &&
               wallAge(obstacle_memory_.seen_wall) <=
                   obstacle_memory_timeout_s_) {
      const double memory_elapsed =
          std::max(0.0, rosAge(obstacle_memory_.measurement_stamp));
      const double propagated_s = track_->wrapS(
          obstacle_memory_.s + obstacle_memory_.speed * memory_elapsed);
      result.push_back(
          {signedTrackDelta(propagated_s, current_s), obstacle_memory_.ey,
           0.0, 0.0, obstacle_memory_.length, obstacle_memory_.width,
           obstacle_memory_.z_min, obstacle_memory_.z_max,
           obstacle_memory_.point_count});
    }
    std::sort(result.begin(), result.end(),
              [](const RoiObstacle& left, const RoiObstacle& right) {
                return left.q < right.q;
              });
    if (single_target_mode_ && result.size() > 1U) {
      auto primary = result.begin();
      if (obstacle_memory_.valid) {
        const double memory_elapsed =
            std::max(0.0, rosAge(obstacle_memory_.measurement_stamp));
        const double memory_s = track_->wrapS(
            obstacle_memory_.s + obstacle_memory_.speed * memory_elapsed);
        const double memory_q = signedTrackDelta(memory_s, current_s);
        primary = std::min_element(
            result.begin(), result.end(),
            [memory_q, this](const RoiObstacle& left,
                             const RoiObstacle& right) {
              const double left_score =
                  std::abs(left.q - memory_q) +
                  0.35 * std::abs(left.ey - obstacle_memory_.ey);
              const double right_score =
                  std::abs(right.q - memory_q) +
                  0.35 * std::abs(right.ey - obstacle_memory_.ey);
              return left_score < right_score;
            });
      }
      const RoiObstacle selected = *primary;
      result.assign(1U, selected);
    }
    return result;
  }

  void transition(State next, const std::string& reason) {
    if (state_ == next) return;
    ROS_WARN_STREAM("V5 Jubu " << stateName(state_) << " -> "
                    << stateName(next) << ": " << reason);
    state_ = next;
    state_reason_ = reason;
    state_started_wall_ = ros::WallTime::now();
    if (next == State::kFree || next == State::kAbort) {
      selected_side_ = "none";
      accepted_path_ = CandidatePath{};
      target_loss_recovery_active_ = false;
      target_loss_started_wall_ = ros::WallTime{};
    } else if (next == State::kPass) {
      target_loss_recovery_active_ = false;
      target_loss_started_wall_ = ros::WallTime{};
    }
    if (next != State::kFollow) resetConfirmation();
  }

  void resetConfirmation() {
    confirmation_frames_ = 0;
    confirmation_started_wall_ = ros::WallTime{};
    confirmation_last_wall_ = ros::WallTime{};
    confirmation_sequence_ = accepted_cloud_sequence_;
  }

  void noteConfirmation(bool candidate_ready) {
    if (!candidate_ready) {
      resetConfirmation();
      return;
    }
    if (confirmation_sequence_ == accepted_cloud_sequence_) return;
    confirmation_sequence_ = accepted_cloud_sequence_;
    const ros::WallTime now = ros::WallTime::now();
    if (confirmation_last_wall_.toSec() > 0.0 &&
        (now - confirmation_last_wall_).toSec() > confirmation_maximum_gap_s_) {
      confirmation_frames_ = 0;
      confirmation_started_wall_ = ros::WallTime{};
    }
    if (confirmation_frames_ == 0) confirmation_started_wall_ = now;
    ++confirmation_frames_;
    confirmation_last_wall_ = now;
  }

  bool confirmationReady() const {
    return confirmation_frames_ >= confirmation_minimum_frames_ &&
           confirmation_started_wall_.toSec() > 0.0 &&
           (ros::WallTime::now() - confirmation_started_wall_).toSec() >=
               confirmation_minimum_duration_s_;
  }

  const CandidatePath* selectedCandidate(const PlanningDecision& decision,
                                         const std::string& side) const {
    if (side == "left" && decision.left.feasible) return &decision.left;
    if (side == "right" && decision.right.feasible) return &decision.right;
    return nullptr;
  }

  double followSpeedCap(const PlanningDecision& decision,
                        double obstacle_speed) const {
    const double requested = obstacle_speed +
        follow_gap_gain_ * (decision.bumper_gap - follow_gap_m_);
    return clamp(requested, follow_minimum_speed_mps_, cruise_speed_cap_mps_);
  }

  bool pathClearAtCurrentOffset(const std::vector<RoiObstacle>& obstacles,
                                double ego_ey) const {
    for (const auto& obstacle : obstacles) {
      const double required_lateral =
          0.5 * (candidate_config_.ego_width +
                 std::max(candidate_config_.default_obstacle_width,
                          obstacle.width)) +
          candidate_config_.lateral_clearance;
      const double required_longitudinal =
          0.5 * (candidate_config_.ego_length +
                 std::max(candidate_config_.default_obstacle_length,
                          obstacle.length)) +
          candidate_config_.longitudinal_clearance;
      if (std::abs(obstacle.q) < required_longitudinal &&
          std::abs(ego_ey - obstacle.ey) < required_lateral) {
        return false;
      }
    }
    return true;
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!latest_odom_.has_value()) {
      publishOutputs(false, 0.0, PlanningDecision{}, {}, nullptr, 0.0);
      return;
    }
    const OdomState odom = *latest_odom_;
    const auto projection = track_->project(odom.x, odom.y);
    const auto corridor = corridorAt(
        odom, false, candidate_config_.maximum_candidate_length);
    const bool inputs_healthy = baseInputsHealthy();
    const bool maneuver_committed =
        state_ == State::kPrepare || state_ == State::kPass ||
        state_ == State::kReturn;
    if (!inputs_healthy && maneuver_committed) {
      transition(State::kAbort, "required_roi_or_odom_stream_stale");
    } else if (!inputs_healthy && state_ == State::kFollow) {
      // Before commitment, remain fail-closed for the stale interval but do
      // not latch ABORT. A fresh cloud can safely resume global tracking.
      transition(State::kFree, "input_stale_before_commit");
    }
    const bool committed = state_ == State::kPrepare || state_ == State::kPass ||
                           state_ == State::kReturn;
    const auto obstacles = currentObstacles(projection.s_wrapped, committed);
    const double ego_speed = std::max(0.0, odom.vx);
    const double obstacle_speed = obstacle_memory_.valid
                                      ? obstacle_memory_.speed
                                      : unknown_obstacle_speed_mps_;
    const auto decision = planAvoidance(
        corridor, projection.e_contour, ego_speed, obstacle_speed,
        obstacles, candidate_config_);
    const CandidatePath* output_path = nullptr;
    double speed_cap = cruise_speed_cap_mps_;

    if (inputs_healthy && state_ != State::kAbort) {
      switch (state_) {
        case State::kFree:
          if (decision.relevant) {
            transition(State::kFollow, "forward_roi_obstacle");
          }
          break;
        case State::kFollow: {
          speed_cap = followSpeedCap(decision, obstacle_speed);
          if (!decision.relevant &&
              wallAge(last_nonempty_cloud_wall_) >
                  follow_target_loss_timeout_s_) {
            transition(State::kFree, "forward_roi_clear");
            speed_cap = cruise_speed_cap_mps_;
            break;
          }
          const bool ready = decision.risk && decision.selected_side != "none";
          noteConfirmation(ready);
          if (confirmationReady()) {
            selected_side_ = decision.selected_side;
            transition(State::kPrepare, "candidate_confirmed");
          }
          break;
        }
        case State::kPrepare: {
          speed_cap = followSpeedCap(decision, obstacle_speed);
          const auto* candidate = selectedCandidate(decision, selected_side_);
          if (!decision.relevant) {
            transition(State::kFollow, "obstacle_lost_before_commit");
          } else if (candidate == nullptr) {
            transition(State::kFollow, "candidate_invalid_before_commit");
          } else if (wallAge(state_started_wall_) >= prepare_time_s_) {
            accepted_path_ = *candidate;
            transition(State::kPass, "pass_committed");
            output_path = &accepted_path_;
            speed_cap = pass_speed_cap_mps_;
          }
          break;
        }
        case State::kPass: {
          speed_cap = pass_speed_cap_mps_;
          if (obstacles.empty()) {
            if (!graceful_pass_target_loss_enabled_) {
              transition(State::kAbort,
                         "obstacle_memory_expired_during_pass");
              speed_cap = 0.0;
              break;
            }
            if (target_loss_started_wall_.isZero()) {
              target_loss_started_wall_ = ros::WallTime::now();
              target_loss_recovery_active_ = true;
              state_reason_ = "target_lost_hold_committed_lane";
              ROS_WARN_STREAM(
                  "V5 Jubu PASS target lost after memory timeout; keeping "
                  "committed lane at " << pass_target_loss_speed_cap_mps_
                  << " m/s before safe RETURN");
            }
            const double target_loss_age =
                wallAge(target_loss_started_wall_);
            speed_cap = pass_target_loss_speed_cap_mps_;
            if (target_loss_age >= pass_target_loss_hold_s_) {
              CandidatePath return_path = buildReturnPath(
                  corridor, projection.e_contour, candidate_config_);
              if (return_path.feasible) {
                accepted_path_ = std::move(return_path);
                transition(State::kReturn,
                           "target_lost_safe_return");
                output_path = &accepted_path_;
                break;
              }
              state_reason_ =
                  "target_lost_waiting_for_safe_return:" +
                  return_path.reason;
              if (target_loss_age >
                  pass_target_loss_return_retry_timeout_s_) {
                transition(State::kAbort,
                           "target_lost_return_unavailable:" +
                               return_path.reason);
                speed_cap = 0.0;
                break;
              }
            }
          } else {
            target_loss_recovery_active_ = false;
            target_loss_started_wall_ = ros::WallTime{};
          }
          if (!obstacles.empty()) {
            const auto& primary = obstacles.front();
            if (primary.q <= -return_confirm_gap_m_) {
              transition(State::kReturn, "obstacle_cleared_rear_gap");
              accepted_path_ = buildReturnPath(
                  corridor, projection.e_contour, candidate_config_);
            } else {
            if (!pathClearAtCurrentOffset(obstacles, projection.e_contour)) {
              transition(State::kAbort, "roi_collision_on_committed_lane");
              speed_cap = 0.0;
              break;
            }
            const auto* replanned =
                selectedCandidate(decision, selected_side_);
            if (replanned != nullptr) {
              // Keep regenerating the entry from the current vehicle pose.
              // This prevents the first PASS tick from replacing the committed
              // lane change with a zero-offset hold path.
              accepted_path_ = *replanned;
            } else {
              const double required_lateral =
                  0.5 * (candidate_config_.ego_width +
                         std::max(candidate_config_.default_obstacle_width,
                                  primary.width)) +
                  candidate_config_.lateral_clearance;
              if (std::abs(projection.e_contour - primary.ey) <
                  required_lateral) {
                // The vehicle has not yet established lateral clearance, so
                // returning to obstacle following is safer and more robust
                // than latching ABORT on one transient infeasible candidate.
                // Hard collision, stale input and invalid committed paths
                // remain fail-closed above and below this branch.
                transition(State::kFollow,
                           "lane_change_cancelled_before_lateral_clearance");
                selected_side_ = "none";
                accepted_path_ = CandidatePath{};
                resetConfirmation();
                speed_cap = followSpeedCap(decision, obstacle_speed);
                break;
              }
              const double obstacle_length = std::max(
                  candidate_config_.default_obstacle_length, primary.length);
              const double hold_distance = std::max(
                  0.0, primary.q +
                           0.5 * (candidate_config_.ego_length +
                                  obstacle_length) +
                           candidate_config_.longitudinal_clearance +
                           candidate_config_.pass_hold_after_obstacle);
              accepted_path_ = buildReturnPath(
                  corridor, projection.e_contour, candidate_config_,
                  hold_distance);
            }
            }
          }
          if (!accepted_path_.feasible) {
            transition(State::kAbort,
                       "committed_path_invalid:" + accepted_path_.reason);
            speed_cap = 0.0;
          } else {
            output_path = &accepted_path_;
          }
          break;
        }
        case State::kReturn:
          speed_cap = target_loss_recovery_active_
                          ? pass_target_loss_speed_cap_mps_
                          : pass_speed_cap_mps_;
          {
          CandidatePath return_path = buildReturnPath(
              corridor, projection.e_contour, candidate_config_);
          if (!return_path.feasible && target_loss_recovery_active_ &&
              wallAge(target_loss_started_wall_) <=
                  pass_target_loss_return_retry_timeout_s_) {
            state_reason_ = "target_loss_return_retry:" + return_path.reason;
            output_path = &accepted_path_;
          } else if (!return_path.feasible) {
            transition(State::kAbort,
                       "return_path_invalid:" + return_path.reason);
            speed_cap = 0.0;
          } else {
            accepted_path_ = std::move(return_path);
            if (std::abs(projection.e_contour) <=
                     return_ey_tolerance_m_) {
              transition(State::kFree, "raceline_rejoined");
              speed_cap = cruise_speed_cap_mps_;
            } else {
              output_path = &accepted_path_;
            }
          }
          }
          break;
        case State::kAbort:
          speed_cap = 0.0;
          break;
      }
    } else if (!inputs_healthy || state_ == State::kAbort) {
      speed_cap = 0.0;
    }
    const bool health = inputs_healthy && state_ != State::kAbort;
    publishOutputs(health, speed_cap, decision, corridor, output_path,
                   projection.e_contour);
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
    polygon.polygon.points.reserve(corridor.size() * 2);
    for (const auto& sample : corridor) {
      geometry_msgs::Point32 point;
      const double limit = sample.width_left - 0.5 * candidate_config_.ego_width -
                           candidate_config_.boundary_margin;
      point.x = static_cast<float>(sample.x - std::sin(sample.yaw) * limit);
      point.y = static_cast<float>(sample.y + std::cos(sample.yaw) * limit);
      polygon.polygon.points.push_back(point);
    }
    for (auto iterator = corridor.rbegin(); iterator != corridor.rend();
         ++iterator) {
      geometry_msgs::Point32 point;
      const double limit = iterator->width_right -
                           0.5 * candidate_config_.ego_width -
                           candidate_config_.boundary_margin;
      point.x = static_cast<float>(iterator->x +
                                   std::sin(iterator->yaw) * limit);
      point.y = static_cast<float>(iterator->y -
                                   std::cos(iterator->yaw) * limit);
      polygon.polygon.points.push_back(point);
    }
    valid_area_publisher_.publish(polygon);
  }

  void publishOutputs(bool health, double speed_cap,
                      const PlanningDecision& decision,
                      const std::vector<CorridorSample>& corridor,
                      const CandidatePath* output_path,
                      double ego_ey) {
    std_msgs::Bool health_message;
    health_message.data = health;
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
    }
    if (output_path != nullptr && output_path->feasible) {
      const auto message = makePath(*output_path);
      selected_path_publisher_.publish(message);
      if (state_ == State::kPass || state_ == State::kReturn) {
        local_reference_publisher_.publish(message);
      }
    }

    std::ostringstream status;
    status << "{\"state\":\"" << stateName(state_)
           << "\",\"healthy\":" << (health ? "true" : "false")
           << ",\"reason\":\"" << state_reason_
           << "\",\"input_error\":\"" << last_input_error_
           << "\",\"roi_cloud_age_s\":" << jsonNumber(wallAge(latest_cloud_wall_))
           << ",\"roi_receive_age_s\":"
           << jsonNumber(wallAge(last_cloud_received_wall_))
           << ",\"odom_age_s\":" << jsonNumber(wallAge(latest_odom_wall_))
           << ",\"ego_ey_m\":" << jsonNumber(ego_ey)
           << ",\"obstacle_speed_mps\":"
           << jsonNumber(obstacle_memory_.speed)
           << ",\"selected_side\":\"" << selected_side_ << "\"}";
    std_msgs::String status_message;
    status_message.data = status.str();
    status_publisher_.publish(status_message);

    std::ostringstream diagnostics;
    diagnostics << "{\"reason\":\"" << decision.reason
                << "\",\"relevant\":"
                << (decision.relevant ? "true" : "false")
                << ",\"risk\":" << (decision.risk ? "true" : "false")
                << ",\"bumper_gap_m\":" << jsonNumber(decision.bumper_gap)
                << ",\"closing_speed_mps\":"
                << jsonNumber(decision.closing_speed)
                << ",\"ttc_s\":" << jsonNumber(decision.ttc)
                << ",\"left\":\"" << decision.left.reason
                << "\",\"right\":\"" << decision.right.reason
                << "\",\"selected\":\"" << decision.selected_side
                << "\",\"accepted_cloud_sequence\":"
                << accepted_cloud_sequence_ << "}";
    std_msgs::String diagnostics_message;
    diagnostics_message.data = diagnostics.str();
    diagnostics_publisher_.publish(diagnostics_message);
  }

  bool resetAbort(std_srvs::Trigger::Request&,
                  std_srvs::Trigger::Response& response) {
    if (state_ != State::kAbort) {
      response.success = true;
      response.message = "planner is not in ABORT";
      return true;
    }
    if (!baseInputsHealthy() || !latest_odom_.has_value()) {
      response.success = false;
      response.message = "ROI cloud or odometry is not healthy";
      return true;
    }
    const auto projection =
        track_->project(latest_odom_->x, latest_odom_->y);
    if (std::abs(projection.e_contour) > abort_recovery_distance_m_) {
      response.success = false;
      response.message = "vehicle is too far from the raceline";
      return true;
    }
    transition(State::kFree, "operator_reset_abort");
    response.success = true;
    response.message = "ABORT reset; planner remains gated by live inputs";
    return true;
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber cloud_subscriber_;
  ros::Publisher local_reference_publisher_;
  ros::Publisher left_path_publisher_;
  ros::Publisher right_path_publisher_;
  ros::Publisher selected_path_publisher_;
  ros::Publisher valid_area_publisher_;
  ros::Publisher health_publisher_;
  ros::Publisher speed_cap_publisher_;
  ros::Publisher state_publisher_;
  ros::Publisher status_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::ServiceServer reset_service_;
  ros::Timer timer_;

  std::unique_ptr<PeriodicTrack> track_;
  RoiExtractionConfig extraction_config_;
  CandidateConfig candidate_config_;
  std::deque<OdomState> odom_history_;
  std::optional<OdomState> latest_odom_;
  std::vector<AbsoluteObstacle> latest_absolute_obstacles_;
  ObstacleMemory obstacle_memory_;

  std::string controller_config_;
  std::string vehicle_config_;
  std::string track_csv_;
  std::string odom_topic_;
  std::string cloud_topic_;
  std::string local_reference_topic_;
  std::string left_path_topic_;
  std::string right_path_topic_;
  std::string selected_path_topic_;
  std::string valid_area_topic_;
  std::string health_topic_;
  std::string speed_cap_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string diagnostics_topic_;
  std::string world_frame_;
  std::string roi_primary_frame_;
  std::string roi_secondary_frame_;
  std::string last_input_error_{"waiting_for_inputs"};
  std::string state_reason_{"startup"};
  std::string selected_side_{"none"};

  double odom_timeout_s_{};
  double cloud_timeout_s_{};
  double cloud_measurement_max_age_s_{};
  double cloud_future_tolerance_s_{};
  double odom_history_duration_s_{};
  double maximum_odom_bracket_gap_s_{};
  double maximum_odom_extrapolation_s_{};
  double obstacle_memory_timeout_s_{};
  double unknown_obstacle_speed_mps_{};
  double obstacle_speed_min_mps_{};
  double obstacle_speed_max_mps_{};
  double obstacle_speed_filter_alpha_{};
  double obstacle_association_distance_m_{};
  bool single_target_mode_{false};
  double sample_ds_{};
  double prepare_time_s_{};
  double follow_target_loss_timeout_s_{};
  double return_confirm_gap_m_{};
  double return_ey_tolerance_m_{};
  double abort_recovery_distance_m_{};
  bool graceful_pass_target_loss_enabled_{false};
  double pass_target_loss_hold_s_{};
  double pass_target_loss_return_retry_timeout_s_{};
  double pass_target_loss_speed_cap_mps_{};
  double update_rate_hz_{};
  double cruise_speed_cap_mps_{};
  double pass_speed_cap_mps_{};
  double follow_minimum_speed_mps_{};
  double follow_gap_m_{};
  double follow_gap_gain_{};
  int confirmation_minimum_frames_{};

  State state_{State::kFree};
  CandidatePath accepted_path_;
  ros::Time latest_cloud_stamp_;
  ros::WallTime latest_odom_wall_;
  ros::WallTime last_cloud_received_wall_;
  ros::WallTime latest_cloud_wall_;
  ros::WallTime last_nonempty_cloud_wall_;
  ros::WallTime state_started_wall_;
  ros::WallTime confirmation_started_wall_;
  ros::WallTime confirmation_last_wall_;
  ros::WallTime target_loss_started_wall_;
  bool target_loss_recovery_active_{false};
  double confirmation_minimum_duration_s_{};
  double confirmation_maximum_gap_s_{};
  int confirmation_frames_{};
  std::size_t accepted_cloud_sequence_{};
  std::size_t confirmation_sequence_{};
};

}  // namespace
}  // namespace f1tenth_dynamic_mpcc::v5_jubu

int main(int argc, char** argv) {
  ros::init(argc, argv, "v5_jubu_local_planner_cpp");
  try {
    f1tenth_dynamic_mpcc::v5_jubu::V5JubuLocalPlanner planner;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("V5 Jubu C++ planner failed: " << error.what());
    return 1;
  }
  return 0;
}
