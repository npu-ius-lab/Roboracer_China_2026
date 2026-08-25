#include "f1tenth_dynamic_mpcc/core.hpp"

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace mpcc = f1tenth_dynamic_mpcc;

namespace {

std::string expandHome(std::string path) {
  if (!path.empty() && path.front() == '~') {
    if (const char* home = std::getenv("HOME")) path.replace(0, 1, home);
  }
  return path;
}

std::string normalizeFrame(std::string frame) {
  while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  return frame;
}

std::string timestampedDefaultLog() {
  std::time_t now = std::time(nullptr);
  std::tm local{};
  localtime_r(&now, &local);
  std::ostringstream name;
  name << "~/.ros/f1tenth_mpcc_laps_" << std::put_time(&local, "%Y%m%d_%H%M%S")
       << ".csv";
  return expandHome(name.str());
}

double finiteOrZero(double value) { return std::isfinite(value) ? value : 0.0; }

}  // namespace

class LapTimerNode {
 public:
  LapTimerNode() : private_("~") {
    const std::string package = ros::package::getPath("f1tenth_dynamic_mpcc");
    std::string controller_path, vehicle_path, track_path;
    private_.param("controller_config", controller_path,
                   package + "/config/controller.yaml");
    private_.param("vehicle_config", vehicle_path,
                   package + "/config/vehicle.yaml");
    private_.param("track_csv", track_path,
                   package + "/data/tracks/virtual_track/raceline.csv");
    private_.param("odom_topic", odom_topic_,
                   std::string("/localization/vehicle_odom"));
    private_.param("world_frame", world_frame_, std::string("map"));
    private_.param("minimum_start_speed_mps", minimum_start_speed_, 0.5);
    private_.param("minimum_forward_step_m", minimum_forward_step_, 0.001);
    private_.param("minimum_lap_time_s", minimum_lap_time_, 3.0);
    private_.param("maximum_progress_step_m", maximum_progress_step_, 1.0);
    private_.param("maximum_odom_gap_s", maximum_odom_gap_, 0.5);
    double status_rate = 5.0;
    private_.param("status_rate_hz", status_rate, 5.0);
    private_.param("log_path", log_path_, std::string());

    world_frame_ = normalizeFrame(world_frame_);
    if (minimum_start_speed_ < 0.0 || minimum_forward_step_ < 0.0 ||
        minimum_lap_time_ <= 0.0 || maximum_progress_step_ <= 0.0 ||
        maximum_odom_gap_ <= 0.0 || status_rate <= 0.0) {
      throw std::invalid_argument("invalid lap timer threshold");
    }

    const YAML::Node controller = YAML::LoadFile(controller_path);
    const YAML::Node vehicle_config = YAML::LoadFile(vehicle_path);
    const auto vehicle = mpcc::VehicleParameters::fromYaml(vehicle_config, controller);
    YAML::Node speed = YAML::Clone(controller["speed_planning"]);
    speed["wheelbase_m"] = vehicle.wheelbase;
    speed["max_steer_rad"] = vehicle.max_steer;
    speed["max_steer_rate_radps"] = vehicle.max_steer_rate;
    speed["max_accel_mps2"] = vehicle.max_accel;
    speed["max_decel_mps2"] = vehicle.max_decel;
    speed["lateral_accel_limit_mps2"] = vehicle.lateral_accel_limit;
    track_ = std::make_unique<mpcc::PeriodicTrack>(track_path, speed, vehicle);

    if (log_path_.empty()) log_path_ = timestampedDefaultLog();
    log_path_ = expandHome(log_path_);
    const std::filesystem::path path(log_path_);
    if (!path.parent_path().empty())
      std::filesystem::create_directories(path.parent_path());
    log_.open(log_path_, std::ios::out | std::ios::app);
    if (!log_) throw std::runtime_error("cannot open lap log: " + log_path_);
    if (std::filesystem::file_size(path) == 0) {
      log_ << "wall_time_s,odom_crossing_stamp_s,lap,lap_time_s,best_lap_time_s,"
              "average_lap_time_s,mean_speed_mps,max_speed_mps,"
              "mean_abs_contour_error_m,max_abs_contour_error_m,timing_line_s_m,"
              "track_length_m\n";
      log_.flush();
    }

    current_pub_ = node_.advertise<std_msgs::Float64>(
        "/f1tenth_mpcc/lap_timer/current_lap_time", 1, true);
    last_pub_ = node_.advertise<std_msgs::Float64>(
        "/f1tenth_mpcc/lap_timer/last_lap_time", 1, true);
    best_pub_ = node_.advertise<std_msgs::Float64>(
        "/f1tenth_mpcc/lap_timer/best_lap_time", 1, true);
    count_pub_ = node_.advertise<std_msgs::UInt32>(
        "/f1tenth_mpcc/lap_timer/lap_count", 1, true);
    status_pub_ = node_.advertise<std_msgs::String>(
        "/f1tenth_mpcc/lap_timer/status", 1, true);
    marker_pub_ = node_.advertise<visualization_msgs::MarkerArray>(
        "/f1tenth_mpcc/lap_timer/markers", 1, true);

    odom_sub_ = node_.subscribe(odom_topic_, 20, &LapTimerNode::odomCallback, this,
                                ros::TransportHints().tcpNoDelay());
    status_timer_ = node_.createTimer(ros::Duration(1.0 / status_rate),
                                      &LapTimerNode::statusTimer, this);
    publishStatus();
    ROS_INFO("Lap timer ready: track=%.3f m, arm speed=%.2f m/s, odom=%s, log=%s",
             track_->length(), minimum_start_speed_, odom_topic_.c_str(),
             log_path_.c_str());
  }

  ~LapTimerNode() {
    if (log_) log_.flush();
  }

 private:
  void resetMetrics() {
    metric_time_ = 0.0;
    speed_integral_ = 0.0;
    contour_integral_ = 0.0;
    maximum_speed_ = 0.0;
    maximum_contour_ = 0.0;
  }

  void invalidateCurrentLap(const std::string& reason) {
    if (armed_)
      ROS_WARN("Lap timer discarded current lap: %s", reason.c_str());
    armed_ = false;
    resetMetrics();
  }

  void arm(double s, double stamp) {
    armed_ = true;
    lap_start_s_ = s;
    timing_line_s_ = s;
    lap_start_stamp_ = stamp;
    resetMetrics();
    ROS_INFO("Lap timer ARMED at s=%.3f m; lap %u started", track_->wrapS(s),
             lap_count_ + 1);
  }

  void integrate(double dt, double speed0, double speed1, double error0,
                 double error1) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;
    const double v0 = std::max(0.0, finiteOrZero(speed0));
    const double v1 = std::max(0.0, finiteOrZero(speed1));
    const double e0 = std::abs(finiteOrZero(error0));
    const double e1 = std::abs(finiteOrZero(error1));
    metric_time_ += dt;
    speed_integral_ += 0.5 * (v0 + v1) * dt;
    contour_integral_ += 0.5 * (e0 + e1) * dt;
    maximum_speed_ = std::max({maximum_speed_, v0, v1});
    maximum_contour_ = std::max({maximum_contour_, e0, e1});
  }

  void completeLap(double crossing_stamp) {
    const double lap_time = crossing_stamp - lap_start_stamp_;
    if (!std::isfinite(lap_time) || lap_time < minimum_lap_time_) {
      ROS_WARN("Lap timer rejected implausible lap time %.3f s", lap_time);
      lap_start_stamp_ = crossing_stamp;
      lap_start_s_ += track_->length();
      resetMetrics();
      return;
    }

    ++lap_count_;
    last_lap_time_ = lap_time;
    best_lap_time_ = std::min(best_lap_time_, lap_time);
    total_lap_time_ += lap_time;
    const double average_lap = total_lap_time_ / static_cast<double>(lap_count_);
    const double mean_speed = metric_time_ > 0.0 ? speed_integral_ / metric_time_ : 0.0;
    const double mean_contour =
        metric_time_ > 0.0 ? contour_integral_ / metric_time_ : 0.0;

    log_ << std::fixed << std::setprecision(9) << ros::WallTime::now().toSec()
         << ',' << crossing_stamp << ',' << lap_count_ << ',' << lap_time << ','
         << best_lap_time_ << ',' << average_lap << ',' << mean_speed << ','
         << maximum_speed_ << ',' << mean_contour << ',' << maximum_contour_ << ','
         << track_->wrapS(timing_line_s_) << ',' << track_->length() << '\n';
    log_.flush();

    ROS_WARN("LAP %u: %.3f s | best %.3f s | avg %.3f s | mean speed %.2f m/s | max |ey| %.3f m",
             lap_count_, lap_time, best_lap_time_, average_lap, mean_speed,
             maximum_contour_);

    lap_start_stamp_ = crossing_stamp;
    lap_start_s_ += track_->length();
    resetMetrics();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    const std::string frame = normalizeFrame(message->header.frame_id);
    if (!frame.empty() && frame != world_frame_) {
      ROS_WARN_THROTTLE(2.0, "Lap timer rejecting odom frame %s (expected %s)",
                        frame.c_str(), world_frame_.c_str());
      return;
    }
    const double stamp = message->header.stamp.isZero()
                             ? ros::Time::now().toSec()
                             : message->header.stamp.toSec();
    const double x = message->pose.pose.position.x;
    const double y = message->pose.pose.position.y;
    const double speed = message->twist.twist.linear.x;
    if (!std::isfinite(stamp) || !std::isfinite(x) || !std::isfinite(y) ||
        !std::isfinite(speed))
      return;
    if (previous_stamp_ && stamp <= *previous_stamp_) {
      ROS_WARN_THROTTLE(2.0, "Lap timer dropped timestamp regression %.6f s",
                        stamp - *previous_stamp_);
      return;
    }

    mpcc::TrackProjection projection;
    try {
      projection = track_->project(x, y, projection_guess_);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(2.0, "Lap timer projection failed: %s", error.what());
      return;
    }
    projection_guess_ = projection.s;
    latest_s_ = projection.s;
    latest_stamp_ = stamp;
    latest_x_ = x;
    latest_y_ = y;
    latest_frame_ = frame.empty() ? world_frame_ : frame;

    if (!previous_s_) {
      previous_s_ = projection.s;
      previous_stamp_ = stamp;
      previous_speed_ = speed;
      previous_contour_ = projection.e_contour;
      if (speed >= minimum_start_speed_) arm(projection.s, stamp);
      return;
    }

    const double dt = stamp - *previous_stamp_;
    const double ds = projection.s - *previous_s_;
    if (dt > maximum_odom_gap_ || std::abs(ds) > maximum_progress_step_) {
      std::ostringstream reason;
      reason << "odom discontinuity dt=" << std::fixed << std::setprecision(3)
             << dt << " s ds=" << ds << " m";
      invalidateCurrentLap(reason.str());
      previous_s_ = projection.s;
      previous_stamp_ = stamp;
      previous_speed_ = speed;
      previous_contour_ = projection.e_contour;
      return;
    }

    if (!armed_) {
      if (speed >= minimum_start_speed_ && ds >= minimum_forward_step_)
        arm(projection.s, stamp);
    } else {
      const double target = lap_start_s_ + track_->length();
      if (ds > 0.0 && *previous_s_ < target && projection.s >= target) {
        const double fraction = std::clamp((target - *previous_s_) / ds, 0.0, 1.0);
        const double crossing_stamp = *previous_stamp_ + fraction * dt;
        const double crossing_speed = previous_speed_ + fraction * (speed - previous_speed_);
        const double crossing_contour =
            previous_contour_ + fraction * (projection.e_contour - previous_contour_);
        integrate(fraction * dt, previous_speed_, crossing_speed, previous_contour_,
                  crossing_contour);
        completeLap(crossing_stamp);
        integrate((1.0 - fraction) * dt, crossing_speed, speed, crossing_contour,
                  projection.e_contour);
      } else {
        integrate(dt, previous_speed_, speed, previous_contour_, projection.e_contour);
      }
    }

    previous_s_ = projection.s;
    previous_stamp_ = stamp;
    previous_speed_ = speed;
    previous_contour_ = projection.e_contour;
  }

  double currentLapTime() const {
    return armed_ && latest_stamp_ >= lap_start_stamp_
               ? latest_stamp_ - lap_start_stamp_
               : 0.0;
  }

  double currentProgress() const {
    if (!armed_) return 0.0;
    return std::clamp((latest_s_ - lap_start_s_) / track_->length(), 0.0, 1.0);
  }

  void publishMarkers() {
    visualization_msgs::MarkerArray array;
    const ros::Time now = ros::Time::now();
    const std::string frame = latest_frame_.empty() ? world_frame_ : latest_frame_;
    const double line_s = armed_ ? timing_line_s_ : 0.0;
    const auto geometry = track_->geometry(line_s);
    const double nx = -std::sin(geometry.yaw);
    const double ny = std::cos(geometry.yaw);

    visualization_msgs::Marker line;
    line.header.frame_id = frame;
    line.header.stamp = now;
    line.ns = "f1tenth_lap_timer";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.08;
    line.color.r = 1.0;
    line.color.g = armed_ ? 1.0 : 0.5;
    line.color.b = armed_ ? 1.0 : 0.0;
    line.color.a = 1.0;
    geometry_msgs::Point left, right;
    left.x = geometry.x + nx * geometry.width_left;
    left.y = geometry.y + ny * geometry.width_left;
    left.z = 0.08;
    right.x = geometry.x - nx * geometry.width_right;
    right.y = geometry.y - ny * geometry.width_right;
    right.z = 0.08;
    line.points = {left, right};
    array.markers.push_back(line);

    visualization_msgs::Marker text;
    text.header.frame_id = frame;
    text.header.stamp = now;
    text.ns = "f1tenth_lap_timer";
    text.id = 1;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.orientation.w = 1.0;
    text.pose.position.x = latest_x_;
    text.pose.position.y = latest_y_;
    text.pose.position.z = 0.8;
    text.scale.z = 0.28;
    text.color.r = 1.0;
    text.color.g = 0.9;
    text.color.b = 0.1;
    text.color.a = 1.0;
    std::ostringstream label;
    label << std::fixed << std::setprecision(2);
    if (armed_)
      label << "LAP " << (lap_count_ + 1) << "  " << currentLapTime() << " s  "
            << std::setprecision(0) << currentProgress() * 100.0 << "%";
    else
      label << "LAP TIMER: waiting for forward motion";
    label << std::setprecision(2);
    if (lap_count_ > 0)
      label << "\nLAST " << last_lap_time_ << " s  BEST " << best_lap_time_ << " s";
    text.text = label.str();
    array.markers.push_back(text);
    marker_pub_.publish(array);
  }

  void publishStatus() {
    std_msgs::Float64 current, last, best;
    std_msgs::UInt32 count;
    std_msgs::String status;
    current.data = currentLapTime();
    last.data = lap_count_ > 0 ? last_lap_time_ : 0.0;
    best.data = lap_count_ > 0 ? best_lap_time_ : 0.0;
    count.data = lap_count_;
    std::ostringstream json;
    json << std::fixed << std::setprecision(6)
         << "{\"armed\":" << (armed_ ? "true" : "false")
         << ",\"lap\":" << lap_count_
         << ",\"current_lap_time_s\":" << current.data
         << ",\"last_lap_time_s\":" << last.data
         << ",\"best_lap_time_s\":" << best.data
         << ",\"progress\":" << currentProgress()
         << ",\"track_length_m\":" << track_->length()
         << ",\"log_path\":\"" << log_path_ << "\"}";
    status.data = json.str();
    current_pub_.publish(current);
    last_pub_.publish(last);
    best_pub_.publish(best);
    count_pub_.publish(count);
    status_pub_.publish(status);
    publishMarkers();
  }

  void statusTimer(const ros::TimerEvent&) {
    publishStatus();
    if (armed_)
      ROS_INFO_THROTTLE(5.0, "Lap timer: lap %u current %.2f s progress %.1f%% best %.3f s",
                        lap_count_ + 1, currentLapTime(), currentProgress() * 100.0,
                        lap_count_ > 0 ? best_lap_time_ : 0.0);
  }

  ros::NodeHandle node_, private_;
  std::unique_ptr<mpcc::PeriodicTrack> track_;
  ros::Subscriber odom_sub_;
  ros::Publisher current_pub_, last_pub_, best_pub_, count_pub_, status_pub_,
      marker_pub_;
  ros::Timer status_timer_;
  std::ofstream log_;
  std::string odom_topic_, world_frame_, latest_frame_, log_path_;

  double minimum_start_speed_{0.5};
  double minimum_forward_step_{0.001};
  double minimum_lap_time_{3.0};
  double maximum_progress_step_{1.0};
  double maximum_odom_gap_{0.5};
  std::optional<double> projection_guess_, previous_s_, previous_stamp_;
  double previous_speed_{0.0}, previous_contour_{0.0};
  bool armed_{false};
  double lap_start_s_{0.0}, timing_line_s_{0.0}, lap_start_stamp_{0.0};
  double latest_s_{0.0}, latest_stamp_{0.0}, latest_x_{0.0}, latest_y_{0.0};
  std::uint32_t lap_count_{0};
  double last_lap_time_{0.0};
  double best_lap_time_{std::numeric_limits<double>::infinity()};
  double total_lap_time_{0.0};
  double metric_time_{0.0}, speed_integral_{0.0}, contour_integral_{0.0};
  double maximum_speed_{0.0}, maximum_contour_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "f1tenth_mpcc_lap_timer");
  try {
    LapTimerNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Lap timer initialization failed: %s", error.what());
    return 1;
  }
  return 0;
}
