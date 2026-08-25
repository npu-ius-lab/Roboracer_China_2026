#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::string normalizeFrame(std::string frame) {
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

class PersistenceFilter {
 public:
  explicit PersistenceFilter(int required_samples)
      : required_samples_(required_samples) {}

  std::uint8_t update(std::uint8_t value) {
    if (!initialized_) {
      force(value);
      return accepted_;
    }

    if (value == accepted_) {
      candidate_ = accepted_;
      candidate_count_ = 0;
    } else if (value == candidate_) {
      ++candidate_count_;
      if (candidate_count_ >= required_samples_) {
        accepted_ = candidate_;
        candidate_count_ = 0;
      }
    } else {
      candidate_ = value;
      candidate_count_ = 1;
      if (candidate_count_ >= required_samples_) {
        accepted_ = candidate_;
        candidate_count_ = 0;
      }
    }
    return accepted_;
  }

  void force(std::uint8_t value) {
    initialized_ = true;
    accepted_ = value;
    candidate_ = value;
    candidate_count_ = 0;
  }

 private:
  int required_samples_{1};
  int candidate_count_{0};
  std::uint8_t accepted_{0};
  std::uint8_t candidate_{0};
  bool initialized_{false};
};

class SemanticRegionNode {
 public:
  SemanticRegionNode() : private_("~"), filter_(readPersistenceSamples()) {
    private_.param("odom_topic", odom_topic_,
                   std::string("/localization/vehicle_odom"));
    private_.param("state_topic", state_topic_,
                   std::string("/semantic_region/state"));
    private_.param("raw_state_topic", raw_state_topic_,
                   std::string("/semantic_region/raw_state"));
    private_.param("name_topic", name_topic_,
                   std::string("/semantic_region/name"));
    private_.param("expected_frame_id", expected_frame_id_, std::string("map"));
    private_.param("require_expected_frame", require_expected_frame_, true);
    private_.param("publish_raw_state", publish_raw_state_, true);
    private_.param("update_rate_hz", update_rate_hz_, 10.0);
    private_.param("odom_timeout_s", odom_timeout_s_, 0.5);

    private_.param("origin_x", origin_x_, -15.217452);
    private_.param("origin_y", origin_y_, -3.122564);
    private_.param("resolution", resolution_, 0.05);
    private_.param("image_width", image_width_, 615);
    private_.param("image_height", image_height_, 403);

    private_.param("right_region_u_min", right_region_u_min_, 415.0);
    private_.param("middle_region_u_min", middle_region_u_min_, 300.0);
    private_.param("upper_regions_v_max", upper_regions_v_max_, 315.0);
    private_.param("left_lower_u_max", left_lower_u_max_, 300.0);
    private_.param("left_lower_v_min", left_lower_v_min_, 190.0);
    private_.param("left_upper_u_max", left_upper_u_max_, 245.0);
    private_.param("left_upper_v_max", left_upper_v_max_, 215.0);
    private_.param("teardrop_center_u", teardrop_center_u_, 285.0);
    private_.param("teardrop_center_v", teardrop_center_v_, 225.0);
    private_.param("teardrop_radius_u", teardrop_radius_u_, 52.0);
    private_.param("teardrop_radius_v", teardrop_radius_v_, 66.0);

    region_names_ = {"unknown",          "obstacle_slalom",
                     "dual_channel_bends", "teardrop_transition",
                     "middle_switchbacks", "right_pear_loop",
                     "outer_return_straight"};
    std::vector<std::string> configured_names;
    if (private_.getParam("region_names", configured_names)) {
      if (configured_names.size() != region_names_.size()) {
        throw std::runtime_error("region_names must contain entries 0 through 6");
      }
      region_names_ = configured_names;
    }

    validateParameters();
    expected_frame_id_ = normalizeFrame(expected_frame_id_);

    state_publisher_ = node_.advertise<std_msgs::UInt8>(state_topic_, 10, true);
    name_publisher_ = node_.advertise<std_msgs::String>(name_topic_, 10, true);
    if (publish_raw_state_) {
      raw_state_publisher_ =
          node_.advertise<std_msgs::UInt8>(raw_state_topic_, 10, true);
    }
    odom_subscriber_ = node_.subscribe(
        odom_topic_, 1, &SemanticRegionNode::odomCallback, this,
        ros::TransportHints().tcpNoDelay());
    timer_ = node_.createTimer(ros::Duration(1.0 / update_rate_hz_),
                               &SemanticRegionNode::timerCallback, this);

    ROS_INFO("semantic region node: odom=%s state=%s rate=%.1f Hz persistence=%d",
             odom_topic_.c_str(), state_topic_.c_str(), update_rate_hz_,
             persistence_samples_);
  }

 private:
  int readPersistenceSamples() {
    private_.param("persistence_samples", persistence_samples_, 5);
    if (persistence_samples_ < 1) {
      throw std::runtime_error("persistence_samples must be at least 1");
    }
    return persistence_samples_;
  }

  void validateParameters() const {
    if (!(resolution_ > 0.0) || image_width_ <= 0 || image_height_ <= 0) {
      throw std::runtime_error("semantic map geometry is invalid");
    }
    if (!(update_rate_hz_ > 0.0) || !(odom_timeout_s_ > 0.0)) {
      throw std::runtime_error("update_rate_hz and odom_timeout_s must be positive");
    }
    if (!(teardrop_radius_u_ > 0.0) || !(teardrop_radius_v_ > 0.0)) {
      throw std::runtime_error("teardrop radii must be positive");
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_x_ = message->pose.pose.position.x;
    latest_y_ = message->pose.pose.position.y;
    latest_frame_id_ = normalizeFrame(message->header.frame_id);
    last_odom_wall_time_ = ros::WallTime::now();
    have_odom_ = true;
    stale_reported_ = false;
  }

  std::uint8_t classify(double x, double y) const {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return 0;
    }

    // This is the same world-to-image transform and ordered rule set used to
    // generate the six-color Blender/Gazebo semantic floor texture.
    const double u = (x - origin_x_) / resolution_;
    const double v =
        static_cast<double>(image_height_) - (y - origin_y_) / resolution_;
    if (u < 0.0 || u >= static_cast<double>(image_width_) || v < 0.0 ||
        v >= static_cast<double>(image_height_)) {
      return 0;
    }

    std::uint8_t result = 6;
    if (u >= right_region_u_min_ && v < upper_regions_v_max_) {
      result = 5;
    }
    if (u >= middle_region_u_min_ && u < right_region_u_min_ &&
        v < upper_regions_v_max_) {
      result = 4;
    }
    if (u < left_lower_u_max_ && v >= left_lower_v_min_) {
      result = 2;
    }
    if (u < left_upper_u_max_ && v < left_upper_v_max_) {
      result = 1;
    }
    const double ellipse_u = (u - teardrop_center_u_) / teardrop_radius_u_;
    const double ellipse_v = (v - teardrop_center_v_) / teardrop_radius_v_;
    if (ellipse_u * ellipse_u + ellipse_v * ellipse_v <= 1.0) {
      result = 3;
    }
    return result;
  }

  void publishState(std::uint8_t raw_state, std::uint8_t filtered_state) {
    std_msgs::UInt8 state_message;
    state_message.data = filtered_state;
    state_publisher_.publish(state_message);

    if (publish_raw_state_) {
      std_msgs::UInt8 raw_message;
      raw_message.data = raw_state;
      raw_state_publisher_.publish(raw_message);
    }

    std_msgs::String name_message;
    name_message.data = region_names_.at(filtered_state);
    name_publisher_.publish(name_message);

    if (!have_published_state_ || filtered_state != last_published_state_) {
      ROS_INFO("semantic region changed: raw=%u filtered=%u name=%s x=%.3f y=%.3f",
               static_cast<unsigned int>(raw_state),
               static_cast<unsigned int>(filtered_state),
               name_message.data.c_str(), latest_x_, latest_y_);
    }
    last_published_state_ = filtered_state;
    have_published_state_ = true;
  }

  void publishUnknown(const char* reason) {
    filter_.force(0);
    publishState(0, 0);
    ROS_WARN_THROTTLE(2.0, "semantic region is unknown: %s", reason);
  }

  void timerCallback(const ros::TimerEvent&) {
    if (!have_odom_) {
      ROS_WARN_THROTTLE(2.0, "semantic region waiting for %s",
                        odom_topic_.c_str());
      return;
    }

    const double odom_age =
        (ros::WallTime::now() - last_odom_wall_time_).toSec();
    if (odom_age > odom_timeout_s_) {
      if (!stale_reported_) {
        stale_reported_ = true;
        ROS_WARN("semantic region odometry stale: age=%.3f s", odom_age);
      }
      publishUnknown("odometry timeout");
      return;
    }

    if (require_expected_frame_ && latest_frame_id_ != expected_frame_id_) {
      publishUnknown("unexpected odometry frame");
      return;
    }

    const std::uint8_t raw_state = classify(latest_x_, latest_y_);
    const std::uint8_t filtered_state = filter_.update(raw_state);
    publishState(raw_state, filtered_state);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_;
  ros::Publisher state_publisher_;
  ros::Publisher raw_state_publisher_;
  ros::Publisher name_publisher_;
  ros::Subscriber odom_subscriber_;
  ros::Timer timer_;

  int persistence_samples_{5};
  PersistenceFilter filter_;
  std::string odom_topic_;
  std::string state_topic_;
  std::string raw_state_topic_;
  std::string name_topic_;
  std::string expected_frame_id_;
  std::vector<std::string> region_names_;

  double update_rate_hz_{10.0};
  double odom_timeout_s_{0.5};
  double origin_x_{-15.217452};
  double origin_y_{-3.122564};
  double resolution_{0.05};
  int image_width_{615};
  int image_height_{403};
  double right_region_u_min_{415.0};
  double middle_region_u_min_{300.0};
  double upper_regions_v_max_{315.0};
  double left_lower_u_max_{300.0};
  double left_lower_v_min_{190.0};
  double left_upper_u_max_{245.0};
  double left_upper_v_max_{215.0};
  double teardrop_center_u_{285.0};
  double teardrop_center_v_{225.0};
  double teardrop_radius_u_{52.0};
  double teardrop_radius_v_{66.0};

  double latest_x_{0.0};
  double latest_y_{0.0};
  std::string latest_frame_id_;
  ros::WallTime last_odom_wall_time_;
  std::uint8_t last_published_state_{0};
  bool require_expected_frame_{true};
  bool publish_raw_state_{true};
  bool have_odom_{false};
  bool stale_reported_{false};
  bool have_published_state_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "semantic_region");
  try {
    SemanticRegionNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("semantic region node failed: %s", error.what());
    return 1;
  }
  return 0;
}
