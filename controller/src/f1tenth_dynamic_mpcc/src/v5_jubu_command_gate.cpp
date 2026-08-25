#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

double age(const ros::WallTime& stamp) {
  if (stamp.toSec() <= 0.0) return 1e99;
  return std::max(0.0, (ros::WallTime::now() - stamp).toSec());
}

class V5JubuCommandGate {
 public:
  V5JubuCommandGate() : private_node_("~") {
    bool hardware_allowed = false;
    private_node_.param("allow_real_hardware", hardware_allowed, false);
    if (!hardware_allowed) {
      throw std::runtime_error("allow_real_hardware must be true");
    }
    private_node_.param(
        "input_topic", input_topic_,
        std::string("/f1tenth_mpcc/v5_jubu/ackermann_cmd_stamped"));
    private_node_.param("output_topic", output_topic_,
                        std::string("/tianracer/ackermann_cmd"));
    private_node_.param("planner_health_topic", health_topic_,
                        std::string("/v5_jubu/health"));
    private_node_.param("mpcc_enable_service", mpcc_enable_service_,
                        std::string(
                            "/f1tenth_dynamic_mpcc_v5_jubu/set_enabled"));
    private_node_.param("state_topic", state_topic_,
                        std::string("/v5_jubu/command_gate_state"));
    private_node_.param("start_armed", armed_, false);
    private_node_.param("command_timeout_s", command_timeout_s_, 0.25);
    private_node_.param("planner_health_timeout_s", health_timeout_s_, 0.50);
    private_node_.param("command_startup_timeout_s", startup_timeout_s_, 1.0);
    private_node_.param("minimum_forward_speed_mps",
                        minimum_forward_speed_mps_, 0.0);
    double publish_rate_hz = 60.0;
    private_node_.param("publish_rate_hz", publish_rate_hz, 60.0);
    if (command_timeout_s_ <= 0.0 || health_timeout_s_ <= 0.0 ||
        startup_timeout_s_ <= 0.0 || publish_rate_hz <= 0.0 ||
        !std::isfinite(minimum_forward_speed_mps_) ||
        minimum_forward_speed_mps_ < 0.0) {
      throw std::runtime_error("invalid V5 Jubu command-gate timing");
    }

    publisher_ =
        node_.advertise<ackermann_msgs::AckermannDrive>(output_topic_, 1);
    state_publisher_ = node_.advertise<std_msgs::String>(state_topic_, 5);
    command_subscriber_ = node_.subscribe(
        input_topic_, 1, &V5JubuCommandGate::commandCallback, this,
        ros::TransportHints().tcpNoDelay());
    health_subscriber_ = node_.subscribe(
        health_topic_, 5, &V5JubuCommandGate::healthCallback, this,
        ros::TransportHints().tcpNoDelay());
    enable_client_ = node_.serviceClient<std_srvs::SetBool>(
        mpcc_enable_service_, false);
    arm_service_ = private_node_.advertiseService(
        "set_armed", &V5JubuCommandGate::setArmed, this);
    timer_ = node_.createTimer(
        ros::Duration(1.0 / publish_rate_hz),
        &V5JubuCommandGate::timerCallback, this);
    publishZero();
    ROS_WARN_STREAM("V5 Jubu C++ hardware gate output=" << output_topic_
                    << " start_armed=" << (armed_ ? "true" : "false")
                    << " minimum_forward_speed="
                    << minimum_forward_speed_mps_ << " m/s");
  }

  ~V5JubuCommandGate() {
    publishZero();
    requestMpcc(false);
  }

 private:
  void commandCallback(
      const ackermann_msgs::AckermannDriveStamped::ConstPtr& message) {
    const auto& command = message->drive;
    if (!std::isfinite(command.speed) ||
        !std::isfinite(command.steering_angle) ||
        !std::isfinite(command.acceleration) ||
        !std::isfinite(command.jerk) ||
        !std::isfinite(command.steering_angle_velocity) ||
        command.speed < -0.01) {
      fault_latched_ = true;
      fault_reason_ = "invalid_mpcc_command";
      command_.reset();
      return;
    }
    command_ = command;
    command_wall_ = ros::WallTime::now();
    if (mpcc_enabled_) command_seen_since_enable_ = true;
  }

  void healthCallback(const std_msgs::Bool::ConstPtr& message) {
    planner_healthy_ = message->data;
    health_wall_ = ros::WallTime::now();
  }

  bool healthFreshAndTrue() const {
    return planner_healthy_ && age(health_wall_) <= health_timeout_s_;
  }

  bool requestMpcc(bool enabled) {
    std_srvs::SetBool service;
    service.request.data = enabled;
    if (!enable_client_.call(service) || !service.response.success) {
      ROS_WARN_THROTTLE(1.0, "V5 Jubu gate could not set MPCC enabled=%s",
                        enabled ? "true" : "false");
      if (enabled) mpcc_enabled_ = false;
      return false;
    }
    mpcc_enabled_ = enabled;
    if (enabled) {
      enabled_wall_ = ros::WallTime::now();
      command_seen_since_enable_ = false;
      command_.reset();
    }
    return true;
  }

  bool setArmed(std_srvs::SetBool::Request& request,
                std_srvs::SetBool::Response& response) {
    if (!request.data) {
      armed_ = false;
      fault_latched_ = false;
      fault_reason_ = "operator_disarmed";
      requestMpcc(false);
      publishZero();
      response.success = true;
      response.message = "V5 Jubu hardware gate disarmed";
      return true;
    }
    if (!healthFreshAndTrue()) {
      response.success = false;
      response.message = "fresh true /v5_jubu/health is required";
      return true;
    }
    armed_ = true;
    fault_latched_ = false;
    fault_reason_ = "none";
    response.success = true;
    response.message = "V5 Jubu hardware gate armed";
    return true;
  }

  void publishZero() {
    publisher_.publish(ackermann_msgs::AckermannDrive{});
  }

  void publishState(const std::string& state) {
    std_msgs::String message;
    message.data = std::string("{\"state\":\"") + state +
                   "\",\"armed\":" + (armed_ ? "true" : "false") +
                   ",\"planner_healthy\":" +
                   (healthFreshAndTrue() ? "true" : "false") +
                   ",\"mpcc_enabled\":" +
                   (mpcc_enabled_ ? "true" : "false") +
                   ",\"fault_latched\":" +
                   (fault_latched_ ? "true" : "false") +
                   ",\"minimum_forward_speed_mps\":" +
                   std::to_string(minimum_forward_speed_mps_) +
                   ",\"reason\":\"" + fault_reason_ + "\"}";
    state_publisher_.publish(message);
  }

  void timerCallback(const ros::TimerEvent&) {
    const bool health_ok = healthFreshAndTrue();
    if (!armed_ || !health_ok || fault_latched_) {
      if (mpcc_enabled_) requestMpcc(false);
      publishZero();
      publishState(fault_latched_ ? "FAULT" :
                   (armed_ ? "WAIT_HEALTH" : "DISARMED"));
      return;
    }
    if (!mpcc_enabled_) {
      publishZero();
      requestMpcc(true);
      publishState(mpcc_enabled_ ? "WAIT_COMMAND" : "WAIT_MPCC_SERVICE");
      return;
    }
    const double command_age = age(command_wall_);
    if (!command_seen_since_enable_) {
      if (age(enabled_wall_) > startup_timeout_s_) {
        fault_latched_ = true;
        fault_reason_ = "mpcc_command_startup_timeout";
        requestMpcc(false);
      }
      publishZero();
      publishState(fault_latched_ ? "FAULT" : "WAIT_COMMAND");
      return;
    }
    if (!command_.has_value() || command_age > command_timeout_s_) {
      fault_latched_ = true;
      fault_reason_ = "mpcc_command_stale";
      requestMpcc(false);
      publishZero();
      publishState("FAULT");
      return;
    }
    auto forwarded = *command_;
    // This floor is applied only after all arming, planner-health, MPCC-enable
    // and command-freshness checks above have passed. DISARMED, WAIT_HEALTH,
    // WAIT_COMMAND and FAULT paths continue to publish an exact zero.
    forwarded.speed = std::max(
        forwarded.speed, static_cast<float>(minimum_forward_speed_mps_));
    publisher_.publish(forwarded);
    publishState("FORWARDING");
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher publisher_;
  ros::Publisher state_publisher_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber health_subscriber_;
  ros::ServiceClient enable_client_;
  ros::ServiceServer arm_service_;
  ros::Timer timer_;
  std::optional<ackermann_msgs::AckermannDrive> command_;
  ros::WallTime command_wall_;
  ros::WallTime health_wall_;
  ros::WallTime enabled_wall_;
  std::string input_topic_;
  std::string output_topic_;
  std::string health_topic_;
  std::string mpcc_enable_service_;
  std::string state_topic_;
  std::string fault_reason_{"none"};
  double command_timeout_s_{};
  double health_timeout_s_{};
  double startup_timeout_s_{};
  double minimum_forward_speed_mps_{0.0};
  bool planner_healthy_{false};
  bool armed_{false};
  bool mpcc_enabled_{false};
  bool command_seen_since_enable_{false};
  bool fault_latched_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "v5_jubu_command_gate_cpp");
  try {
    V5JubuCommandGate gate;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("V5 Jubu C++ command gate failed: " << error.what());
    return 1;
  }
  return 0;
}
