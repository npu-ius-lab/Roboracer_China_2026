#include "roboracer_react_overtake/minimum_speed_gate.hpp"

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <cmath>
#include <iomanip>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace roboracer_react_overtake {
namespace {

bool containsJsonString(const std::string& json, const std::string& key,
                        const std::string& value) {
  const std::regex expression("\\\"" + key +
                              "\\\"\\s*:\\s*\\\"" + value +
                              "\\\"");
  return std::regex_search(json, expression);
}

bool containsJsonBool(const std::string& json, const std::string& key,
                      bool value) {
  const std::regex expression("\\\"" + key +
                              "\\\"\\s*:\\s*" +
                              (value ? "true" : "false"));
  return std::regex_search(json, expression);
}

double wallAge(const ros::WallTime& stamp) {
  if (stamp.isZero()) return INFINITY;
  return (ros::WallTime::now() - stamp).toSec();
}

}  // namespace

class MinimumSpeedGateNode {
 public:
  MinimumSpeedGateNode() : private_node_("~") {
    private_node_.param("minimum_speed_mps", minimum_speed_mps_, 1.0);
    private_node_.param("input_timeout_s", input_timeout_s_, 0.35);
    private_node_.param("supervisor_timeout_s", supervisor_timeout_s_, 0.35);
    private_node_.param("require_supervisor_running",
                        require_supervisor_running_, true);
    private_node_.param("raw_command_topic", raw_command_topic_,
                        std::string("/roboracer_react/mpcc/raw_ackermann_cmd_stamped"));
    private_node_.param("output_command_topic", output_command_topic_,
                        std::string("/roboracer_react/mpcc/ackermann_cmd_stamped"));
    private_node_.param("planner_state_topic", planner_state_topic_,
                        std::string("/roboracer_react/state"));
    private_node_.param("planner_health_topic", planner_health_topic_,
                        std::string("/roboracer_react/health"));
    private_node_.param(
        "supervisor_state_topic", supervisor_state_topic_,
        std::string("/automatic_relaunch_supervisor_roboracer_react/state"));
    private_node_.param("status_topic", status_topic_,
                        std::string("/roboracer_react/min_speed_gate/status"));

    if (!std::isfinite(minimum_speed_mps_) || minimum_speed_mps_ < 0.0 ||
        !std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0 ||
        !std::isfinite(supervisor_timeout_s_) ||
        supervisor_timeout_s_ <= 0.0) {
      throw std::invalid_argument("invalid minimum-speed gate parameters");
    }

    output_publisher_ = node_.advertise<ackermann_msgs::AckermannDriveStamped>(
        output_command_topic_, 5);
    status_publisher_ = node_.advertise<std_msgs::String>(status_topic_, 5);
    state_subscriber_ = node_.subscribe(
        planner_state_topic_, 5, &MinimumSpeedGateNode::stateCallback, this);
    health_subscriber_ = node_.subscribe(
        planner_health_topic_, 5, &MinimumSpeedGateNode::healthCallback, this);
    supervisor_subscriber_ = node_.subscribe(
        supervisor_state_topic_, 5,
        &MinimumSpeedGateNode::supervisorCallback, this);
    command_subscriber_ = node_.subscribe(
        raw_command_topic_, 5, &MinimumSpeedGateNode::commandCallback, this);

    ROS_INFO_STREAM("V3 React minimum-speed gate: driving-state floor="
                    << minimum_speed_mps_ << " m/s, supervisor_required="
                    << (require_supervisor_running_ ? "true" : "false"));
  }

 private:
  void stateCallback(const std_msgs::String::ConstPtr& message) {
    planner_state_ = message->data;
    state_received_wall_ = ros::WallTime::now();
  }

  void healthCallback(const std_msgs::Bool::ConstPtr& message) {
    planner_healthy_ = message->data;
    health_received_wall_ = ros::WallTime::now();
  }

  void supervisorCallback(const std_msgs::String::ConstPtr& message) {
    supervisor_running_ = containsJsonString(message->data, "state", "RUNNING");
    controller_enabled_ =
        containsJsonBool(message->data, "controller_disabled", false);
    supervisor_received_wall_ = ros::WallTime::now();
  }

  void commandCallback(
      const ackermann_msgs::AckermannDriveStamped::ConstPtr& message) {
    MinimumSpeedGateInput input;
    input.planner_state = planner_state_;
    input.planner_healthy = planner_healthy_;
    input.planner_state_fresh =
        wallAge(state_received_wall_) <= input_timeout_s_;
    input.planner_health_fresh =
        wallAge(health_received_wall_) <= input_timeout_s_;
    input.require_supervisor_running = require_supervisor_running_;
    input.supervisor_fresh =
        wallAge(supervisor_received_wall_) <= supervisor_timeout_s_;
    input.supervisor_running = supervisor_running_;
    input.controller_enabled = controller_enabled_;

    const auto result = applyMinimumSpeedGate(
        message->drive.speed, minimum_speed_mps_, input);
    auto output = *message;
    output.drive.speed = result.speed;
    output_publisher_.publish(output);

    std::ostringstream status;
    status << std::fixed << std::setprecision(3)
           << "{\"floor_applied\":"
           << (result.floor_applied ? "true" : "false")
           << ",\"reason\":\"" << result.reason
           << "\",\"planner_state\":\"" << planner_state_
           << "\",\"raw_speed\":" << message->drive.speed
           << ",\"output_speed\":" << output.drive.speed << "}";
    std_msgs::String status_message;
    status_message.data = status.str();
    status_publisher_.publish(status_message);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher output_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber health_subscriber_;
  ros::Subscriber supervisor_subscriber_;

  double minimum_speed_mps_{1.0};
  double input_timeout_s_{0.35};
  double supervisor_timeout_s_{0.35};
  bool require_supervisor_running_{true};
  bool planner_healthy_{false};
  bool supervisor_running_{false};
  bool controller_enabled_{false};
  std::string planner_state_{"ABORT"};
  ros::WallTime state_received_wall_;
  ros::WallTime health_received_wall_;
  ros::WallTime supervisor_received_wall_;
  std::string raw_command_topic_;
  std::string output_command_topic_;
  std::string planner_state_topic_;
  std::string planner_health_topic_;
  std::string supervisor_state_topic_;
  std::string status_topic_;
};

}  // namespace roboracer_react_overtake

int main(int argc, char** argv) {
  ros::init(argc, argv, "roboracer_react_minimum_speed_gate");
  try {
    roboracer_react_overtake::MinimumSpeedGateNode node;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("V3 React minimum-speed gate failed: " << error.what());
    return 1;
  }
  return 0;
}
