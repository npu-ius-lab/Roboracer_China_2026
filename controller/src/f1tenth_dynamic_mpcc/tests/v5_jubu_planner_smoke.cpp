#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct TrackStart {
  double x{};
  double y{};
  double yaw{};
  double obstacle_x{3.0};
  double obstacle_y{};
};

TrackStart loadTrackStart(const std::string& path, double requested_s) {
  std::ifstream stream(path);
  std::string header;
  std::string line;
  if (!std::getline(stream, header) || !std::getline(stream, line)) {
    throw std::runtime_error("cannot read track start from " + path);
  }
  auto parse = [](std::string row, double& s, double& x, double& y,
                  double& yaw) {
    std::replace(row.begin(), row.end(), ',', ' ');
    std::istringstream values(row);
    return static_cast<bool>(values >> s >> x >> y >> yaw);
  };
  double start_s = 0.0;
  TrackStart result;
  if (!parse(line, start_s, result.x, result.y, result.yaw)) {
    throw std::runtime_error("invalid first raceline row");
  }
  while (start_s < requested_s && std::getline(stream, line)) {
    if (!parse(line, start_s, result.x, result.y, result.yaw)) continue;
  }
  double target_s = start_s;
  double target_x = result.x;
  double target_y = result.y;
  double target_yaw = result.yaw;
  while (target_s < start_s + 3.0 && std::getline(stream, line)) {
    if (!parse(line, target_s, target_x, target_y, target_yaw)) continue;
  }
  const double dx = target_x - result.x;
  const double dy = target_y - result.y;
  result.obstacle_x = std::cos(result.yaw) * dx + std::sin(result.yaw) * dy;
  result.obstacle_y = -std::sin(result.yaw) * dx + std::cos(result.yaw) * dy;
  return result;
}

class PlannerSmoke {
 public:
  explicit PlannerSmoke(TrackStart start) : start_(start) {
    odom_publisher_ =
        node_.advertise<nav_msgs::Odometry>("/v5_jubu_smoke/odom", 10);
    cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
        "/v5_jubu_smoke/roi_cloud", 2);
    health_subscriber_ = node_.subscribe(
        "/v5_jubu_smoke/health", 10, &PlannerSmoke::healthCallback, this);
    state_subscriber_ = node_.subscribe(
        "/v5_jubu_smoke/state", 10, &PlannerSmoke::stateCallback, this);
    diagnostics_subscriber_ = node_.subscribe(
        "/v5_jubu_smoke/diagnostics", 10,
        &PlannerSmoke::diagnosticsCallback, this);
    path_subscriber_ = node_.subscribe(
        "/v5_jubu_smoke/local_reference", 10,
        &PlannerSmoke::pathCallback, this);
  }

  bool run(double timeout_s) {
    const ros::WallTime deadline =
        ros::WallTime::now() + ros::WallDuration(timeout_s);
    ros::Rate rate(30.0);
    int cycle = 0;
    while (ros::ok() && ros::WallTime::now() < deadline) {
      const ros::Time stamp = ros::Time::now();
      publishOdom(stamp);
      // Match the reduced aligned-scan/perception cadence used on the car.
      // A 10 Hz stationary measurement also exercises the obstacle-speed
      // estimator's >=50 ms association path instead of its unknown-speed
      // bootstrap value.
      if ((cycle++ % 3) == 0) publishCloud(stamp);
      ros::spinOnce();
      if (healthy_ && saw_pass_ && saw_local_reference_) return true;
      rate.sleep();
    }
    ROS_ERROR("smoke timeout: healthy=%s pass=%s local_reference=%s diagnostics=%s",
              healthy_ ? "true" : "false", saw_pass_ ? "true" : "false",
              saw_local_reference_ ? "true" : "false",
              latest_diagnostics_.c_str());
    return false;
  }

 private:
  void publishOdom(const ros::Time& stamp) {
    nav_msgs::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.child_frame_id = "body";
    message.pose.pose.position.x = start_.x;
    message.pose.pose.position.y = start_.y;
    message.pose.pose.orientation.z = std::sin(0.5 * start_.yaw);
    message.pose.pose.orientation.w = std::cos(0.5 * start_.yaw);
    message.twist.twist.linear.x = 2.0;
    odom_publisher_.publish(message);
  }

  void publishCloud(const ros::Time& stamp) {
    sensor_msgs::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = "body";
    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(49);
    sensor_msgs::PointCloud2Iterator<float> x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> z(message, "z");
    for (int ix = 0; ix < 7; ++ix) {
      for (int iy = 0; iy < 7; ++iy, ++x, ++y, ++z) {
        *x = static_cast<float>(start_.obstacle_x - 0.05 + ix / 60.0);
        *y = static_cast<float>(start_.obstacle_y - 0.05 + iy / 60.0);
        *z = static_cast<float>(0.08 + 0.01 * ((ix + iy) % 4));
      }
    }
    cloud_publisher_.publish(message);
  }

  void healthCallback(const std_msgs::Bool::ConstPtr& message) {
    healthy_ = healthy_ || message->data;
  }

  void stateCallback(const std_msgs::String::ConstPtr& message) {
    saw_pass_ = saw_pass_ || message->data == "PASS";
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& message) {
    saw_local_reference_ = saw_local_reference_ || !message->poses.empty();
  }

  void diagnosticsCallback(const std_msgs::String::ConstPtr& message) {
    latest_diagnostics_ = message->data;
  }

  ros::NodeHandle node_;
  ros::Publisher odom_publisher_;
  ros::Publisher cloud_publisher_;
  ros::Subscriber health_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber diagnostics_subscriber_;
  ros::Subscriber path_subscriber_;
  TrackStart start_;
  bool healthy_{false};
  bool saw_pass_{false};
  bool saw_local_reference_{false};
  std::string latest_diagnostics_;
};

}  // namespace

TEST(V5JubuPlannerSmoke, RoiCloudToLocalReference) {
  ros::NodeHandle private_node("~");
  std::string track_csv;
  double track_s = 0.0;
  private_node.param("track_csv", track_csv, std::string{});
  private_node.param("track_s", track_s, 0.0);
  ASSERT_FALSE(track_csv.empty()) << "~track_csv is required";
  PlannerSmoke smoke(loadTrackStart(track_csv, track_s));
  EXPECT_TRUE(smoke.run(8.0));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "v5_jubu_planner_smoke_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
