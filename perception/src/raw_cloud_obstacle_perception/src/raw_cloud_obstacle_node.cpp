#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

double yawFromQuaternion(const Eigen::Quaternionf &q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

struct Bounds {
  double x_min = 0.0;
  double x_max = 6.0;
  double y_abs_max = 1.5;
  double z_min = 0.0;
  double z_max = 0.30;
};

struct EgoBox {
  bool enabled = true;
  double x_min = -0.25;
  double x_max = 0.38;
  double y_min = -0.19;
  double y_max = 0.19;
  double z_min = -0.15;
  double z_max = 0.55;
};

struct ClusterParams {
  double voxel_leaf = 0.03;
  double tolerance = 0.12;
  int min_points = 8;
  int max_points = 10000;
};

struct SizeFilter {
  bool enabled = true;
  double max_length = 0.70;
  double max_width = 0.50;
  double max_height = 0.40;
};

struct TrackSample {
  float s = 0.0f;
  float x = 0.0f;
  float y = 0.0f;
  float psi = 0.0f;
  float width_right = 0.0f;
  float width_left = 0.0f;
};

struct TrackCorridorParams {
  bool enabled = true;
  std::string raceline_csv;
  double boundary_inset = 0.08;
  double behind = 1.0;
  double ahead = 8.0;
};

std::vector<std::string> splitCsv(const std::string &line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    if (!field.empty() && field.back() == '\r')
      field.pop_back();
    fields.push_back(field);
  }
  return fields;
}

} // namespace

class RawCloudObstacleNode {
public:
  RawCloudObstacleNode()
      : pnh_("~"), track_xy_(new pcl::PointCloud<pcl::PointXY>),
        track_length_(0.0f), have_odom_(false) {
    pnh_.param<std::string>("cloud_topic", cloud_topic_,
                            "/cloud_registered_body");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/localization/odom");
    pnh_.param<std::string>("roi_topic", roi_topic_,
                            "/raw_cloud_perception/roi_cloud");
    pnh_.param<std::string>("obstacle_topic", obstacle_topic_,
                            "/raw_cloud_perception/obstacle_cloud");
    pnh_.param<std::string>("output_frame", output_frame_, "raw_body_leveled");
    pnh_.param<int>("cloud_queue", cloud_queue_, 1);
    pnh_.param<int>("odom_queue", odom_queue_, 20);
    pnh_.param<double>("max_odom_age", max_odom_age_, 0.25);
    pnh_.param<bool>("publish_tf", publish_tf_, true);

    pnh_.param<double>("pre_crop/x_min", pre_crop_.x_min, 0.0);
    pnh_.param<double>("pre_crop/x_max", pre_crop_.x_max, 6.0);
    pnh_.param<double>("pre_crop/y_abs_max", pre_crop_.y_abs_max, 1.5);
    pnh_.param<double>("pre_crop/z_max", pre_crop_.z_max, 0.30);

    pnh_.param<bool>("ego_box/enabled", ego_.enabled, true);
    pnh_.param<double>("ego_box/x_min", ego_.x_min, -0.25);
    pnh_.param<double>("ego_box/x_max", ego_.x_max, 0.38);
    pnh_.param<double>("ego_box/y_min", ego_.y_min, -0.19);
    pnh_.param<double>("ego_box/y_max", ego_.y_max, 0.19);
    pnh_.param<double>("ego_box/z_min", ego_.z_min, -0.15);
    pnh_.param<double>("ego_box/z_max", ego_.z_max, 0.55);

    pnh_.param<double>("leveled_roi/x_min", leveled_roi_.x_min, 0.0);
    pnh_.param<double>("leveled_roi/x_max", leveled_roi_.x_max, 6.0);
    pnh_.param<double>("leveled_roi/y_abs_max", leveled_roi_.y_abs_max, 1.5);
    pnh_.param<double>("leveled_roi/z_min", leveled_roi_.z_min, 0.0);
    pnh_.param<double>("leveled_roi/z_max", leveled_roi_.z_max, 0.30);

    pnh_.param<bool>("track_corridor/enabled", track_corridor_.enabled, true);
    pnh_.param<std::string>("track_corridor/raceline_csv",
                            track_corridor_.raceline_csv, "");
    pnh_.param<double>("track_corridor/boundary_inset",
                       track_corridor_.boundary_inset, 0.08);
    pnh_.param<double>("track_corridor/behind", track_corridor_.behind, 1.0);
    pnh_.param<double>("track_corridor/ahead", track_corridor_.ahead, 8.0);
    pnh_.param<std::string>("track_corridor/marker_topic",
                            corridor_marker_topic_,
                            "/raw_cloud_perception/corridor_markers");

    pnh_.param<double>("clustering/voxel_leaf", clustering_.voxel_leaf, 0.03);
    pnh_.param<double>("clustering/tolerance", clustering_.tolerance, 0.12);
    pnh_.param<int>("clustering/min_points", clustering_.min_points, 8);
    pnh_.param<int>("clustering/max_points", clustering_.max_points, 10000);

    pnh_.param<bool>("size_filter/enabled", size_filter_.enabled, true);
    pnh_.param<double>("size_filter/max_length", size_filter_.max_length, 0.70);
    pnh_.param<double>("size_filter/max_width", size_filter_.max_width, 0.50);
    pnh_.param<double>("size_filter/max_height", size_filter_.max_height, 0.40);

    if (pre_crop_.x_min >= pre_crop_.x_max || pre_crop_.y_abs_max <= 0.0 ||
        leveled_roi_.x_min >= leveled_roi_.x_max ||
        leveled_roi_.y_abs_max <= 0.0 ||
        leveled_roi_.z_min >= leveled_roi_.z_max ||
        clustering_.voxel_leaf <= 0.0 || clustering_.tolerance <= 0.0 ||
        clustering_.min_points < 1 ||
        clustering_.max_points < clustering_.min_points ||
        track_corridor_.boundary_inset < 0.0 || track_corridor_.behind < 0.0 ||
        track_corridor_.ahead <= 0.0) {
      throw std::runtime_error("invalid raw cloud perception parameters");
    }
    if (track_corridor_.enabled)
      loadTrack(track_corridor_.raceline_csv);

    roi_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(roi_topic_, 1);
    obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(obstacle_topic_, 1);
    corridor_marker_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(corridor_marker_topic_,
                                                       1, true);
    odom_sub_ = nh_.subscribe(odom_topic_, odom_queue_,
                              &RawCloudObstacleNode::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe(cloud_topic_, cloud_queue_,
                               &RawCloudObstacleNode::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("[raw_cloud_perception] cloud=" << cloud_topic_
                                                    << " odom=" << odom_topic_);
    ROS_INFO_STREAM("[raw_cloud_perception] output roi="
                    << roi_topic_ << " obstacle=" << obstacle_topic_);
    ROS_INFO_STREAM("[raw_cloud_perception] leveled ROI x=["
                    << leveled_roi_.x_min << ", " << leveled_roi_.x_max
                    << "] |y|<" << leveled_roi_.y_abs_max << " z=("
                    << leveled_roi_.z_min << ", " << leveled_roi_.z_max << ")");
    if (track_corridor_.enabled) {
      ROS_INFO_STREAM("[raw_cloud_perception] RoboRacer corridor="
                      << track_corridor_.raceline_csv
                      << " samples=" << track_samples_.size()
                      << " inset=" << track_corridor_.boundary_inset
                      << " local_s=[-" << track_corridor_.behind << ", +"
                      << track_corridor_.ahead << "]");
      publishCorridorMarkers();
    }
  }

private:
  void publishCorridorMarkers() {
    visualization_msgs::MarkerArray output;
    output.markers.resize(2);
    for (size_t side = 0; side < output.markers.size(); ++side) {
      auto &marker = output.markers[side];
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.ns = "raw_cloud_perception_green_corridor";
      marker.id = static_cast<int>(side);
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.035;
      marker.color.r = 0.05f;
      marker.color.g = 1.0f;
      marker.color.b = 0.10f;
      marker.color.a = 1.0f;
      marker.text = side == 0 ? "perception_green_left"
                              : "perception_green_right";
      marker.points.reserve(track_samples_.size() + 1);
    }
    for (const auto &sample : track_samples_) {
      const double nx = -std::sin(sample.psi);
      const double ny = std::cos(sample.psi);
      const double left =
          std::max(0.0, static_cast<double>(sample.width_left) -
                            track_corridor_.boundary_inset);
      const double right =
          std::max(0.0, static_cast<double>(sample.width_right) -
                            track_corridor_.boundary_inset);
      geometry_msgs::Point left_point;
      left_point.x = sample.x + nx * left;
      left_point.y = sample.y + ny * left;
      left_point.z = 0.055;
      output.markers[0].points.push_back(left_point);
      geometry_msgs::Point right_point;
      right_point.x = sample.x - nx * right;
      right_point.y = sample.y - ny * right;
      right_point.z = 0.055;
      output.markers[1].points.push_back(right_point);
    }
    for (auto &marker : output.markers) {
      if (!marker.points.empty())
        marker.points.push_back(marker.points.front());
    }
    corridor_marker_pub_.publish(output);
  }

  void loadTrack(const std::string &path) {
    if (path.empty())
      throw std::runtime_error(
          "track corridor enabled but raceline_csv is empty");
    std::ifstream input(path);
    if (!input)
      throw std::runtime_error("cannot open RoboRacer raceline: " + path);

    std::string line;
    if (!std::getline(input, line))
      throw std::runtime_error("RoboRacer raceline is empty: " + path);
    const std::vector<std::string> header = splitCsv(line);
    const auto column = [&header](const std::string &name) {
      const auto it = std::find(header.begin(), header.end(), name);
      if (it == header.end())
        throw std::runtime_error("raceline is missing column: " + name);
      return static_cast<size_t>(std::distance(header.begin(), it));
    };
    const size_t s_col = column("s_m");
    const size_t x_col = column("x_m");
    const size_t y_col = column("y_m");
    const size_t psi_col = column("psi_rad");
    const size_t right_col = column("w_tr_right_m");
    const size_t left_col = column("w_tr_left_m");
    const size_t required_col =
        std::max({s_col, x_col, y_col, psi_col, right_col, left_col});

    track_samples_.clear();
    track_xy_->clear();
    while (std::getline(input, line)) {
      if (line.empty())
        continue;
      const std::vector<std::string> fields = splitCsv(line);
      if (fields.size() <= required_col)
        throw std::runtime_error("short row in RoboRacer raceline");
      TrackSample sample;
      try {
        sample.s = std::stof(fields[s_col]);
        sample.x = std::stof(fields[x_col]);
        sample.y = std::stof(fields[y_col]);
        sample.psi = std::stof(fields[psi_col]);
        sample.width_right = std::stof(fields[right_col]);
        sample.width_left = std::stof(fields[left_col]);
      } catch (const std::exception &) {
        throw std::runtime_error("invalid numeric row in RoboRacer raceline");
      }
      if (!track_samples_.empty() && sample.s <= track_samples_.back().s)
        throw std::runtime_error("RoboRacer raceline s_m is not increasing");
      if (sample.width_right <= 0.0f || sample.width_left <= 0.0f)
        throw std::runtime_error(
            "RoboRacer raceline contains non-positive width");
      track_samples_.push_back(sample);
      pcl::PointXY point;
      point.x = sample.x;
      point.y = sample.y;
      track_xy_->push_back(point);
    }
    if (track_samples_.size() < 8)
      throw std::runtime_error("RoboRacer raceline has too few samples");
    const TrackSample &first = track_samples_.front();
    const TrackSample &last = track_samples_.back();
    track_length_ = last.s + std::hypot(first.x - last.x, first.y - last.y);
    if (!(track_length_ > last.s))
      throw std::runtime_error("invalid RoboRacer raceline loop length");
    track_xy_->width = track_xy_->size();
    track_xy_->height = 1;
    track_tree_.setInputCloud(track_xy_);
  }

  int nearestTrackIndex(float x, float y) {
    pcl::PointXY query;
    query.x = x;
    query.y = y;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (track_tree_.nearestKSearch(query, 1, indices, distances) != 1)
      return -1;
    return indices.front();
  }

  float wrappedTrackDelta(float s, float reference) const {
    float delta = std::fmod(s - reference, track_length_);
    if (delta > 0.5f * track_length_)
      delta -= track_length_;
    if (delta < -0.5f * track_length_)
      delta += track_length_;
    return delta;
  }

  bool insideCurrentTrackCorridor(float map_x, float map_y, float ego_s) {
    if (!track_corridor_.enabled)
      return true;
    const int index = nearestTrackIndex(map_x, map_y);
    if (index < 0)
      return false;
    const TrackSample &sample = track_samples_[static_cast<size_t>(index)];
    const float ds = wrappedTrackDelta(sample.s, ego_s);
    if (ds < -static_cast<float>(track_corridor_.behind) ||
        ds > static_cast<float>(track_corridor_.ahead)) {
      return false;
    }

    const float dx = map_x - sample.x;
    const float dy = map_y - sample.y;
    const float lateral =
        -std::sin(sample.psi) * dx + std::cos(sample.psi) * dy;
    const float left =
        std::max(0.0f, sample.width_left -
                           static_cast<float>(track_corridor_.boundary_inset));
    const float right =
        std::max(0.0f, sample.width_right -
                           static_cast<float>(track_corridor_.boundary_inset));
    return lateral >= -right && lateral <= left;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    have_odom_ = true;
  }

  bool latestLevelRotation(const ros::Time &cloud_stamp,
                           Eigen::Matrix3f &r_level_body,
                           Eigen::Matrix3f &r_map_level,
                           Eigen::Vector3f &map_translation,
                           Eigen::Quaternionf &q_body_level,
                           ros::Time &odom_stamp) {
    nav_msgs::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!have_odom_)
        return false;
      odom = latest_odom_;
    }
    odom_stamp = odom.header.stamp;
    const double age = std::fabs((cloud_stamp - odom_stamp).toSec());
    if (max_odom_age_ > 0.0 && age > max_odom_age_) {
      ROS_WARN_THROTTLE(
          2.0,
          "[raw_cloud_perception] newest odom is %.3f s from cloud; skipping",
          age);
      return false;
    }

    const auto &o = odom.pose.pose.orientation;
    Eigen::Quaternionf q_map_body(
        static_cast<float>(o.w), static_cast<float>(o.x),
        static_cast<float>(o.y), static_cast<float>(o.z));
    if (q_map_body.norm() < 1.0e-6f)
      return false;
    q_map_body.normalize();
    const float yaw = static_cast<float>(yawFromQuaternion(q_map_body));
    const Eigen::Matrix3f r_map_body = q_map_body.toRotationMatrix();
    r_map_level =
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const auto &position = odom.pose.pose.position;
    map_translation = Eigen::Vector3f(static_cast<float>(position.x),
                                      static_cast<float>(position.y),
                                      static_cast<float>(position.z));

    // body point -> gravity-leveled body point. Yaw is retained; roll/pitch are
    // removed.
    r_level_body = r_map_level.transpose() * r_map_body;
    // TF parent body -> child leveled is the inverse rotation.
    q_body_level = Eigen::Quaternionf(r_level_body.transpose());
    q_body_level.normalize();
    return true;
  }

  bool insideEgoBox(const PointT &p) const {
    return ego_.enabled && p.x >= ego_.x_min && p.x <= ego_.x_max &&
           p.y >= ego_.y_min && p.y <= ego_.y_max && p.z >= ego_.z_min &&
           p.z <= ego_.z_max;
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    Eigen::Matrix3f r_level_body;
    Eigen::Matrix3f r_map_level;
    Eigen::Vector3f map_translation;
    Eigen::Quaternionf q_body_level;
    ros::Time odom_stamp;
    if (!latestLevelRotation(msg->header.stamp, r_level_body, r_map_level,
                             map_translation, q_body_level, odom_stamp)) {
      ROS_WARN_THROTTLE(
          2.0,
          "[raw_cloud_perception] waiting for fresh localization odometry");
      return;
    }

    CloudT raw;
    pcl::fromROSMsg(*msg, raw);
    CloudT::Ptr roi(new CloudT);
    roi->reserve(raw.size() / 4);
    size_t pre_crop_count = 0;
    size_t above_ground_count = 0;
    float ego_s = 0.0f;
    if (track_corridor_.enabled) {
      const int ego_index =
          nearestTrackIndex(map_translation.x(), map_translation.y());
      if (ego_index < 0) {
        ROS_WARN_THROTTLE(
            2.0,
            "[raw_cloud_perception] cannot project ego onto RoboRacer raceline");
        return;
      }
      ego_s = track_samples_[static_cast<size_t>(ego_index)].s;
    }

    for (const auto &p : raw) {
      if (!pcl::isFinite(p))
        continue;

      // Required first-stage crop in the unmodified body-frame cloud.
      if (p.x < pre_crop_.x_min || p.x > pre_crop_.x_max ||
          std::fabs(p.y) > pre_crop_.y_abs_max || p.z >= pre_crop_.z_max) {
        continue;
      }
      ++pre_crop_count;
      if (insideEgoBox(p))
        continue;

      const Eigen::Vector3f corrected =
          r_level_body * Eigen::Vector3f(p.x, p.y, p.z);
      if (corrected.x() < leveled_roi_.x_min ||
          corrected.x() > leveled_roi_.x_max ||
          std::fabs(corrected.y()) > leveled_roi_.y_abs_max ||
          corrected.z() <= leveled_roi_.z_min ||
          corrected.z() >= leveled_roi_.z_max) {
        continue;
      }
      ++above_ground_count;
      const Eigen::Vector3f point_map =
          map_translation + r_map_level * corrected;
      if (!insideCurrentTrackCorridor(point_map.x(), point_map.y(), ego_s))
        continue;
      roi->push_back(PointT(corrected.x(), corrected.y(), corrected.z()));
    }
    roi->width = roi->size();
    roi->height = 1;
    roi->is_dense = false;

    publishCloud(*roi, msg->header.stamp, roi_pub_);

    CloudT obstacles;
    int accepted_clusters = 0;
    if (roi->size() >= static_cast<size_t>(clustering_.min_points)) {
      CloudT::Ptr downsampled(new CloudT);
      pcl::VoxelGrid<PointT> voxel;
      voxel.setInputCloud(roi);
      const float leaf = static_cast<float>(clustering_.voxel_leaf);
      voxel.setLeafSize(leaf, leaf, leaf);
      voxel.filter(*downsampled);

      if (downsampled->size() >= static_cast<size_t>(clustering_.min_points)) {
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(downsampled);
        std::vector<pcl::PointIndices> indices;
        pcl::EuclideanClusterExtraction<PointT> extractor;
        extractor.setClusterTolerance(clustering_.tolerance);
        extractor.setMinClusterSize(clustering_.min_points);
        extractor.setMaxClusterSize(clustering_.max_points);
        extractor.setSearchMethod(tree);
        extractor.setInputCloud(downsampled);
        extractor.extract(indices);

        for (const auto &cluster_indices : indices) {
          CloudT cluster;
          cluster.reserve(cluster_indices.indices.size());
          for (const int index : cluster_indices.indices)
            cluster.push_back((*downsampled)[index]);
          if (!clusterAccepted(cluster))
            continue;
          obstacles += cluster;
          ++accepted_clusters;
        }
      }
    }
    obstacles.width = obstacles.size();
    obstacles.height = 1;
    obstacles.is_dense = false;
    publishCloud(obstacles, msg->header.stamp, obstacle_pub_);
    broadcastLeveledFrame(msg->header.frame_id, msg->header.stamp,
                          q_body_level);

    ROS_INFO_THROTTLE(2.0,
                      "[raw_cloud_perception] raw=%zu pre_crop=%zu "
                      "above_ground=%zu corridor=%zu "
                      "obstacle=%zu clusters=%d "
                      "odom_dt=%.3f",
                      raw.size(), pre_crop_count, above_ground_count,
                      roi->size(), obstacles.size(), accepted_clusters,
                      std::fabs((msg->header.stamp - odom_stamp).toSec()));
  }

  bool clusterAccepted(const CloudT &cluster) const {
    if (!size_filter_.enabled || cluster.size() < 3)
      return true;

    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    float z_min = std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();
    for (const auto &p : cluster) {
      mean += Eigen::Vector2f(p.x, p.y);
      z_min = std::min(z_min, p.z);
      z_max = std::max(z_max, p.z);
    }
    mean /= static_cast<float>(cluster.size());

    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto &p : cluster) {
      const Eigen::Vector2f delta(p.x - mean.x(), p.y - mean.y());
      covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(covariance);
    const Eigen::Vector2f minor = solver.eigenvectors().col(0);
    const Eigen::Vector2f major = solver.eigenvectors().col(1);
    float major_min = std::numeric_limits<float>::max();
    float major_max = -std::numeric_limits<float>::max();
    float minor_min = std::numeric_limits<float>::max();
    float minor_max = -std::numeric_limits<float>::max();
    for (const auto &p : cluster) {
      const Eigen::Vector2f xy(p.x, p.y);
      const float a = xy.dot(major);
      const float b = xy.dot(minor);
      major_min = std::min(major_min, a);
      major_max = std::max(major_max, a);
      minor_min = std::min(minor_min, b);
      minor_max = std::max(minor_max, b);
    }
    const float length = major_max - major_min;
    const float width = minor_max - minor_min;
    const float height = z_max - z_min;
    return length <= size_filter_.max_length &&
           width <= size_filter_.max_width && height <= size_filter_.max_height;
  }

  void publishCloud(const CloudT &cloud, const ros::Time &stamp,
                    const ros::Publisher &publisher) {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.stamp = stamp;
    output.header.frame_id = output_frame_;
    publisher.publish(output);
  }

  void broadcastLeveledFrame(const std::string &parent, const ros::Time &stamp,
                             const Eigen::Quaternionf &q_body_level) {
    if (!publish_tf_ || parent.empty() || parent == output_frame_)
      return;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent;
    transform.child_frame_id = output_frame_;
    transform.transform.rotation.w = q_body_level.w();
    transform.transform.rotation.x = q_body_level.x();
    transform.transform.rotation.y = q_body_level.y();
    transform.transform.rotation.z = q_body_level.z();
    tf_broadcaster_.sendTransform(transform);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher roi_pub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher corridor_marker_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string roi_topic_;
  std::string obstacle_topic_;
  std::string corridor_marker_topic_;
  std::string output_frame_;
  int cloud_queue_;
  int odom_queue_;
  double max_odom_age_;
  bool publish_tf_;
  Bounds pre_crop_;
  Bounds leveled_roi_;
  EgoBox ego_;
  ClusterParams clustering_;
  SizeFilter size_filter_;
  TrackCorridorParams track_corridor_;
  std::vector<TrackSample> track_samples_;
  pcl::PointCloud<pcl::PointXY>::Ptr track_xy_;
  pcl::KdTreeFLANN<pcl::PointXY> track_tree_;
  float track_length_;

  std::mutex odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  bool have_odom_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "raw_cloud_obstacle_perception");
  try {
    RawCloudObstacleNode node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception &error) {
    ROS_FATAL("[raw_cloud_perception] startup failed: %s", error.what());
    return 1;
  }
  return 0;
}
