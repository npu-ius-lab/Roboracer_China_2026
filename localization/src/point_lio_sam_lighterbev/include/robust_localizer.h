#pragma once

#include <atomic>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nano_gicp/nano_gicp.hpp>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>

#include "Lighterbev.h"
#include "descriptor_io.hpp"
#include "point_lio_sam_lighterbev/LocalizationStatus.h"
#include "pose_pcd.hpp"
#include "BEVProjector.hpp"
#include "utilities.hpp"

class RobustMapLocalizer
{
public:
    explicit RobustMapLocalizer(const ros::NodeHandle &private_nh);
    ~RobustMapLocalizer() = default;

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry, sensor_msgs::PointCloud2>;
    using NanoGICP = nano_gicp::NanoGICP<PointType, PointType>;

    enum class State : uint8_t
    {
        WAITING_FOR_MAP = 0,
        RELOCALIZING = 1,
        TRACKING = 2,
        DEGRADED = 3
    };

    struct MapKeyframe
    {
        double timestamp = 0.0;
        Eigen::Matrix4d map_from_body = Eigen::Matrix4d::Identity();
        pcl::PointCloud<PointType>::Ptr points_body{
            new pcl::PointCloud<PointType>()};
        BEVFrame bev;
    };

    struct LiveFrame
    {
        uint64_t sequence = 0;
        ros::Time stamp;
        Eigen::Matrix4d odom_from_body = Eigen::Matrix4d::Identity();
        pcl::PointCloud<PointType>::Ptr points_body{
            new pcl::PointCloud<PointType>()};
    };

    struct StampedOdom
    {
        ros::Time stamp;
        Eigen::Matrix4d odom_from_body = Eigen::Matrix4d::Identity();
    };

    struct RecoveryFrame
    {
        uint64_t sequence = 0;
        ros::Time stamp;
        pcl::PointCloud<PointType>::Ptr points_body{
            new pcl::PointCloud<PointType>()};
    };

    struct PendingInitialPose
    {
        uint64_t sequence = 0;
        ros::WallTime received_wall_time;
        geometry_msgs::PoseWithCovarianceStamped message;
        Eigen::Matrix4d map_from_vehicle_base = Eigen::Matrix4d::Identity();
    };

    struct RegistrationResult
    {
        bool valid = false;
        bool converged = false;
        Eigen::Matrix4d map_from_body = Eigen::Matrix4d::Identity();
        double score = std::numeric_limits<double>::infinity();
        double correspondence_ratio = 0.0;
        double mean_residual = std::numeric_limits<double>::infinity();
        double innovation_translation = 0.0;
        double innovation_rotation_deg = 0.0;
        int matched_keyframe = -1;
        int lighterbev_inliers = 0;
        float descriptor_distance = std::numeric_limits<float>::infinity();
        std::string reason;
    };

    struct StatusSnapshot
    {
        double score = std::numeric_limits<double>::infinity();
        double correspondence_ratio = 0.0;
        double mean_residual = std::numeric_limits<double>::infinity();
        double correction_translation = 0.0;
        double correction_rotation_deg = 0.0;
        int matched_keyframe = -1;
        std::string reason = "initializing";
    };

    void readParameters();
    bool loadMapBundle();
    void resolveDescriptorCachePath();
    bool loadPoses(const std::string &path,
                   std::vector<std::pair<double, Eigen::Matrix4d>> &poses) const;
    bool loadDescriptorDatabase();
    bool buildDescriptorDatabase();

    void odomCloudCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                           const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void controlOdomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void recoveryCloudCallback(
        const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void initialPoseCallback(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg);
    bool takePendingInitialPose(PendingInitialPose &request);
    bool processInitialPoseRequest(const PendingInitialPose &request,
                                   const LiveFrame &latest);
    bool estimateRecentOdomMotion(double &linear_speed_mps,
                                  double &angular_speed_degps);
    int nearestMapKeyframe2d(double x, double y) const;
    void publishRefinedInitialPose(
        const PendingInitialPose &request,
        const Eigen::Matrix4d &map_from_body,
        const ros::Time &stamp);
    bool getLatestRecoverySource(
        pcl::PointCloud<PointType>::Ptr &points,
        ros::Time &raw_stamp,
        ros::Time &matched_odom_stamp,
        Eigen::Matrix4d &matched_odom_from_body,
        double &sync_dt);
    void backendTimerCallback(const ros::TimerEvent &event);
    bool forceRelocalizationService(std_srvs::Trigger::Request &request,
                                    std_srvs::Trigger::Response &response);

    pcl::PointCloud<PointType>::Ptr buildAccumulatedSource(
        const std::vector<LiveFrame> &frames,
        const LiveFrame &latest) const;
    pcl::PointCloud<PointType>::Ptr buildMapSubmap(
        const Eigen::Vector3d &center, int center_keyframe,
        bool force_center_keyframe) const;
    std::vector<int> selectKeyframesAround(
        const Eigen::Vector3d &center) const;
    const pcl::PointCloud<PointType>::Ptr &getKeyframeWorldCloud(int index);
    pcl::PointCloud<PointType>::Ptr concatKeyframeWorldClouds(
        const std::vector<int> &indices);
    bool refreshContinuousTarget(const Eigen::Matrix4d &predicted_map_from_body);
    RegistrationResult registerCloud(
        NanoGICP &registration,
        const pcl::PointCloud<PointType>::Ptr &source,
        const pcl::PointCloud<PointType>::Ptr &target,
        const Eigen::Matrix4d &initial_map_from_body,
        double max_score, double min_correspondence_ratio,
        bool target_is_prepared) const;
    RegistrationResult attemptRelocalization(
        const LiveFrame &latest,
        const pcl::PointCloud<PointType>::Ptr &source);
    void processTracking(const LiveFrame &latest,
                         const pcl::PointCloud<PointType>::Ptr &source);
    bool observeRelocalizationHypothesis(
        const RegistrationResult &result,
        const Eigen::Matrix4d &odom_from_body,
        bool recovery);
    void applyMapToOdom(const Eigen::Matrix4d &candidate,
                        bool hard_reset, const std::string &reason);

    void publishMapToOdom(const ros::Time &stamp);
    void publishAlignedScan(const LiveFrame &frame);
    void publishStatus(const ros::Time &stamp);
    void publishPriorMap();
    void publishRelocalizationMatch(const cv::Mat &image,
                                    const ros::Time &stamp);
    bool requestPointLioLocalMapReset();

    static Eigen::Matrix4d odometryToMatrix(const nav_msgs::Odometry &message);
    static nav_msgs::Odometry matrixToOdometry(
        const Eigen::Matrix4d &pose, const ros::Time &stamp,
        const std::string &frame, const std::string &child);
    static double rotationAngleDeg(const Eigen::Matrix4d &transform);
    static Eigen::Matrix4d interpolateTransform(
        const Eigen::Matrix4d &from, const Eigen::Matrix4d &to,
        double ratio);
    static pcl::PointCloud<PointType>::Ptr downsample(
        const pcl::PointCloud<PointType>::Ptr &cloud, double voxel_size);

    ros::NodeHandle nh_;
    std::string package_path_;
    std::string map_dir_;
    std::string model_path_;
    std::string descriptor_cache_path_;
    std::string lighterbev_device_ = "cpu";
    std::string odom_topic_ = "/aft_mapped_to_init";
    std::string cloud_topic_ = "/cloud_registered";
    std::string tracking_cloud_topic_ = "/cloud_registered";
    bool tracking_cloud_is_body_frame_ = false;
    std::string recovery_cloud_topic_ =
        "/pointlio/recovery_cloud_body_raw";
    double recovery_cloud_max_age_s_ = 0.30;
    int recovery_min_points_ = 100;
    double recovery_odom_sync_tolerance_s_ = 0.05;
    bool use_raw_recovery_cloud_ = true;
    bool initialpose_enabled_ = true;
    std::string initialpose_topic_ = "/initialpose";
    std::string initialpose_refined_topic_ =
        "/localization/initialpose_refined";
    bool initialpose_reference_is_vehicle_base_ = true;
    bool initialpose_require_stationary_ = true;
    double initialpose_stationary_max_speed_mps_ = 0.25;
    double initialpose_stationary_max_angular_speed_degps_ = 15.0;
    double initialpose_request_timeout_s_ = 5.0;
    int initialpose_min_points_ = 100;
    int initialpose_gicp_max_iterations_ = 40;
    double initialpose_gicp_max_correspondence_distance_ = 1.8;
    double initialpose_max_score_ = 0.50;
    double initialpose_min_correspondence_ratio_ = 0.20;
    double initialpose_max_translation_correction_m_ = 3.0;
    double initialpose_max_rotation_correction_deg_ = 45.0;
    std::string map_frame_ = "map";
    std::string odom_frame_ = "odom";
    std::string body_frame_ = "body";
    std::string vehicle_odom_topic_ = "/localization/vehicle_odom";
    std::string vehicle_base_frame_ = "localization_base_link";
    // Position of the Point-LIO body (MID360/IMU) origin, expressed from
    // the vehicle base origin. The measured sensor is 135 mm forward.
    Eigen::Vector3d vehicle_base_to_body_{0.135, 0.0, 0.0};

    double frontend_max_hz_ = 50.0;
    double control_output_hz_ = 50.0;
    double frontend_max_speed_mps_ = 15.0;
    double frontend_max_angular_speed_degps_ = 180.0;
    double frontend_max_frame_jump_m_ = 2.0;
    int frontend_bad_frame_limit_ = 3;
    double backend_hz_ = 10.0;
    int accumulation_frames_ = 3;
    double input_voxel_size_ = 0.0;
    double registration_voxel_size_ = 0.20;
    double local_map_radius_ = 15.0;
    double local_map_refresh_distance_ = 2.0;
    int local_map_max_keyframes_ = 40;
    bool continuous_target_incremental_ = false;
    int continuous_target_compaction_updates_ = 8;
    double continuous_target_compaction_stale_ratio_ = 0.35;
    int relocalization_submap_half_width_ = 5;

    int gicp_threads_ = 2;
    int gicp_covariance_threads_ = 4;
    int gicp_correspondences_ = 15;
    int gicp_max_iterations_ = 20;
    double gicp_max_correspondence_distance_ = 1.2;
    double continuous_max_score_ = 0.35;
    double continuous_min_correspondence_ratio_ = 0.25;
    double continuous_max_translation_update_ = 1.0;
    double continuous_max_rotation_update_deg_ = 10.0;
    double correction_smoothing_alpha_ = 0.20;
    int tracking_failure_limit_ = 10;

    // Runtime pose-jump diagnostics. Startup alignment and the first corrected
    // pose establish the baseline and are deliberately excluded.
    double pose_jump_log_translation_m_ = 0.20;
    double pose_jump_log_rotation_deg_ = 8.0;

    double bev_range_ = 5.0;
    double bev_resolution_ = 0.05;
    bool bev_downsample_enable_ = true;
    double bev_voxel_size_ = 0.1;
    bool bev_z_max_filter_enable_ = true;
    double bev_z_max_ = 2.0;
    int bev_threads_ = 2;
    int relocalization_top_k_ = 5;
    float descriptor_max_distance_ = 0.80f;
    int lighterbev_min_inliers_ = 20;
    double relocalization_max_score_ = 0.50;
    double relocalization_min_correspondence_ratio_ = 0.20;
    int relocalization_required_confirmations_ = 1;
    double hypothesis_max_translation_delta_ = 1.0;
    double hypothesis_max_rotation_delta_deg_ = 5.0;
    double relocalization_retry_sec_ = 0.5;
    double outside_map_relocalization_retry_sec_ = 10.0;
    double global_probe_interval_sec_ = 30.0;
    double severe_global_translation_error_ = 3.0;
    double severe_global_rotation_error_deg_ = 15.0;
    double recovery_settle_sec_ = 1.0;
    bool reset_pointlio_local_map_on_recovery_ = true;
    bool lighterbev_viz_enable_ = true;
    std::string lighterbev_viz_topic_ =
        "/lio/relocalization/lighterbev_match";

    std::vector<MapKeyframe> map_keyframes_;
    pcl::PointCloud<PointType>::Ptr prior_map_{new pcl::PointCloud<PointType>()};
    std::unique_ptr<LighterBEVManager> bev_manager_;

    std::unique_ptr<NanoGICP> continuous_gicp_;
    pcl::PointCloud<PointType>::Ptr continuous_target_{
        new pcl::PointCloud<PointType>()};
    pcl::KdTreeFLANN<PointType> continuous_target_tree_;
    Eigen::Vector3d continuous_target_center_ =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    std::vector<int> continuous_target_keyframes_;
    std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_world_cache_;
    double last_target_update_ms_ = 0.0;
    double last_target_delete_ms_ = 0.0;
    int continuous_target_updates_since_build_ = 0;

    std::mutex frame_mutex_;
    std::deque<LiveFrame> live_frames_;
    uint64_t next_sequence_ = 1;
    uint64_t last_processed_sequence_ = 0;
    double last_accepted_input_stamp_ = -1.0;
    std::atomic<bool> frontend_healthy_{true};
    int frontend_bad_count_ = 0;

    std::mutex recovery_frame_mutex_;
    RecoveryFrame latest_recovery_frame_;
    bool has_recovery_frame_ = false;
    uint64_t next_recovery_sequence_ = 0;
    std::mutex recent_odom_mutex_;
    std::deque<StampedOdom> recent_odom_;

    std::mutex initialpose_mutex_;
    PendingInitialPose pending_initialpose_;
    bool has_pending_initialpose_ = false;
    uint64_t next_initialpose_sequence_ = 1;

    std::mutex transform_mutex_;
    Eigen::Matrix4d map_from_odom_ = Eigen::Matrix4d::Identity();
    std::atomic<bool> has_map_from_odom_{false};
    uint64_t map_correction_sequence_ = 0;
    std::string last_map_correction_reason_ = "initializing";
    bool last_map_correction_hard_reset_ = false;
    double last_map_correction_translation_ = 0.0;
    double last_map_correction_rotation_deg_ = 0.0;

    std::mutex control_odom_mutex_;
    double last_control_output_stamp_ = -1.0;
    bool pose_jump_baseline_valid_ = false;
    Eigen::Matrix4d previous_raw_odom_from_body_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d previous_corrected_map_from_body_ = Eigen::Matrix4d::Identity();
    double previous_pose_jump_stamp_ = -1.0;
    uint64_t previous_map_correction_sequence_ = 0;

    std::mutex processing_mutex_;
    State state_ = State::WAITING_FOR_MAP;
    bool map_loaded_ = false;
    int consecutive_tracking_failures_ = 0;
    bool recovery_outside_map_ = false;
    int relocalization_confirmations_ = 0;
    bool pending_hypothesis_valid_ = false;
    Eigen::Matrix4d pending_map_from_odom_ = Eigen::Matrix4d::Identity();
    double last_relocalization_attempt_stamp_ = -1.0;
    double last_global_probe_stamp_ = -1.0;
    double last_recovery_stamp_ = -1.0;
    bool global_probe_active_ = false;
    std::atomic<bool> force_relocalization_{false};
    StatusSnapshot status_;

    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
    ros::Subscriber control_odom_sub_;
    ros::Subscriber recovery_cloud_sub_;
    ros::Subscriber initialpose_sub_;
    ros::Publisher corrected_odom_pub_;
    ros::Publisher vehicle_odom_pub_;
    ros::Publisher corrected_cloud_pub_;
    ros::Publisher prior_map_pub_;
    ros::Publisher status_pub_;
    ros::Publisher relocalization_match_pub_;
    ros::Publisher initialpose_refined_pub_;
    ros::ServiceServer relocalize_service_;
    ros::ServiceClient pointlio_reset_client_;
    ros::Timer backend_timer_;
    tf::TransformBroadcaster tf_broadcaster_;
};
