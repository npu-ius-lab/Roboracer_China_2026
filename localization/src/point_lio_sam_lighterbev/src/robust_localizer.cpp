#include "robust_localizer.h"
#include "path_utils.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <ros/package.h>

namespace fs = boost::filesystem;

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool finiteTransform(const Eigen::Matrix4d &transform)
{
    return transform.allFinite() &&
           std::abs(transform(3, 3) - 1.0) < 1.0e-6;
}

void configureRegistration(nano_gicp::NanoGICP<PointType, PointType> &registration,
                           int threads, int correspondences, int iterations,
                           double max_correspondence_distance,
                           int covariance_threads = 0)
{
    registration.setNumThreads(std::max(1, threads));
    registration.setCovarianceThreads(covariance_threads);
    registration.setCorrespondenceRandomness(std::max(5, correspondences));
    registration.setMaximumIterations(std::max(1, iterations));
    registration.setMaxCorrespondenceDistance(
        std::max(0.1, max_correspondence_distance));
    registration.setTransformationEpsilon(0.001);
    registration.setEuclideanFitnessEpsilon(0.001);
    registration.setRANSACIterations(0);
}

cv::Mat bevWithColor(const cv::Mat &image, const cv::Scalar &color)
{
    if (image.empty())
        return cv::Mat();
    cv::Mat gray;
    if (image.channels() == 1)
        gray = image;
    else
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat output(gray.size(), CV_8UC3, cv::Scalar::all(0));
    for (int row = 0; row < gray.rows; ++row)
    {
        const uchar *source = gray.ptr<uchar>(row);
        cv::Vec3b *target = output.ptr<cv::Vec3b>(row);
        for (int column = 0; column < gray.cols; ++column)
        {
            const double scale = static_cast<double>(source[column]) / 255.0;
            target[column] = cv::Vec3b(
                static_cast<uchar>(std::min(255.0, color[0] * scale)),
                static_cast<uchar>(std::min(255.0, color[1] * scale)),
                static_cast<uchar>(std::min(255.0, color[2] * scale)));
        }
    }
    return output;
}

cv::Mat toBgrImage(const cv::Mat &image)
{
    if (image.empty())
        return cv::Mat();
    if (image.channels() == 3)
        return image.clone();
    cv::Mat output;
    cv::cvtColor(image, output, cv::COLOR_GRAY2BGR);
    return output;
}

cv::Mat overlayBevImages(const cv::Mat &first, const cv::Mat &second)
{
    if (first.empty())
        return second.clone();
    if (second.empty())
        return first.clone();
    cv::Mat output;
    cv::add(first, second, output);
    return output;
}

void placeImageInPanel(const cv::Mat &image, cv::Mat &canvas,
                       const cv::Rect &panel)
{
    if (image.empty())
        return;
    const cv::Mat bgr = toBgrImage(image);
    const double scale = std::min(
        static_cast<double>(panel.width) / bgr.cols,
        static_cast<double>(panel.height) / bgr.rows);
    cv::Mat resized;
    cv::resize(bgr, resized,
               cv::Size(std::max(1, static_cast<int>(std::round(bgr.cols * scale))),
                        std::max(1, static_cast<int>(std::round(bgr.rows * scale)))),
               0.0, 0.0, cv::INTER_AREA);
    const int x = panel.x + (panel.width - resized.cols) / 2;
    const int y = panel.y + (panel.height - resized.rows) / 2;
    resized.copyTo(canvas(cv::Rect(x, y, resized.cols, resized.rows)));
}

void drawPanelLabel(cv::Mat &canvas, const std::string &text,
                    const cv::Point &origin)
{
    cv::putText(canvas, text, origin + cv::Point(2, 2),
                cv::FONT_HERSHEY_SIMPLEX, 0.85, cv::Scalar::all(0), 3,
                cv::LINE_AA);
    cv::putText(canvas, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.85,
                cv::Scalar(235, 235, 235), 2, cv::LINE_AA);
}

cv::Mat makeRelocalizationMatchImage(const BEVFrame &query,
                                     const BEVFrame &target,
                                     int target_id, int inliers)
{
    constexpr int cell_width = 320;
    constexpr int cell_height = 320;
    cv::Mat canvas(cell_height * 2, cell_width * 3, CV_8UC3,
                   cv::Scalar::all(0));

    const cv::Mat query_color =
        bevWithColor(query.bev_img, cv::Scalar(255, 255, 255));
    const cv::Mat target_color =
        bevWithColor(target.bev_img, cv::Scalar(0, 45, 255));
    const cv::Mat initial_overlap = query.initial_overlap.empty() ?
        overlayBevImages(target_color, query_color) :
        toBgrImage(query.initial_overlap);
    const cv::Mat feature_matches = query.img_matches.empty() ?
        initial_overlap : toBgrImage(query.img_matches);
    const cv::Mat registered_overlap = query.registered_overlap.empty() ?
        initial_overlap : toBgrImage(query.registered_overlap);

    placeImageInPanel(query_color, canvas,
                      cv::Rect(0, 0, cell_width, cell_height));
    placeImageInPanel(target_color, canvas,
                      cv::Rect(cell_width, 0, cell_width, cell_height));
    placeImageInPanel(initial_overlap, canvas,
                      cv::Rect(cell_width * 2, 0, cell_width, cell_height));
    placeImageInPanel(feature_matches, canvas,
                      cv::Rect(0, cell_height, cell_width * 2, cell_height));
    placeImageInPanel(registered_overlap, canvas,
                      cv::Rect(cell_width * 2, cell_height,
                               cell_width, cell_height));

    drawPanelLabel(canvas, "Query", cv::Point(18, 42));
    drawPanelLabel(canvas, "Top1", cv::Point(cell_width + 18, 42));
    drawPanelLabel(canvas, "Initial_Overlap",
                   cv::Point(cell_width * 2 + 18, 42));
    drawPanelLabel(canvas, "Feature matching",
                   cv::Point(18, cell_height + 42));
    drawPanelLabel(canvas, "Overlap aft.",
                   cv::Point(cell_width * 2 + 18, cell_height + 42));
    drawPanelLabel(canvas, "Registration",
                   cv::Point(cell_width * 2 + 18, cell_height + 82));

    std::ostringstream summary;
    summary << "id=" << target_id << " inliers=" << inliers;
    cv::putText(canvas, summary.str(),
                cv::Point(cell_width * 2 + 18, cell_height * 2 - 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(190, 255, 190), 1, cv::LINE_AA);
    for (int x = cell_width; x < canvas.cols; x += cell_width)
        cv::line(canvas, cv::Point(x, 0), cv::Point(x, canvas.rows - 1),
                 cv::Scalar(25, 25, 25), 1);
    cv::line(canvas, cv::Point(0, cell_height),
             cv::Point(canvas.cols - 1, cell_height),
             cv::Scalar(25, 25, 25), 1);
    return canvas;
}

}  // namespace

RobustMapLocalizer::RobustMapLocalizer(const ros::NodeHandle &private_nh)
    : nh_(private_nh)
{
    package_path_ = ros::package::getPath("point_lio_sam_lighterbev");
    readParameters();

    bev_manager_.reset(new LighterBEVManager(model_path_, lighterbev_device_));
    bev_manager_->metric_scale_ = bev_resolution_;
    ros::param::param("/LighterBEV/iterations",
                     bev_manager_->max_iterations, 1000);
    ros::param::param("/LighterBEV/threshold",
                     bev_manager_->ransac_threshold, 0.5);
    double matching_ratio = 0.9;
    int matching_min = lighterbev_min_inliers_;
    ros::param::param("/LighterBEV/match_ratio_test", matching_ratio, 0.9);
    ros::param::param("/LighterBEV/min_matches", matching_min,
                     lighterbev_min_inliers_);
    bev_manager_->setMatchingParams(static_cast<float>(matching_ratio),
                                    matching_min);

    LighterBEVKeypointConfig keypoint_config;
    ros::param::param("/LighterBEV/keypoint_backend",
                     keypoint_config.backend, std::string("learned_fast"));
    ros::param::param("/LighterBEV/fast_threshold",
                     keypoint_config.fast_threshold, 10);
    ros::param::param("/LighterBEV/fast_nonmax_suppression",
                     keypoint_config.fast_nonmax_suppression, true);
    ros::param::param("/LighterBEV/orb_nfeatures",
                     keypoint_config.orb_nfeatures, 1500);
    double orb_scale_factor = keypoint_config.orb_scale_factor;
    ros::param::param("/LighterBEV/orb_scale_factor", orb_scale_factor, 1.15);
    keypoint_config.orb_scale_factor = static_cast<float>(orb_scale_factor);
    ros::param::param("/LighterBEV/orb_nlevels",
                     keypoint_config.orb_nlevels, 12);
    ros::param::param("/LighterBEV/orb_edge_threshold",
                     keypoint_config.orb_edge_threshold, 8);
    ros::param::param("/LighterBEV/orb_patch_size",
                     keypoint_config.orb_patch_size, 21);
    ros::param::param("/LighterBEV/orb_fast_threshold",
                     keypoint_config.orb_fast_threshold, 3);
    bev_manager_->setKeypointConfig(keypoint_config);

    std::string scene_profile = "indoor";
    std::string bev_input_mode = "single_frame";
    ros::param::param("/scene/profile", scene_profile, scene_profile);
    ros::param::param("/LighterBEV/input_mode", bev_input_mode,
                     bev_input_mode);
    if (bev_input_mode != "single_frame") {
        ROS_FATAL("[Localization] BEV/REIN requires input_mode=single_frame; got '%s'",
                  bev_input_mode.c_str());
        throw std::invalid_argument(
            "localization BEV/REIN input must be exactly one frame");
    }
    ROS_INFO("[Localization] scene_profile=%s BEV/REIN input_mode=%s "
             "keypoint_backend=%s min_inliers=%d",
             scene_profile.c_str(), bev_input_mode.c_str(),
             keypoint_config.backend.c_str(), lighterbev_min_inliers_);

    continuous_gicp_.reset(new NanoGICP());
    configureRegistration(*continuous_gicp_, gicp_threads_,
                          gicp_correspondences_, gicp_max_iterations_,
                          gicp_max_correspondence_distance_,
                          gicp_covariance_threads_);

    map_loaded_ = loadMapBundle();
    state_ = map_loaded_ ? State::RELOCALIZING : State::WAITING_FOR_MAP;
    status_.reason = map_loaded_ ? "startup_global_relocalization" :
                                  "map_bundle_load_failed";

    corrected_odom_pub_ =
        nh_.advertise<nav_msgs::Odometry>("/localization/odom", 20);
    vehicle_odom_pub_ =
        nh_.advertise<nav_msgs::Odometry>(vehicle_odom_topic_, 20);
    corrected_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/localization/aligned_scan", 2);
    prior_map_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/localization/prior_map", 1, true);
    status_pub_ = nh_.advertise<point_lio_sam_lighterbev::LocalizationStatus>(
        "/localization/status", 10, true);
    if (initialpose_enabled_)
        initialpose_refined_pub_ =
            nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                initialpose_refined_topic_, 1, true);
    if (lighterbev_viz_enable_)
        relocalization_match_pub_ =
            nh_.advertise<sensor_msgs::Image>(lighterbev_viz_topic_, 1, false);
    relocalize_service_ = nh_.advertiseService(
        "/localization/relocalize",
        &RobustMapLocalizer::forceRelocalizationService, this);
    pointlio_reset_client_ =
        nh_.serviceClient<std_srvs::Trigger>("/pointlio/reset_local_map");

    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
        nh_, odom_topic_, 30));
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        nh_, tracking_cloud_topic_, 30));
    synchronizer_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(30), *odom_sub_, *cloud_sub_));
    synchronizer_->registerCallback(boost::bind(
        &RobustMapLocalizer::odomCloudCallback, this, _1, _2));

    // This odometry-only path is intentionally independent of the cloud
    // synchronizer. PointLIO can publish propagation updates much faster than
    // the scan rate; retain only the newest sample and expose it to control at
    // a strict, configurable maximum rate.
    control_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        odom_topic_, 1, &RobustMapLocalizer::controlOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
    recovery_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        recovery_cloud_topic_, 2,
        &RobustMapLocalizer::recoveryCloudCallback, this);
    if (initialpose_enabled_)
        initialpose_sub_ =
            nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
                initialpose_topic_, 1,
                &RobustMapLocalizer::initialPoseCallback, this);

    backend_timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(0.1, backend_hz_)),
        &RobustMapLocalizer::backendTimerCallback, this);

    if (map_loaded_)
        publishPriorMap();
    publishStatus(ros::Time::now());

    ROS_INFO("[Localization] map=%s keyframes=%zu frontend<=%.1f Hz backend=%.1f Hz control<=%.1f Hz vehicle_odom=%s base->body=(%.3f, %.3f, %.3f)m",
             map_dir_.c_str(), map_keyframes_.size(), frontend_max_hz_,
             backend_hz_, control_output_hz_, vehicle_odom_topic_.c_str(),
             vehicle_base_to_body_.x(), vehicle_base_to_body_.y(),
             vehicle_base_to_body_.z());
    if (initialpose_enabled_)
        ROS_INFO("[LocalizationInitialPose] enabled topic=%s reference=%s "
                 "refined_topic=%s stationary_required=%d",
                 initialpose_topic_.c_str(),
                 initialpose_reference_is_vehicle_base_ ?
                     vehicle_base_frame_.c_str() : body_frame_.c_str(),
                 initialpose_refined_topic_.c_str(),
                 initialpose_require_stationary_ ? 1 : 0);
}

void RobustMapLocalizer::readParameters()
{
    std::string configured_map_dir;
    std::string configured_model_path;
    nh_.param<std::string>("map_dir", configured_map_dir, "");
    nh_.param<std::string>("model_path", configured_model_path, "");
    nh_.param<std::string>("descriptor_cache_path", descriptor_cache_path_,
                           descriptor_cache_path_);
    nh_.param<std::string>("lighterbev_device", lighterbev_device_,
                           lighterbev_device_);
    map_dir_ = point_lio_sam_lighterbev::resolvePackagePath(
        package_path_, configured_map_dir, "point_lio/localization_map");
    model_path_ = point_lio_sam_lighterbev::resolvePackagePath(
        package_path_, configured_model_path, "models/pca_kitti_best.pt");
    nh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    nh_.param<std::string>("cloud_topic", cloud_topic_, cloud_topic_);
    nh_.param<std::string>("tracking_cloud_topic", tracking_cloud_topic_,
                           cloud_topic_);
    nh_.param("tracking_cloud_is_body_frame", tracking_cloud_is_body_frame_,
              tracking_cloud_is_body_frame_);
    nh_.param<std::string>("recovery_cloud_topic", recovery_cloud_topic_,
                           recovery_cloud_topic_);
    nh_.param("recovery_cloud_max_age_s", recovery_cloud_max_age_s_,
              recovery_cloud_max_age_s_);
    nh_.param("recovery_min_points", recovery_min_points_,
              recovery_min_points_);
    nh_.param("recovery_odom_sync_tolerance_s",
              recovery_odom_sync_tolerance_s_,
              recovery_odom_sync_tolerance_s_);
    nh_.param("use_raw_recovery_cloud", use_raw_recovery_cloud_,
              use_raw_recovery_cloud_);
    nh_.param("initialpose_enabled", initialpose_enabled_,
              initialpose_enabled_);
    nh_.param<std::string>("initialpose_topic", initialpose_topic_,
                           initialpose_topic_);
    nh_.param<std::string>("initialpose_refined_topic",
                           initialpose_refined_topic_,
                           initialpose_refined_topic_);
    nh_.param("initialpose_reference_is_vehicle_base",
              initialpose_reference_is_vehicle_base_,
              initialpose_reference_is_vehicle_base_);
    nh_.param("initialpose_require_stationary",
              initialpose_require_stationary_,
              initialpose_require_stationary_);
    nh_.param("initialpose_stationary_max_speed_mps",
              initialpose_stationary_max_speed_mps_,
              initialpose_stationary_max_speed_mps_);
    nh_.param("initialpose_stationary_max_angular_speed_degps",
              initialpose_stationary_max_angular_speed_degps_,
              initialpose_stationary_max_angular_speed_degps_);
    nh_.param("initialpose_request_timeout_s",
              initialpose_request_timeout_s_,
              initialpose_request_timeout_s_);
    nh_.param("initialpose_min_points", initialpose_min_points_,
              initialpose_min_points_);
    nh_.param("initialpose_gicp_max_iterations",
              initialpose_gicp_max_iterations_,
              initialpose_gicp_max_iterations_);
    nh_.param("initialpose_gicp_max_correspondence_distance",
              initialpose_gicp_max_correspondence_distance_,
              initialpose_gicp_max_correspondence_distance_);
    nh_.param("initialpose_max_score", initialpose_max_score_,
              initialpose_max_score_);
    nh_.param("initialpose_min_correspondence_ratio",
              initialpose_min_correspondence_ratio_,
              initialpose_min_correspondence_ratio_);
    nh_.param("initialpose_max_translation_correction_m",
              initialpose_max_translation_correction_m_,
              initialpose_max_translation_correction_m_);
    nh_.param("initialpose_max_rotation_correction_deg",
              initialpose_max_rotation_correction_deg_,
              initialpose_max_rotation_correction_deg_);
    nh_.param<std::string>("map_frame", map_frame_, map_frame_);
    nh_.param<std::string>("odom_frame", odom_frame_, odom_frame_);
    nh_.param<std::string>("body_frame", body_frame_, body_frame_);
    nh_.param<std::string>("vehicle_odom_topic", vehicle_odom_topic_,
                           vehicle_odom_topic_);
    nh_.param<std::string>("vehicle_base_frame", vehicle_base_frame_,
                           vehicle_base_frame_);
    nh_.param("vehicle_base_to_body_x_m", vehicle_base_to_body_.x(),
              vehicle_base_to_body_.x());
    nh_.param("vehicle_base_to_body_y_m", vehicle_base_to_body_.y(),
              vehicle_base_to_body_.y());
    nh_.param("vehicle_base_to_body_z_m", vehicle_base_to_body_.z(),
              vehicle_base_to_body_.z());

    nh_.param("frontend_max_hz", frontend_max_hz_, frontend_max_hz_);
    nh_.param("control_output_hz", control_output_hz_, control_output_hz_);
    nh_.param("frontend_max_speed_mps", frontend_max_speed_mps_,
              frontend_max_speed_mps_);
    nh_.param("frontend_max_angular_speed_degps",
              frontend_max_angular_speed_degps_,
              frontend_max_angular_speed_degps_);
    nh_.param("frontend_max_frame_jump_m", frontend_max_frame_jump_m_,
              frontend_max_frame_jump_m_);
    nh_.param("frontend_bad_frame_limit", frontend_bad_frame_limit_,
              frontend_bad_frame_limit_);
    nh_.param("backend_hz", backend_hz_, backend_hz_);
    nh_.param("accumulation_frames", accumulation_frames_,
              accumulation_frames_);
    nh_.param("input_voxel_size", input_voxel_size_, input_voxel_size_);
    nh_.param("registration_voxel_size", registration_voxel_size_,
              registration_voxel_size_);
    nh_.param("local_map_radius", local_map_radius_, local_map_radius_);
    nh_.param("local_map_refresh_distance", local_map_refresh_distance_,
              local_map_refresh_distance_);
    nh_.param("local_map_max_keyframes", local_map_max_keyframes_,
              local_map_max_keyframes_);
    nh_.param("continuous_target_incremental",
              continuous_target_incremental_, continuous_target_incremental_);
    nh_.param("continuous_target_compaction_updates",
              continuous_target_compaction_updates_,
              continuous_target_compaction_updates_);
    nh_.param("continuous_target_compaction_stale_ratio",
              continuous_target_compaction_stale_ratio_,
              continuous_target_compaction_stale_ratio_);
    continuous_target_compaction_updates_ =
        std::max(continuous_target_compaction_updates_, 1);
    continuous_target_compaction_stale_ratio_ = std::max(
        0.0, std::min(continuous_target_compaction_stale_ratio_, 0.95));
    nh_.param("relocalization_submap_half_width",
              relocalization_submap_half_width_,
              relocalization_submap_half_width_);

    nh_.param("gicp_threads", gicp_threads_, gicp_threads_);
    nh_.param("gicp_covariance_threads", gicp_covariance_threads_,
              gicp_covariance_threads_);
    nh_.param("gicp_correspondences", gicp_correspondences_,
              gicp_correspondences_);
    nh_.param("gicp_max_iterations", gicp_max_iterations_,
              gicp_max_iterations_);
    nh_.param("gicp_max_correspondence_distance",
              gicp_max_correspondence_distance_,
              gicp_max_correspondence_distance_);
    nh_.param("continuous_max_score", continuous_max_score_,
              continuous_max_score_);
    nh_.param("continuous_min_correspondence_ratio",
              continuous_min_correspondence_ratio_,
              continuous_min_correspondence_ratio_);
    nh_.param("continuous_max_translation_update",
              continuous_max_translation_update_,
              continuous_max_translation_update_);
    nh_.param("continuous_max_rotation_update_deg",
              continuous_max_rotation_update_deg_,
              continuous_max_rotation_update_deg_);
    nh_.param("correction_smoothing_alpha", correction_smoothing_alpha_,
              correction_smoothing_alpha_);
    nh_.param("tracking_failure_limit", tracking_failure_limit_,
              tracking_failure_limit_);
    nh_.param("pose_jump_log_translation_m",
              pose_jump_log_translation_m_,
              pose_jump_log_translation_m_);
    nh_.param("pose_jump_log_rotation_deg",
              pose_jump_log_rotation_deg_,
              pose_jump_log_rotation_deg_);

    ros::param::param("/LighterBEV/range", bev_range_, bev_range_);
    ros::param::param("/LighterBEV/resolution", bev_resolution_,
                     bev_resolution_);
    ros::param::param("/LighterBEV/downsample_enable",
                     bev_downsample_enable_, bev_downsample_enable_);
    ros::param::param("/LighterBEV/downsample_voxel_size",
                     bev_voxel_size_, bev_voxel_size_);
    ros::param::param("/LighterBEV/z_max_filter_enable",
                     bev_z_max_filter_enable_, bev_z_max_filter_enable_);
    ros::param::param("/LighterBEV/z_max", bev_z_max_, bev_z_max_);
    nh_.param("bev_threads", bev_threads_, bev_threads_);
    nh_.param("relocalization_top_k", relocalization_top_k_,
              relocalization_top_k_);
    double descriptor_distance = descriptor_max_distance_;
    nh_.param("descriptor_max_distance", descriptor_distance,
              descriptor_distance);
    descriptor_max_distance_ = static_cast<float>(descriptor_distance);
    nh_.param("lighterbev_min_inliers", lighterbev_min_inliers_,
              lighterbev_min_inliers_);
    ros::param::param("/LighterBEV/ninliers", lighterbev_min_inliers_,
                     lighterbev_min_inliers_);
    nh_.param("relocalization_max_score", relocalization_max_score_,
              relocalization_max_score_);
    nh_.param("relocalization_min_correspondence_ratio",
              relocalization_min_correspondence_ratio_,
              relocalization_min_correspondence_ratio_);
    nh_.param("relocalization_required_confirmations",
              relocalization_required_confirmations_,
              relocalization_required_confirmations_);
    nh_.param("hypothesis_max_translation_delta",
              hypothesis_max_translation_delta_,
              hypothesis_max_translation_delta_);
    nh_.param("hypothesis_max_rotation_delta_deg",
              hypothesis_max_rotation_delta_deg_,
              hypothesis_max_rotation_delta_deg_);
    nh_.param("relocalization_retry_sec", relocalization_retry_sec_,
              relocalization_retry_sec_);
    nh_.param("outside_map_relocalization_retry_sec",
              outside_map_relocalization_retry_sec_,
              outside_map_relocalization_retry_sec_);
    nh_.param("global_probe_interval_sec", global_probe_interval_sec_,
              global_probe_interval_sec_);
    nh_.param("severe_global_translation_error",
              severe_global_translation_error_,
              severe_global_translation_error_);
    nh_.param("severe_global_rotation_error_deg",
              severe_global_rotation_error_deg_,
              severe_global_rotation_error_deg_);
    nh_.param("recovery_settle_sec", recovery_settle_sec_,
              recovery_settle_sec_);
    nh_.param("reset_pointlio_local_map_on_recovery",
              reset_pointlio_local_map_on_recovery_,
              reset_pointlio_local_map_on_recovery_);
    nh_.param("lighterbev_enable_viz", lighterbev_viz_enable_,
              lighterbev_viz_enable_);
    nh_.param<std::string>("lighterbev_viz_topic", lighterbev_viz_topic_,
                           lighterbev_viz_topic_);

    accumulation_frames_ = std::max(1, accumulation_frames_);
    frontend_bad_frame_limit_ = std::max(1, frontend_bad_frame_limit_);
    tracking_failure_limit_ = std::max(1, tracking_failure_limit_);
    pose_jump_log_translation_m_ =
        std::max(0.0, pose_jump_log_translation_m_);
    pose_jump_log_rotation_deg_ =
        std::max(0.0, pose_jump_log_rotation_deg_);
    relocalization_required_confirmations_ =
        std::max(1, relocalization_required_confirmations_);
    relocalization_retry_sec_ = std::max(0.0, relocalization_retry_sec_);
    outside_map_relocalization_retry_sec_ =
        std::max(0.0, outside_map_relocalization_retry_sec_);
    initialpose_stationary_max_speed_mps_ =
        std::max(0.0, initialpose_stationary_max_speed_mps_);
    initialpose_stationary_max_angular_speed_degps_ =
        std::max(0.0, initialpose_stationary_max_angular_speed_degps_);
    initialpose_request_timeout_s_ =
        std::max(0.1, initialpose_request_timeout_s_);
    initialpose_min_points_ = std::max(50, initialpose_min_points_);
    initialpose_gicp_max_iterations_ =
        std::max(1, initialpose_gicp_max_iterations_);
    initialpose_gicp_max_correspondence_distance_ =
        std::max(0.1, initialpose_gicp_max_correspondence_distance_);
    initialpose_max_score_ = std::max(0.0, initialpose_max_score_);
    initialpose_min_correspondence_ratio_ = std::max(
        0.0, std::min(1.0, initialpose_min_correspondence_ratio_));
    initialpose_max_translation_correction_m_ =
        std::max(0.0, initialpose_max_translation_correction_m_);
    initialpose_max_rotation_correction_deg_ =
        std::max(0.0, initialpose_max_rotation_correction_deg_);
}

bool RobustMapLocalizer::loadPoses(
    const std::string &path,
    std::vector<std::pair<double, Eigen::Matrix4d>> &poses) const
{
    poses.clear();
    std::ifstream stream(path);
    if (!stream)
        return false;

    std::string line;
    while (std::getline(stream, line))
    {
        if (line.empty() || line.front() == '#')
            continue;
        std::istringstream parser(line);
        double stamp, x, y, z, qx, qy, qz, qw;
        if (!(parser >> stamp >> x >> y >> z >> qx >> qy >> qz >> qw))
            continue;
        Eigen::Quaterniond quaternion(qw, qx, qy, qz);
        if (quaternion.norm() < 1.0e-9)
            continue;
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
        pose.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
        poses.emplace_back(stamp, pose);
    }
    return !poses.empty();
}

bool RobustMapLocalizer::loadMapBundle()
{
    if (!fs::exists(map_dir_ + "/poses_tum.txt") && fs::is_directory(map_dir_))
    {
        std::string newest_bundle;
        std::time_t newest_time = 0;
        for (fs::directory_iterator entry(map_dir_), end; entry != end; ++entry)
        {
            if (!fs::is_directory(entry->path()) ||
                !fs::exists(entry->path() / "poses_tum.txt") ||
                !fs::exists(entry->path() / "point_lio_map.pcd"))
                continue;
            const std::time_t modified =
                fs::last_write_time(entry->path() / "poses_tum.txt");
            if (newest_bundle.empty() || modified > newest_time)
            {
                newest_bundle = entry->path().string();
                newest_time = modified;
            }
        }
        if (!newest_bundle.empty())
        {
            ROS_INFO("[Localization] Selected latest map bundle: %s",
                     newest_bundle.c_str());
            map_dir_ = newest_bundle;
        }
    }

    resolveDescriptorCachePath();

    std::vector<std::pair<double, Eigen::Matrix4d>> poses;
    if (!loadPoses(map_dir_ + "/poses_tum.txt", poses))
    {
        ROS_ERROR("[Localization] Cannot load %s/poses_tum.txt", map_dir_.c_str());
        return false;
    }

    const bool has_raw_keyframes = fs::exists(map_dir_ + "/keyframes_raw");
    const std::string keyframe_dir = has_raw_keyframes ?
        map_dir_ + "/keyframes_raw" : map_dir_ + "/scans";
    map_keyframes_.clear();
    map_keyframes_.reserve(poses.size());
    for (size_t index = 0; index < poses.size(); ++index)
    {
        std::ostringstream filename;
        filename << std::setw(6) << std::setfill('0') << index;
        const std::string pcd_path = keyframe_dir + "/" + filename.str() + ".pcd";
        std::string image_path =
            map_dir_ + "/bev_imgs/" + filename.str() + ".png";
        if (!fs::exists(image_path))
            image_path = map_dir_ + "/bev_imgs/" +
                         std::to_string(index) + ".png";

        MapKeyframe keyframe;
        keyframe.timestamp = poses[index].first;
        keyframe.map_from_body = poses[index].second;
        if (pcl::io::loadPCDFile<PointType>(pcd_path,
                                            *keyframe.points_body) != 0 ||
            keyframe.points_body->empty())
        {
            ROS_ERROR("[Localization] Cannot load keyframe %s", pcd_path.c_str());
            map_keyframes_.clear();
            return false;
        }
        if (has_raw_keyframes)
        {
            // Raw keyframes and poses share the same index by construction.
            // Regenerate BEV in memory instead of trusting legacy bev_imgs/
            // directories, which may contain stale real-time images.
            PosePcd pose_pcd;
            pose_pcd.idx_ = static_cast<int>(index);
            pose_pcd.pcd_ = *keyframe.points_body;
            keyframe.bev.bev_img = getBEVImageParallelAndSave(
                pose_pcd, bev_threads_, "", true, bev_range_,
                bev_resolution_, bev_downsample_enable_, bev_voxel_size_,
                false, bev_z_max_filter_enable_, bev_z_max_);
        }
        else
        {
            // scans/ are exported in map coordinates. Recover body-frame
            // points so submap construction does not apply the pose twice.
            pcl::PointCloud<PointType> body_cloud;
            pcl::transformPointCloud(
                *keyframe.points_body, body_cloud,
                keyframe.map_from_body.inverse().cast<float>());
            *keyframe.points_body = std::move(body_cloud);
            keyframe.bev.bev_img =
                cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        }
        if (keyframe.bev.bev_img.empty())
        {
            ROS_ERROR("[Localization] Cannot load BEV %s", image_path.c_str());
            map_keyframes_.clear();
            return false;
        }
        keyframe.bev.header.stamp.fromSec(keyframe.timestamp);
        keyframe.bev.header.frame_id = map_frame_;
        keyframe.bev.pose = keyframe.map_from_body;
        map_keyframes_.emplace_back(std::move(keyframe));
    }

    const std::string map_path = map_dir_ + "/point_lio_map.pcd";
    if (pcl::io::loadPCDFile<PointType>(map_path, *prior_map_) != 0 ||
        prior_map_->empty())
    {
        ROS_WARN("[Localization] %s missing; aggregating map keyframes for visualization.",
                 map_path.c_str());
        for (const auto &keyframe : map_keyframes_)
        {
            pcl::PointCloud<PointType> transformed;
            pcl::transformPointCloud(*keyframe.points_body, transformed,
                                     keyframe.map_from_body.cast<float>());
            *prior_map_ += transformed;
        }
        prior_map_ = downsample(prior_map_, registration_voxel_size_);
    }

    if (!loadDescriptorDatabase() && !buildDescriptorDatabase())
    {
        ROS_ERROR("[Localization] Cannot build LighterBEV descriptor database.");
        map_keyframes_.clear();
        return false;
    }
    return true;
}

void RobustMapLocalizer::resolveDescriptorCachePath()
{
    if (!descriptor_cache_path_.empty())
    {
        fs::path configured(descriptor_cache_path_);
        if (configured.is_relative())
            configured = fs::path(map_dir_) / configured;
        descriptor_cache_path_ = configured.string();
        return;
    }

    std::string model_stem = fs::path(model_path_).stem().string();
    for (char &character : model_stem)
    {
        const bool safe =
            (character >= 'a' && character <= 'z') ||
            (character >= 'A' && character <= 'Z') ||
            (character >= '0' && character <= '9') ||
            character == '-' || character == '_';
        if (!safe)
            character = '_';
    }
    if (model_stem.empty())
        model_stem = "unnamed_model";

    // Keep the verified production cache path backward compatible. Every
    // other weight gets an isolated cache so descriptors produced by two
    // networks can never be mixed during a profile switch.
    const std::string filename = model_stem == "pca_kitti_best" ?
        "lighterbev_descriptors.bin" :
        "lighterbev_descriptors_" + model_stem + ".bin";
    descriptor_cache_path_ = (fs::path(map_dir_) / filename).string();
}

bool RobustMapLocalizer::loadDescriptorDatabase()
{
    std::vector<lighterbev_descriptor_io::DescriptorRecord> records;
    if (!lighterbev_descriptor_io::load(
            descriptor_cache_path_, records,
            bev_range_, bev_resolution_, bev_downsample_enable_,
            bev_voxel_size_, bev_z_max_filter_enable_, bev_z_max_) ||
        records.size() != map_keyframes_.size())
        return false;

    for (size_t index = 0; index < records.size(); ++index)
    {
        if (std::abs(records[index].timestamp -
                     map_keyframes_[index].timestamp) > 1.0e-4)
            return false;
        if (!bev_manager_->addKeyFrameDescriptor(records[index].descriptor,
                                                  records[index].timestamp))
            return false;
        map_keyframes_[index].bev.global_desc = records[index].descriptor;
    }
    ROS_INFO("[Localization] Loaded %zu cached LighterBEV descriptors from %s.",
             records.size(), descriptor_cache_path_.c_str());
    return true;
}

bool RobustMapLocalizer::buildDescriptorDatabase()
{
    std::vector<lighterbev_descriptor_io::DescriptorRecord> records;
    records.reserve(map_keyframes_.size());
    ROS_WARN("[Localization] Descriptor cache missing; computing %zu map descriptors once.",
             map_keyframes_.size());
    for (size_t index = 0; index < map_keyframes_.size(); ++index)
    {
        try
        {
            bev_manager_->addNewKeyFrame(map_keyframes_[index].bev);
        }
        catch (const std::exception &error)
        {
            ROS_ERROR("[Localization] Descriptor %zu failed: %s", index,
                      error.what());
            return false;
        }
        lighterbev_descriptor_io::DescriptorRecord record;
        record.timestamp = map_keyframes_[index].timestamp;
        record.descriptor = map_keyframes_[index].bev.global_desc;
        records.emplace_back(std::move(record));
    }
    if (!lighterbev_descriptor_io::save(
            descriptor_cache_path_, records,
            bev_range_, bev_resolution_, bev_downsample_enable_,
            bev_voxel_size_, bev_z_max_filter_enable_, bev_z_max_))
    {
        ROS_WARN("[Localization] Map is usable, but descriptor cache could not be written.");
    }
    else
    {
        ROS_INFO("[Localization] Saved %zu LighterBEV descriptors to %s.",
                 records.size(), descriptor_cache_path_.c_str());
    }
    return !records.empty();
}

void RobustMapLocalizer::odomCloudCallback(
    const nav_msgs::OdometryConstPtr &odom_msg,
    const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    const double stamp = odom_msg->header.stamp.toSec();
    if (!std::isfinite(stamp))
        return;
    if (last_accepted_input_stamp_ >= 0.0 && stamp >= last_accepted_input_stamp_ &&
        frontend_max_hz_ > 0.0 &&
        stamp - last_accepted_input_stamp_ < 1.0 / frontend_max_hz_)
        return;
    if (stamp < last_accepted_input_stamp_)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        live_frames_.clear();
        frontend_bad_count_ = 0;
    }
    last_accepted_input_stamp_ = stamp;

    LiveFrame frame;
    frame.sequence = next_sequence_++;
    frame.stamp = odom_msg->header.stamp;
    frame.odom_from_body = odometryToMatrix(*odom_msg);
    pcl::PointCloud<PointType>::Ptr local_cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *local_cloud);
    if (!tracking_cloud_is_body_frame_)
    {
        // Legacy path: the cloud arrives in the Point-LIO odom/world frame
        // and must be brought back to body with the matching frontend pose.
        const auto world_cloud = local_cloud;
        local_cloud.reset(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(
            *world_cloud, *local_cloud,
            frame.odom_from_body.inverse().cast<float>());
    }
    frame.points_body = downsample(local_cloud, input_voxel_size_);
    if (!frame.points_body || frame.points_body->size() < 50)
        return;

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (!live_frames_.empty())
        {
            const LiveFrame &previous = live_frames_.back();
            const double dt = (frame.stamp - previous.stamp).toSec();
            if (dt > 1.0e-4)
            {
                const Eigen::Matrix4d delta =
                    previous.odom_from_body.inverse() * frame.odom_from_body;
                const double translation = delta.block<3, 1>(0, 3).norm();
                const double speed = translation / dt;
                const double angular_speed = rotationAngleDeg(delta) / dt;
                const bool bad = translation > frontend_max_frame_jump_m_ ||
                                 speed > frontend_max_speed_mps_ ||
                                 angular_speed > frontend_max_angular_speed_degps_;
                frontend_bad_count_ = bad ? frontend_bad_count_ + 1 : 0;
                frontend_healthy_ =
                    frontend_bad_count_ < frontend_bad_frame_limit_;
            }
        }
        live_frames_.push_back(frame);
        while (static_cast<int>(live_frames_.size()) > accumulation_frames_)
            live_frames_.pop_front();
    }

    // Keep scan processing separate from the odometry-only control path.
    // The odometry callback applies the latest map->odom correction exactly
    // once per selected PointLIO odometry sample.
}

void RobustMapLocalizer::controlOdomCallback(
    const nav_msgs::OdometryConstPtr &odom_msg)
{
    if (!odom_msg)
        return;
    const double stamp = odom_msg->header.stamp.toSec();
    if (!std::isfinite(stamp))
        return;

    {
        std::lock_guard<std::mutex> lock(recent_odom_mutex_);
        if (recent_odom_.empty() ||
            stamp >= recent_odom_.back().stamp.toSec())
        {
            StampedOdom sample;
            sample.stamp = odom_msg->header.stamp;
            sample.odom_from_body = odometryToMatrix(*odom_msg);
            recent_odom_.push_back(sample);
            const double horizon = stamp - 2.0;
            while (!recent_odom_.empty() &&
                   recent_odom_.front().stamp.toSec() < horizon)
                recent_odom_.pop_front();
        }
    }

    if (!has_map_from_odom_)
        return;

    {
        std::lock_guard<std::mutex> lock(control_odom_mutex_);
        if (stamp < last_control_output_stamp_)
            last_control_output_stamp_ = -1.0;
        if (stamp == last_control_output_stamp_)
            return;
        if (control_output_hz_ > 0.0 && last_control_output_stamp_ >= 0.0 &&
            stamp - last_control_output_stamp_ + 1.0e-9 <
                1.0 / control_output_hz_)
            return;
        last_control_output_stamp_ = stamp;
    }

    Eigen::Matrix4d map_from_odom;
    uint64_t correction_sequence = 0;
    std::string correction_reason;
    bool correction_hard_reset = false;
    double correction_translation = 0.0;
    double correction_rotation_deg = 0.0;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (!has_map_from_odom_)
            return;
        map_from_odom = map_from_odom_;
        correction_sequence = map_correction_sequence_;
        correction_reason = last_map_correction_reason_;
        correction_hard_reset = last_map_correction_hard_reset_;
        correction_translation = last_map_correction_translation_;
        correction_rotation_deg = last_map_correction_rotation_deg_;
    }

    // The backend registration callback may occasionally spend several
    // seconds matching a scan. Publishing map->odom only from that callback
    // makes the otherwise high-rate control pose unusable by move_base once
    // its transform tolerance expires. The correction itself is piecewise
    // constant between backend updates, so refresh the same transform at the
    // timestamp of every accepted control odometry sample.
    publishMapToOdom(odom_msg->header.stamp);

    const Eigen::Matrix4d raw_odom_from_body = odometryToMatrix(*odom_msg);
    const Eigen::Matrix4d map_from_body = map_from_odom * raw_odom_from_body;

    // The first corrected sample only establishes the baseline. This excludes
    // startup global localization as requested. Every later discontinuity is
    // decomposed into raw PointLIO motion versus map->odom correction motion.
    {
        std::lock_guard<std::mutex> lock(control_odom_mutex_);
        if (pose_jump_baseline_valid_ && stamp > previous_pose_jump_stamp_)
        {
            const double dt = stamp - previous_pose_jump_stamp_;
            const Eigen::Matrix4d raw_delta =
                previous_raw_odom_from_body_.inverse() * raw_odom_from_body;
            const Eigen::Matrix4d output_delta =
                previous_corrected_map_from_body_.inverse() * map_from_body;
            const double raw_translation =
                raw_delta.block<3, 1>(0, 3).norm();
            const double raw_rotation_deg = rotationAngleDeg(raw_delta);
            const double output_translation =
                output_delta.block<3, 1>(0, 3).norm();
            const double output_rotation_deg = rotationAngleDeg(output_delta);
            const bool jumped =
                output_translation >= pose_jump_log_translation_m_ ||
                output_rotation_deg >= pose_jump_log_rotation_deg_;
            if (jumped)
            {
                ROS_WARN_STREAM(
                    std::fixed << std::setprecision(6)
                    << "[LocalizationJump] runtime_output_jump"
                    << " stamp=" << stamp << " dt=" << dt
                    << " output_dtrans=" << output_translation
                    << " output_drot_deg=" << output_rotation_deg
                    << " raw_pointlio_dtrans=" << raw_translation
                    << " raw_pointlio_drot_deg=" << raw_rotation_deg
                    << " correction_changed="
                    << (correction_sequence != previous_map_correction_sequence_)
                    << " correction_seq=" << correction_sequence
                    << " correction_reason=" << correction_reason
                    << " correction_hard_reset=" << correction_hard_reset
                    << " correction_dtrans=" << correction_translation
                    << " correction_drot_deg=" << correction_rotation_deg
                    << " raw_xyz=" << raw_odom_from_body(0, 3) << ","
                    << raw_odom_from_body(1, 3) << ","
                    << raw_odom_from_body(2, 3)
                    << " output_xyz=" << map_from_body(0, 3) << ","
                    << map_from_body(1, 3) << ","
                    << map_from_body(2, 3));
            }
        }
        previous_raw_odom_from_body_ = raw_odom_from_body;
        previous_corrected_map_from_body_ = map_from_body;
        previous_pose_jump_stamp_ = stamp;
        previous_map_correction_sequence_ = correction_sequence;
        pose_jump_baseline_valid_ = true;
    }
    nav_msgs::Odometry corrected = matrixToOdometry(
        map_from_body, odom_msg->header.stamp, map_frame_, body_frame_);
    // Twist is expressed in the child frame for standard nav_msgs/Odometry,
    // so it remains valid when only the parent frame changes odom -> map.
    corrected.twist = odom_msg->twist;
    corrected_odom_pub_.publish(corrected);

    // Point-LIO estimates the pose and twist at the MID360/IMU (body) origin.
    // Publish a second, control-specific odometry at the vehicle center while
    // preserving /localization/odom for existing localization consumers.
    // Axes are assumed aligned; only the measured rigid translation is used.
    Eigen::Matrix4d body_from_vehicle_base = Eigen::Matrix4d::Identity();
    body_from_vehicle_base.block<3, 1>(0, 3) = -vehicle_base_to_body_;
    const Eigen::Matrix4d map_from_vehicle_base =
        map_from_body * body_from_vehicle_base;
    nav_msgs::Odometry vehicle = matrixToOdometry(
        map_from_vehicle_base, odom_msg->header.stamp, map_frame_,
        vehicle_base_frame_);
    vehicle.pose.covariance = corrected.pose.covariance;
    vehicle.twist = corrected.twist;

    const Eigen::Vector3d angular_body(
        corrected.twist.twist.angular.x,
        corrected.twist.twist.angular.y,
        corrected.twist.twist.angular.z);
    const Eigen::Vector3d linear_body(
        corrected.twist.twist.linear.x,
        corrected.twist.twist.linear.y,
        corrected.twist.twist.linear.z);
    const Eigen::Vector3d body_to_vehicle_base = -vehicle_base_to_body_;
    const Eigen::Vector3d linear_vehicle =
        linear_body + angular_body.cross(body_to_vehicle_base);
    vehicle.twist.twist.linear.x = linear_vehicle.x();
    vehicle.twist.twist.linear.y = linear_vehicle.y();
    vehicle.twist.twist.linear.z = linear_vehicle.z();
    vehicle_odom_pub_.publish(vehicle);
}

pcl::PointCloud<PointType>::Ptr RobustMapLocalizer::buildAccumulatedSource(
    const std::vector<LiveFrame> &frames, const LiveFrame &latest) const
{
    pcl::PointCloud<PointType>::Ptr accumulated(new pcl::PointCloud<PointType>());
    for (const auto &frame : frames)
    {
        const Eigen::Matrix4d latest_body_from_frame_body =
            latest.odom_from_body.inverse() * frame.odom_from_body;
        pcl::PointCloud<PointType> transformed;
        pcl::transformPointCloud(*frame.points_body, transformed,
                                 latest_body_from_frame_body.cast<float>());
        *accumulated += transformed;
    }
    // Preserve raw density for LighterBEV. Map descriptors are generated by
    // applying bev_voxel_size_ directly to the deskewed keyframe; applying the
    // coarser registration voxel first changes the BEV occupancy statistics.
    // Nano-GICP receives a separate downsampled copy at its call site.
    return accumulated;
}

pcl::PointCloud<PointType>::Ptr RobustMapLocalizer::buildMapSubmap(
    const Eigen::Vector3d &center, int center_keyframe,
    bool force_center_keyframe) const
{
    std::vector<std::pair<double, int>> selected;
    if (force_center_keyframe && center_keyframe >= 0)
    {
        const int begin = std::max(0, center_keyframe -
                                       relocalization_submap_half_width_);
        const int end = std::min(static_cast<int>(map_keyframes_.size()) - 1,
                                 center_keyframe +
                                     relocalization_submap_half_width_);
        for (int index = begin; index <= end; ++index)
            selected.emplace_back(std::abs(index - center_keyframe), index);
    }
    else
    {
        selected.reserve(map_keyframes_.size());
        for (size_t index = 0; index < map_keyframes_.size(); ++index)
        {
            const double distance =
                (map_keyframes_[index].map_from_body.block<3, 1>(0, 3) -
                 center).norm();
            if (distance <= local_map_radius_)
                selected.emplace_back(distance, static_cast<int>(index));
        }
        std::sort(selected.begin(), selected.end());
        if (static_cast<int>(selected.size()) > local_map_max_keyframes_)
            selected.resize(local_map_max_keyframes_);
    }

    pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>());
    for (const auto &entry : selected)
    {
        const MapKeyframe &keyframe = map_keyframes_[entry.second];
        pcl::PointCloud<PointType> transformed;
        pcl::transformPointCloud(*keyframe.points_body, transformed,
                                 keyframe.map_from_body.cast<float>());
        *target += transformed;
    }
    return downsample(target, registration_voxel_size_);
}

std::vector<int> RobustMapLocalizer::selectKeyframesAround(
    const Eigen::Vector3d &center) const
{
    std::vector<std::pair<double, int>> selected;
    selected.reserve(map_keyframes_.size());
    for (size_t index = 0; index < map_keyframes_.size(); ++index)
    {
        const double distance =
            (map_keyframes_[index].map_from_body.block<3, 1>(0, 3) -
             center)
                .norm();
        if (distance <= local_map_radius_)
            selected.emplace_back(distance, static_cast<int>(index));
    }
    std::sort(selected.begin(), selected.end());
    if (static_cast<int>(selected.size()) > local_map_max_keyframes_)
        selected.resize(local_map_max_keyframes_);

    std::vector<int> indices;
    indices.reserve(selected.size());
    for (const auto &entry : selected)
        indices.push_back(entry.second);
    std::sort(indices.begin(), indices.end());
    return indices;
}

const pcl::PointCloud<PointType>::Ptr &RobustMapLocalizer::getKeyframeWorldCloud(
    int index)
{
    if (keyframe_world_cache_.size() != map_keyframes_.size())
    {
        keyframe_world_cache_.assign(map_keyframes_.size(), nullptr);
    }
    if (!keyframe_world_cache_[index])
    {
        const MapKeyframe &keyframe = map_keyframes_[index];
        pcl::PointCloud<PointType>::Ptr world(
            new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*keyframe.points_body, *world,
                                 keyframe.map_from_body.cast<float>());
        keyframe_world_cache_[index] =
            downsample(world, registration_voxel_size_);
    }
    return keyframe_world_cache_[index];
}

pcl::PointCloud<PointType>::Ptr RobustMapLocalizer::concatKeyframeWorldClouds(
    const std::vector<int> &indices)
{
    pcl::PointCloud<PointType>::Ptr target(new pcl::PointCloud<PointType>());
    for (const int index : indices)
    {
        const auto &cloud = getKeyframeWorldCloud(index);
        if (cloud)
            *target += *cloud;
    }
    return target;
}

bool RobustMapLocalizer::refreshContinuousTarget(
    const Eigen::Matrix4d &predicted_map_from_body)
{
    const Eigen::Vector3d center = predicted_map_from_body.block<3, 1>(0, 3);
    const auto update_begin = std::chrono::steady_clock::now();
    last_target_delete_ms_ = 0.0;
    const auto mark_update = [&]() {
        last_target_update_ms_ =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - update_begin)
                .count();
    };
    if (!continuous_target_->empty() && continuous_target_center_.allFinite() &&
        (center - continuous_target_center_).norm() <
            local_map_refresh_distance_)
    {
        last_target_update_ms_ = 0.0;
        return true;
    }

    if (continuous_target_incremental_)
    {
        const std::vector<int> selection = selectKeyframesAround(center);
        if (selection.empty())
            return false;

        if (!continuous_gicp_->ikdTargetMode())
        {
            pcl::PointCloud<PointType>::Ptr target =
                concatKeyframeWorldClouds(selection);
            if (!target || target->size() < 100)
                return false;
            continuous_gicp_->setInputTargetIkd(target);
            continuous_target_ = target;
            continuous_target_keyframes_ = selection;
            continuous_target_updates_since_build_ = 0;
            continuous_target_center_ = center;
            mark_update();
            return true;
        }

        std::vector<int> added, removed;
        std::set_difference(selection.begin(), selection.end(),
                            continuous_target_keyframes_.begin(),
                            continuous_target_keyframes_.end(),
                            std::back_inserter(added));
        std::set_difference(continuous_target_keyframes_.begin(),
                            continuous_target_keyframes_.end(),
                            selection.begin(), selection.end(),
                            std::back_inserter(removed));

        // Delete exact keyframe-owned points before additions. Spatial AABB
        // deletion is invalid here because neighboring keyframes overlap and
        // would erase retained/new target geometry.
        for (const int index : removed)
        {
            const auto &cloud = getKeyframeWorldCloud(index);
            if (!cloud || cloud->empty())
                continue;
            const auto del_begin = std::chrono::steady_clock::now();
            continuous_gicp_->deleteTargetPointsIkd(cloud);
            last_target_delete_ms_ +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - del_begin)
                    .count();
        }
        for (const int index : added)
        {
            const auto &cloud = getKeyframeWorldCloud(index);
            if (cloud && cloud->size() >= 10)
                continuous_gicp_->addTargetPointsIkd(cloud);
        }

        continuous_target_keyframes_ = selection;
        const bool target_changed = !added.empty() || !removed.empty();
        if (target_changed)
            ++continuous_target_updates_since_build_;
        const std::size_t active_points =
            continuous_gicp_->activeTargetPoints();
        const std::size_t stored_points =
            continuous_gicp_->storedTargetPoints();
        const double stale_ratio = stored_points > 0 &&
                                           stored_points >= active_points
                                       ? static_cast<double>(stored_points -
                                                             active_points) /
                                             stored_points
                                       : 0.0;
        if (continuous_target_updates_since_build_ >=
                continuous_target_compaction_updates_ ||
            stale_ratio >= continuous_target_compaction_stale_ratio_)
        {
            // Periodic compaction bounds stale target points accumulated from
            // removals (their indices are kept stable for GICP covariance
            // bookkeeping but they are excluded from the ikd tree).
            pcl::PointCloud<PointType>::Ptr rebuilt =
                concatKeyframeWorldClouds(selection);
            continuous_gicp_->setInputTargetIkd(rebuilt);
            continuous_target_ = rebuilt;
            continuous_target_updates_since_build_ = 0;
        }
        continuous_target_center_ = center;
        mark_update();
        return true;
    }

    pcl::PointCloud<PointType>::Ptr target =
        buildMapSubmap(center, -1, false);
    if (!target || target->size() < 100)
        return false;
    continuous_target_ = target;
    continuous_target_center_ = center;
    continuous_gicp_->setInputTarget(continuous_target_);
    continuous_gicp_->calculateTargetCovariances();
    continuous_target_tree_.setInputCloud(continuous_target_);
    mark_update();
    return true;
}

RobustMapLocalizer::RegistrationResult RobustMapLocalizer::registerCloud(
    NanoGICP &registration,
    const pcl::PointCloud<PointType>::Ptr &source,
    const pcl::PointCloud<PointType>::Ptr &target,
    const Eigen::Matrix4d &initial_map_from_body,
    double max_score, double min_correspondence_ratio,
    bool target_is_prepared) const
{
    RegistrationResult result;
    result.map_from_body = initial_map_from_body;
    const std::size_t target_points =
        target_is_prepared && registration.ikdTargetMode()
            ? registration.activeTargetPoints()
            : (target ? target->size() : 0);
    if (!source || !target || source->size() < 50 || target_points < 100 ||
        !finiteTransform(initial_map_from_body))
    {
        result.reason = "registration_input_invalid";
        return result;
    }

    if (!target_is_prepared)
    {
        registration.setInputTarget(target);
        registration.calculateTargetCovariances();
    }
    registration.setInputSource(source);
    registration.calculateSourceCovariances();
    pcl::PointCloud<PointType> aligned;
    registration.align(aligned, initial_map_from_body.cast<float>());
    result.converged = registration.hasConverged();
    result.map_from_body = registration.getFinalTransformation().cast<double>();
    result.score = registration.ikdTargetMode()
                       ? registration.meanSquaredCorrespondenceResidual()
                       : registration.getFitnessScore(
                             gicp_max_correspondence_distance_);

    // Nano-GICP already performed the identical nearest-neighbour pass during
    // its final update_correspondences() call (same corr_dist_threshold), so
    // read the cached statistics instead of rebuilding a KdTree and querying
    // every aligned point again.
    const int correspondences = registration.numCorrespondences();
    result.correspondence_ratio = aligned.empty() ? 0.0 :
        static_cast<double>(correspondences) / aligned.size();
    result.mean_residual = registration.meanCorrespondenceResidual();
    const Eigen::Matrix4d innovation =
        initial_map_from_body.inverse() * result.map_from_body;
    result.innovation_translation = innovation.block<3, 1>(0, 3).norm();
    result.innovation_rotation_deg = rotationAngleDeg(innovation);
    // Nano-GICP reports hasConverged()==false when it reaches the configured
    // iteration cap, even when the final transform already has strong geometric
    // support.  For a bounded real-time backend, gate on the actual result
    // quality here; continuous tracking additionally applies strict translation
    // and rotation innovation limits before updating map->odom.
    result.valid = finiteTransform(result.map_from_body) &&
                   std::isfinite(result.score) && result.score <= max_score &&
                   result.correspondence_ratio >= min_correspondence_ratio;
    result.reason = result.valid ? "registration_accepted" :
                                   "registration_quality_rejected";
    return result;
}

RobustMapLocalizer::RegistrationResult
RobustMapLocalizer::attemptRelocalization(
    const LiveFrame &latest,
    const pcl::PointCloud<PointType>::Ptr &source)
{
    RegistrationResult best;
    if (!source || source->size() < 50)
    {
        best.reason = "relocalization_source_too_small";
        return best;
    }

    PosePcd query_pose;
    query_pose.idx_ = static_cast<int>(latest.sequence);
    query_pose.timestamp_ = latest.stamp.toSec();
    query_pose.pcd_ = *source;
    query_pose.bev_frame.bev_img = getBEVImageParallelAndSave(
        query_pose, bev_threads_, "", true, bev_range_, bev_resolution_,
        bev_downsample_enable_, bev_voxel_size_, false,
        bev_z_max_filter_enable_, bev_z_max_);
    query_pose.bev_frame.header.stamp = latest.stamp;
    query_pose.bev_frame.header.frame_id = body_frame_;
    try
    {
        bev_manager_->detectBEVFeatures(query_pose.bev_frame, false);
    }
    catch (const std::exception &error)
    {
        best.reason = std::string("query_descriptor_failed: ") + error.what();
        publishRelocalizationMatch(
            makeRelocalizationMatchImage(query_pose.bev_frame, BEVFrame(),
                                         -1, 0),
            latest.stamp);
        return best;
    }

    // Retrieve the unfiltered top-k first. This mirrors the reference
    // visualization behavior: even when Top1 is over the descriptor threshold,
    // RViz still shows why the attempt was rejected. Acceptance remains gated
    // by descriptor_max_distance_ below.
    const auto ranked_candidates =
        bev_manager_->detectGlobalRelocalizationCandidates(
            query_pose.bev_frame, relocalization_top_k_,
            std::numeric_limits<float>::infinity());
    if (ranked_candidates.empty())
    {
        best.reason = "no_lighterbev_candidate";
        publishRelocalizationMatch(
            makeRelocalizationMatchImage(query_pose.bev_frame, BEVFrame(),
                                         -1, 0),
            latest.stamp);
        return best;
    }

    cv::Mat top1_visualization;
    cv::Mat best_registration_visualization;
    double best_visualized_score = std::numeric_limits<double>::infinity();
    bool has_descriptor_candidate = false;
    const auto registration_source =
        downsample(source, registration_voxel_size_);
    for (size_t rank = 0; rank < ranked_candidates.size(); ++rank)
    {
        const auto &candidate = ranked_candidates[rank];
        const int index = candidate.first;
        if (index < 0 || index >= static_cast<int>(map_keyframes_.size()))
            continue;
        if (rank > 0 && candidate.second > descriptor_max_distance_)
            continue;
        if (candidate.second <= descriptor_max_distance_)
            has_descriptor_candidate = true;
        BEVFrame query_bev = query_pose.bev_frame.clone();
        BEVFrame reference_bev = map_keyframes_[index].bev.clone();
        Eigen::Matrix4d reference_from_query = Eigen::Matrix4d::Identity();
        int inliers = 0;
        try
        {
            std::tie(reference_from_query, inliers) =
                bev_manager_->poseEstimation(query_bev, reference_bev);
        }
        catch (const std::exception &error)
        {
            ROS_WARN("[Localization] LighterBEV pose candidate %d failed: %s",
                     index, error.what());
            if (rank == 0)
                top1_visualization = makeRelocalizationMatchImage(
                    query_bev, reference_bev, index, 0);
            continue;
        }
        const double lighterbev_translation =
            reference_from_query.block<3, 1>(0, 3).norm();
        ROS_INFO_STREAM(
            std::fixed << std::setprecision(6)
            << "[LocalizationReloc] lighterbev_candidate"
            << " stamp=" << latest.stamp.toSec()
            << " rank=" << rank << " keyframe=" << index
            << " descriptor=" << candidate.second
            << " inliers=" << inliers
            << " bev_dtrans=" << lighterbev_translation
            << " bev_drot_deg=" << rotationAngleDeg(reference_from_query));
        const cv::Mat candidate_visualization =
            makeRelocalizationMatchImage(query_bev, reference_bev,
                                         index, inliers);
        if (rank == 0)
            top1_visualization = candidate_visualization;
        if (candidate.second > descriptor_max_distance_)
            continue;
        if (inliers < lighterbev_min_inliers_)
            continue;

        const Eigen::Matrix4d initial_map_from_body =
            map_keyframes_[index].map_from_body * reference_from_query;
        const auto target = buildMapSubmap(
            map_keyframes_[index].map_from_body.block<3, 1>(0, 3), index,
            true);
        NanoGICP registration;
        configureRegistration(registration, gicp_threads_,
                              gicp_correspondences_, gicp_max_iterations_ + 10,
                              gicp_max_correspondence_distance_ * 1.5,
                              gicp_covariance_threads_);
        RegistrationResult result = registerCloud(
            registration, registration_source, target, initial_map_from_body,
            relocalization_max_score_,
            relocalization_min_correspondence_ratio_, false);
        result.matched_keyframe = index;
        result.lighterbev_inliers = inliers;
        result.descriptor_distance = candidate.second;
        ROS_INFO_STREAM(
            std::fixed << std::setprecision(6)
            << "[LocalizationReloc] gicp_candidate"
            << " stamp=" << latest.stamp.toSec()
            << " rank=" << rank << " keyframe=" << index
            << " descriptor=" << candidate.second
            << " inliers=" << inliers
            << " valid=" << result.valid
            << " converged=" << result.converged
            << " score=" << result.score
            << " correspondence_ratio=" << result.correspondence_ratio
            << " mean_residual=" << result.mean_residual
            << " gicp_innovation_m=" << result.innovation_translation
            << " gicp_innovation_deg=" << result.innovation_rotation_deg
            << " map_body_xyz=" << result.map_from_body(0, 3) << ","
            << result.map_from_body(1, 3) << ","
            << result.map_from_body(2, 3));
        if (std::isfinite(result.score) &&
            result.score < best_visualized_score)
        {
            best_visualized_score = result.score;
            best_registration_visualization = candidate_visualization;
        }
        if (result.valid && (!best.valid || result.score < best.score))
            best = result;
    }
    publishRelocalizationMatch(
        best_registration_visualization.empty() ? top1_visualization :
                                                  best_registration_visualization,
        latest.stamp);
    if (!best.valid && best.reason.empty())
        best.reason = has_descriptor_candidate ?
            "all_relocalization_candidates_rejected" :
            "no_lighterbev_candidate";
    return best;
}

void RobustMapLocalizer::processTracking(
    const LiveFrame &latest,
    const pcl::PointCloud<PointType>::Ptr &source)
{
    const auto registration_source =
        downsample(source, registration_voxel_size_);
    Eigen::Matrix4d current_map_from_odom;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        current_map_from_odom = map_from_odom_;
    }
    const Eigen::Matrix4d predicted_map_from_body =
        current_map_from_odom * latest.odom_from_body;
    bool outside_map_this_cycle = false;
    if (!frontend_healthy_)
    {
        ++consecutive_tracking_failures_;
        status_.reason = "pointlio_motion_jump";
    }
    else if (!refreshContinuousTarget(predicted_map_from_body))
    {
        ++consecutive_tracking_failures_;
        outside_map_this_cycle = true;
        status_.reason = "local_map_unavailable";
    }
    else
    {
        ROS_INFO_THROTTLE(
            1.0,
            "[LocalizerMap] update_ms=%.1f incremental=%d selection=%zu "
            "active_points=%zu stored_points=%zu delete_ms=%.1f",
            last_target_update_ms_,
            continuous_target_incremental_ ? 1 : 0,
            continuous_target_keyframes_.size(),
            continuous_gicp_->activeTargetPoints(),
            continuous_gicp_->storedTargetPoints(),
            last_target_delete_ms_);
        RegistrationResult result = registerCloud(
            *continuous_gicp_, registration_source, continuous_target_,
            predicted_map_from_body, continuous_max_score_,
            continuous_min_correspondence_ratio_, true);
        status_.score = result.score;
        status_.correspondence_ratio = result.correspondence_ratio;
        status_.mean_residual = result.mean_residual;
        status_.matched_keyframe = -1;
        if (result.valid &&
            result.innovation_translation <=
                continuous_max_translation_update_ &&
            result.innovation_rotation_deg <=
                continuous_max_rotation_update_deg_)
        {
            const Eigen::Matrix4d candidate_map_from_odom =
                result.map_from_body * latest.odom_from_body.inverse();
            applyMapToOdom(candidate_map_from_odom, false,
                           "continuous_tracking");
            consecutive_tracking_failures_ = 0;
            status_.reason = "continuous_tracking";
        }
        else
        {
            ++consecutive_tracking_failures_;
            status_.reason = result.valid ? "continuous_innovation_rejected" :
                                            result.reason;
        }
    }

    if (consecutive_tracking_failures_ >= tracking_failure_limit_)
    {
        state_ = State::DEGRADED;
        // Only a missing prior-map submap is treated as out-of-map recovery.
        // Frontend jumps, bad registration and global inconsistency must keep
        // the fast recovery cadence.
        recovery_outside_map_ = outside_map_this_cycle;
        pending_hypothesis_valid_ = false;
        relocalization_confirmations_ = 0;
        status_.reason = recovery_outside_map_ ?
            "outside_map_slow_recovery" :
            "tracking_lost_request_lighterbev";
    }
}

bool RobustMapLocalizer::observeRelocalizationHypothesis(
    const RegistrationResult &result,
    const Eigen::Matrix4d &odom_from_body,
    bool recovery)
{
    if (!result.valid)
    {
        status_.reason = result.reason;
        return false;
    }
    const Eigen::Matrix4d candidate =
        result.map_from_body * odom_from_body.inverse();
    double current_delta_translation = 0.0;
    double current_delta_rotation_deg = 0.0;
    if (has_map_from_odom_)
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        const Eigen::Matrix4d delta = map_from_odom_.inverse() * candidate;
        current_delta_translation = delta.block<3, 1>(0, 3).norm();
        current_delta_rotation_deg = rotationAngleDeg(delta);
    }
    ROS_WARN_STREAM(
        std::fixed << std::setprecision(6)
        << "[LocalizationReloc] hypothesis"
        << " recovery=" << recovery
        << " keyframe=" << result.matched_keyframe
        << " descriptor=" << result.descriptor_distance
        << " inliers=" << result.lighterbev_inliers
        << " score=" << result.score
        << " correspondence_ratio=" << result.correspondence_ratio
        << " current_dtrans=" << current_delta_translation
        << " current_drot_deg=" << current_delta_rotation_deg
        << " confirmations_before=" << relocalization_confirmations_);
    if (!pending_hypothesis_valid_)
    {
        pending_map_from_odom_ = candidate;
        pending_hypothesis_valid_ = true;
        relocalization_confirmations_ = 1;
    }
    else
    {
        const Eigen::Matrix4d delta =
            pending_map_from_odom_.inverse() * candidate;
        if (delta.block<3, 1>(0, 3).norm() <=
                hypothesis_max_translation_delta_ &&
            rotationAngleDeg(delta) <= hypothesis_max_rotation_delta_deg_)
        {
            pending_map_from_odom_ = interpolateTransform(
                pending_map_from_odom_, candidate, 0.5);
            ++relocalization_confirmations_;
        }
        else
        {
            pending_map_from_odom_ = candidate;
            relocalization_confirmations_ = 1;
        }
    }

    status_.score = result.score;
    status_.correspondence_ratio = result.correspondence_ratio;
    status_.mean_residual = result.mean_residual;
    status_.matched_keyframe = result.matched_keyframe;
    status_.reason = "relocalization_confirmation_pending";
    if (relocalization_confirmations_ <
        relocalization_required_confirmations_)
        return false;

    applyMapToOdom(pending_map_from_odom_, true,
                   recovery ? "global_recovery_accepted" :
                              "startup_relocalization_accepted");
    has_map_from_odom_ = true;
    state_ = State::TRACKING;
    consecutive_tracking_failures_ = 0;
    recovery_outside_map_ = false;
    pending_hypothesis_valid_ = false;
    relocalization_confirmations_ = 0;
    continuous_target_->clear();
    continuous_gicp_->clearTargetIkd();
    continuous_target_keyframes_.clear();
    continuous_target_updates_since_build_ = 0;
    continuous_target_center_ =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    last_recovery_stamp_ = odom_from_body.allFinite() ?
        last_accepted_input_stamp_ : -1.0;
    // Do not run the slow global consistency probe immediately after a
    // confirmed startup/recovery result. Give continuous registration one
    // full probe interval to establish its local track first.
    last_global_probe_stamp_ = last_accepted_input_stamp_;
    if (recovery && reset_pointlio_local_map_on_recovery_)
        requestPointLioLocalMapReset();
    return true;
}

void RobustMapLocalizer::applyMapToOdom(
    const Eigen::Matrix4d &candidate, bool hard_reset,
    const std::string &reason)
{
    if (!finiteTransform(candidate))
        return;
    std::lock_guard<std::mutex> lock(transform_mutex_);
    const bool had_transform = has_map_from_odom_;
    const Eigen::Matrix4d previous = map_from_odom_;
    double requested_translation = 0.0;
    double requested_rotation_deg = 0.0;
    if (had_transform)
    {
        const Eigen::Matrix4d requested_delta = previous.inverse() * candidate;
        requested_translation =
            requested_delta.block<3, 1>(0, 3).norm();
        requested_rotation_deg = rotationAngleDeg(requested_delta);
    }
    if (!has_map_from_odom_ || hard_reset)
    {
        if (has_map_from_odom_)
        {
            const Eigen::Matrix4d delta = map_from_odom_.inverse() * candidate;
            status_.correction_translation =
                delta.block<3, 1>(0, 3).norm();
            status_.correction_rotation_deg = rotationAngleDeg(delta);
        }
        map_from_odom_ = candidate;
    }
    else
    {
        const Eigen::Matrix4d delta = map_from_odom_.inverse() * candidate;
        status_.correction_translation = delta.block<3, 1>(0, 3).norm();
        status_.correction_rotation_deg = rotationAngleDeg(delta);
        map_from_odom_ = interpolateTransform(
            map_from_odom_, candidate, correction_smoothing_alpha_);
    }
    has_map_from_odom_ = true;
    status_.reason = reason;
    const Eigen::Matrix4d applied_delta = previous.inverse() * map_from_odom_;
    last_map_correction_translation_ = had_transform ?
        applied_delta.block<3, 1>(0, 3).norm() : 0.0;
    last_map_correction_rotation_deg_ = had_transform ?
        rotationAngleDeg(applied_delta) : 0.0;
    last_map_correction_reason_ = reason;
    last_map_correction_hard_reset_ = hard_reset;
    ++map_correction_sequence_;
    if (had_transform &&
        (hard_reset ||
         last_map_correction_translation_ >= pose_jump_log_translation_m_ ||
         last_map_correction_rotation_deg_ >= pose_jump_log_rotation_deg_))
    {
        ROS_WARN_STREAM(
            std::fixed << std::setprecision(6)
            << "[LocalizationCorrection] applied"
            << " seq=" << map_correction_sequence_
            << " reason=" << reason
            << " hard_reset=" << hard_reset
            << " requested_dtrans=" << requested_translation
            << " requested_drot_deg=" << requested_rotation_deg
            << " applied_dtrans=" << last_map_correction_translation_
            << " applied_drot_deg=" << last_map_correction_rotation_deg_
            << " score=" << status_.score
            << " correspondence_ratio=" << status_.correspondence_ratio
            << " mean_residual=" << status_.mean_residual
            << " keyframe=" << status_.matched_keyframe);
    }
}

void RobustMapLocalizer::recoveryCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    if (!cloud_msg)
        return;

    RecoveryFrame frame;
    frame.stamp = cloud_msg->header.stamp;
    pcl::PointCloud<PointType>::Ptr cloud(
        new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (static_cast<int>(cloud->size()) < recovery_min_points_)
        return;
    frame.points_body = cloud;

    std::lock_guard<std::mutex> lock(recovery_frame_mutex_);
    frame.sequence = next_recovery_sequence_++;
    latest_recovery_frame_ = std::move(frame);
    has_recovery_frame_ = true;
}

void RobustMapLocalizer::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
{
    if (!initialpose_enabled_ || !pose_msg)
        return;
    if (!map_loaded_)
    {
        ROS_ERROR("[LocalizationInitialPose] rejected: prior map is not loaded");
        return;
    }

    const auto canonical_frame = [](std::string frame) {
        while (!frame.empty() && frame.front() == '/')
            frame.erase(frame.begin());
        return frame;
    };
    const std::string request_frame =
        canonical_frame(pose_msg->header.frame_id);
    if (!request_frame.empty() &&
        request_frame != canonical_frame(map_frame_))
    {
        ROS_ERROR("[LocalizationInitialPose] rejected: frame '%s' must be '%s'",
                  pose_msg->header.frame_id.c_str(), map_frame_.c_str());
        return;
    }

    const auto &position = pose_msg->pose.pose.position;
    const auto &orientation = pose_msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
        !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
        ROS_ERROR("[LocalizationInitialPose] rejected: non-finite pose");
        return;
    }
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    if (quaternion.norm() < 1.0e-9)
    {
        ROS_ERROR("[LocalizationInitialPose] rejected: invalid quaternion");
        return;
    }
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));

    Eigen::Matrix4d map_from_reference = Eigen::Matrix4d::Identity();
    map_from_reference.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    map_from_reference(0, 3) = position.x;
    map_from_reference(1, 3) = position.y;
    map_from_reference(2, 3) = std::isfinite(position.z) ? position.z : 0.0;

    PendingInitialPose request;
    request.received_wall_time = ros::WallTime::now();
    request.message = *pose_msg;
    if (initialpose_reference_is_vehicle_base_)
    {
        request.map_from_vehicle_base = map_from_reference;
    }
    else
    {
        Eigen::Matrix4d body_from_vehicle_base = Eigen::Matrix4d::Identity();
        body_from_vehicle_base.block<3, 1>(0, 3) = -vehicle_base_to_body_;
        request.map_from_vehicle_base =
            map_from_reference * body_from_vehicle_base;
    }
    {
        std::lock_guard<std::mutex> lock(initialpose_mutex_);
        request.sequence = next_initialpose_sequence_++;
        pending_initialpose_ = request;
        has_pending_initialpose_ = true;
    }
    ROS_WARN_STREAM(
        std::fixed << std::setprecision(4)
        << "[LocalizationInitialPose] received"
        << " seq=" << request.sequence
        << " x=" << position.x << " y=" << position.y
        << " yaw_deg=" << yaw * 180.0 / kPi
        << " reference="
        << (initialpose_reference_is_vehicle_base_ ?
                vehicle_base_frame_ : body_frame_)
        << "; waiting for single-frame GICP refinement");
}

bool RobustMapLocalizer::takePendingInitialPose(
    PendingInitialPose &request)
{
    std::lock_guard<std::mutex> lock(initialpose_mutex_);
    if (!has_pending_initialpose_)
        return false;
    request = pending_initialpose_;
    has_pending_initialpose_ = false;
    return true;
}

bool RobustMapLocalizer::estimateRecentOdomMotion(
    double &linear_speed_mps, double &angular_speed_degps)
{
    std::deque<StampedOdom> history;
    {
        std::lock_guard<std::mutex> lock(recent_odom_mutex_);
        history = recent_odom_;
    }
    linear_speed_mps = std::numeric_limits<double>::infinity();
    angular_speed_degps = std::numeric_limits<double>::infinity();
    if (history.size() < 2)
        return false;

    const StampedOdom &latest = history.back();
    for (auto iterator = history.rbegin() + 1;
         iterator != history.rend(); ++iterator)
    {
        const double dt = (latest.stamp - iterator->stamp).toSec();
        if (dt < 0.04)
            continue;
        const Eigen::Matrix4d delta =
            iterator->odom_from_body.inverse() * latest.odom_from_body;
        linear_speed_mps = delta.block<3, 1>(0, 3).norm() / dt;
        angular_speed_degps = rotationAngleDeg(delta) / dt;
        return std::isfinite(linear_speed_mps) &&
               std::isfinite(angular_speed_degps);
    }
    return false;
}

int RobustMapLocalizer::nearestMapKeyframe2d(double x, double y) const
{
    int nearest = -1;
    double best_distance_squared = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < map_keyframes_.size(); ++index)
    {
        const Eigen::Vector3d translation =
            map_keyframes_[index].map_from_body.block<3, 1>(0, 3);
        const double dx = translation.x() - x;
        const double dy = translation.y() - y;
        const double distance_squared = dx * dx + dy * dy;
        if (distance_squared < best_distance_squared)
        {
            best_distance_squared = distance_squared;
            nearest = static_cast<int>(index);
        }
    }
    return nearest;
}

void RobustMapLocalizer::publishRefinedInitialPose(
    const PendingInitialPose &request,
    const Eigen::Matrix4d &map_from_body,
    const ros::Time &stamp)
{
    if (initialpose_refined_pub_.getTopic().empty())
        return;
    Eigen::Matrix4d output_pose = map_from_body;
    if (initialpose_reference_is_vehicle_base_)
    {
        Eigen::Matrix4d body_from_vehicle_base = Eigen::Matrix4d::Identity();
        body_from_vehicle_base.block<3, 1>(0, 3) = -vehicle_base_to_body_;
        output_pose = map_from_body * body_from_vehicle_base;
    }
    const nav_msgs::Odometry converted = matrixToOdometry(
        output_pose, stamp, map_frame_, initialpose_reference_is_vehicle_base_ ?
            vehicle_base_frame_ : body_frame_);
    geometry_msgs::PoseWithCovarianceStamped message = request.message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.pose.pose = converted.pose.pose;
    initialpose_refined_pub_.publish(message);
}

bool RobustMapLocalizer::processInitialPoseRequest(
    const PendingInitialPose &request, const LiveFrame &latest)
{
    const double age_s =
        (ros::WallTime::now() - request.received_wall_time).toSec();
    if (age_s > initialpose_request_timeout_s_)
    {
        status_.reason = "initialpose_rejected_request_timeout";
        ROS_ERROR("[LocalizationInitialPose] rejected seq=%llu: request age %.3f s exceeds %.3f s",
                  static_cast<unsigned long long>(request.sequence), age_s,
                  initialpose_request_timeout_s_);
        return false;
    }

    if (initialpose_require_stationary_)
    {
        double linear_speed = 0.0;
        double angular_speed = 0.0;
        if (!estimateRecentOdomMotion(linear_speed, angular_speed))
        {
            status_.reason = "initialpose_rejected_no_motion_estimate";
            ROS_ERROR("[LocalizationInitialPose] rejected seq=%llu: odometry motion estimate unavailable",
                      static_cast<unsigned long long>(request.sequence));
            return false;
        }
        if (linear_speed > initialpose_stationary_max_speed_mps_ ||
            angular_speed > initialpose_stationary_max_angular_speed_degps_)
        {
            status_.reason = "initialpose_rejected_vehicle_moving";
            ROS_ERROR("[LocalizationInitialPose] rejected seq=%llu: vehicle moving at %.3f m/s, %.2f deg/s",
                      static_cast<unsigned long long>(request.sequence),
                      linear_speed, angular_speed);
            return false;
        }
    }

    pcl::PointCloud<PointType>::Ptr source;
    ros::Time source_stamp = latest.stamp;
    ros::Time matched_odom_stamp = latest.stamp;
    Eigen::Matrix4d source_odom_from_body = latest.odom_from_body;
    double sync_dt = -1.0;
    const bool using_raw_source = use_raw_recovery_cloud_ &&
        getLatestRecoverySource(source, source_stamp, matched_odom_stamp,
                                source_odom_from_body, sync_dt);
    if (!using_raw_source)
        source = latest.points_body;
    if (!source || static_cast<int>(source->size()) < initialpose_min_points_)
    {
        status_.reason = "initialpose_rejected_source_too_small";
        ROS_ERROR("[LocalizationInitialPose] rejected seq=%llu: only %zu source points",
                  static_cast<unsigned long long>(request.sequence),
                  source ? source->size() : 0UL);
        return false;
    }

    Eigen::Matrix4d vehicle_base_from_body = Eigen::Matrix4d::Identity();
    vehicle_base_from_body.block<3, 1>(0, 3) = vehicle_base_to_body_;
    Eigen::Matrix4d initial_map_from_body =
        request.map_from_vehicle_base * vehicle_base_from_body;
    const int nearest_keyframe = nearestMapKeyframe2d(
        initial_map_from_body(0, 3), initial_map_from_body(1, 3));
    if (nearest_keyframe < 0)
    {
        status_.reason = "initialpose_rejected_no_map_keyframe";
        ROS_ERROR("[LocalizationInitialPose] rejected seq=%llu: map has no keyframe",
                  static_cast<unsigned long long>(request.sequence));
        return false;
    }
    // RViz supplies only x/y/yaw. Seed the unobservable 2D height from the
    // nearest map keyframe and let GICP refine the complete SE(3) pose.
    initial_map_from_body(2, 3) =
        map_keyframes_[nearest_keyframe].map_from_body(2, 3);

    const auto target = buildMapSubmap(
        initial_map_from_body.block<3, 1>(0, 3), -1, false);
    const auto registration_source =
        downsample(source, registration_voxel_size_);
    NanoGICP registration;
    configureRegistration(registration, gicp_threads_,
                          gicp_correspondences_,
                          initialpose_gicp_max_iterations_,
                          initialpose_gicp_max_correspondence_distance_,
                          gicp_covariance_threads_);
    RegistrationResult result = registerCloud(
        registration, registration_source, target, initial_map_from_body,
        initialpose_max_score_, initialpose_min_correspondence_ratio_, false);
    result.matched_keyframe = nearest_keyframe;
    if (result.valid &&
        (result.innovation_translation >
             initialpose_max_translation_correction_m_ ||
         result.innovation_rotation_deg >
             initialpose_max_rotation_correction_deg_))
    {
        result.valid = false;
        result.reason = "initialpose_correction_limit_rejected";
    }

    status_.score = result.score;
    status_.correspondence_ratio = result.correspondence_ratio;
    status_.mean_residual = result.mean_residual;
    status_.matched_keyframe = result.matched_keyframe;
    ROS_WARN_STREAM(
        std::fixed << std::setprecision(6)
        << "[LocalizationInitialPose] refinement"
        << " seq=" << request.sequence
        << " source=" << (using_raw_source ? "raw_single_frame" :
                                                 "deskewed_single_frame")
        << " source_points=" << source->size()
        << " target_points=" << (target ? target->size() : 0)
        << " nearest_keyframe=" << nearest_keyframe
        << " valid=" << result.valid
        << " converged=" << result.converged
        << " score=" << result.score
        << " correspondence_ratio=" << result.correspondence_ratio
        << " mean_residual=" << result.mean_residual
        << " correction_m=" << result.innovation_translation
        << " correction_deg=" << result.innovation_rotation_deg
        << " sync_dt=" << sync_dt
        << " reason=" << result.reason);
    if (!result.valid)
    {
        status_.reason = result.reason.empty() ?
            "initialpose_gicp_rejected" : result.reason;
        return false;
    }

    const Eigen::Matrix4d candidate_map_from_odom =
        result.map_from_body * source_odom_from_body.inverse();
    applyMapToOdom(candidate_map_from_odom, true,
                   "rviz_initialpose_gicp_accepted");
    state_ = State::TRACKING;
    consecutive_tracking_failures_ = 0;
    recovery_outside_map_ = false;
    pending_hypothesis_valid_ = false;
    relocalization_confirmations_ = 0;
    continuous_target_->clear();
    continuous_gicp_->clearTargetIkd();
    continuous_target_keyframes_.clear();
    continuous_target_updates_since_build_ = 0;
    continuous_target_center_ =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    last_recovery_stamp_ = source_stamp.toSec();
    last_global_probe_stamp_ = source_stamp.toSec();
    status_.reason = "rviz_initialpose_gicp_accepted";
    publishRefinedInitialPose(request, result.map_from_body, source_stamp);
    ROS_WARN("[LocalizationInitialPose] accepted seq=%llu; map->odom updated without resetting Point-LIO odometry",
             static_cast<unsigned long long>(request.sequence));
    return true;
}

bool RobustMapLocalizer::getLatestRecoverySource(
    pcl::PointCloud<PointType>::Ptr &points,
    ros::Time &raw_stamp,
    ros::Time &matched_odom_stamp,
    Eigen::Matrix4d &matched_odom_from_body,
    double &sync_dt)
{
    RecoveryFrame frame;
    std::deque<StampedOdom> odom_history;
    {
        std::lock_guard<std::mutex> lock(recovery_frame_mutex_);
        if (!has_recovery_frame_)
            return false;
        frame = latest_recovery_frame_;
    }
    {
        std::lock_guard<std::mutex> lock(recent_odom_mutex_);
        odom_history = recent_odom_;
    }

    if (odom_history.empty())
        return false;

    const double raw_sec = frame.stamp.toSec();
    const double newest_odom_sec = odom_history.back().stamp.toSec();
    if (raw_sec > newest_odom_sec + recovery_odom_sync_tolerance_s_ ||
        newest_odom_sec - raw_sec > recovery_cloud_max_age_s_)
        return false;

    const StampedOdom *best = &odom_history.front();
    double best_dt = std::abs(best->stamp.toSec() - raw_sec);
    for (const auto &sample : odom_history)
    {
        const double dt = std::abs(sample.stamp.toSec() - raw_sec);
        if (dt < best_dt)
        {
            best_dt = dt;
            best = &sample;
        }
    }
    if (best_dt > recovery_odom_sync_tolerance_s_)
        return false;

    points = frame.points_body;
    raw_stamp = frame.stamp;
    matched_odom_stamp = best->stamp;
    matched_odom_from_body = best->odom_from_body;
    sync_dt = best_dt;
    // Prefer an SE(3) interpolation when the raw scan lies between two
    // sufficiently close odometry samples. This makes the pose and scan share
    // the exact timestamp instead of accepting up to 50 ms of motion error.
    for (std::size_t i = 1; i < odom_history.size(); ++i)
    {
        const auto &before = odom_history[i - 1];
        const auto &after = odom_history[i];
        const double t0 = before.stamp.toSec();
        const double t1 = after.stamp.toSec();
        if (t0 <= raw_sec && raw_sec <= t1 && t1 > t0 &&
            t1 - t0 <= 2.0 * recovery_odom_sync_tolerance_s_)
        {
            matched_odom_from_body = interpolateTransform(
                before.odom_from_body, after.odom_from_body,
                (raw_sec - t0) / (t1 - t0));
            matched_odom_stamp = frame.stamp;
            sync_dt = 0.0;
            break;
        }
    }
    return true;
}

void RobustMapLocalizer::backendTimerCallback(const ros::TimerEvent &)
{
    std::unique_lock<std::mutex> processing_lock(processing_mutex_,
                                                  std::try_to_lock);
    if (!processing_lock.owns_lock())
        return;

    std::vector<LiveFrame> frames;
    LiveFrame latest;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (!live_frames_.empty())
        {
            frames.assign(live_frames_.begin(), live_frames_.end());
            latest = live_frames_.back();
        }
    }
    if (frames.empty())
    {
        publishStatus(ros::Time::now());
        return;
    }

    PendingInitialPose initialpose_request;
    const bool has_initialpose_request =
        takePendingInitialPose(initialpose_request);

    // map->odom is deliberately published only by controlOdomCallback(), at
    // the accepted PointLIO measurement stamp. The backend only updates the
    // cached correction so slow registration/relocalization work cannot mix
    // wall-clock TF stamps with delayed estimator stamps. A new RViz initial
    // pose is allowed to reuse the latest single frame even if no newer scan
    // arrived after the click.
    if (latest.sequence == last_processed_sequence_ &&
        !has_initialpose_request)
    {
        publishStatus(latest.stamp);
        return;
    }
    if (latest.sequence != last_processed_sequence_)
        last_processed_sequence_ = latest.sequence;
    const auto source = buildAccumulatedSource(frames, latest);
    const double stamp = latest.stamp.toSec();

    if (has_initialpose_request)
    {
        processInitialPoseRequest(initialpose_request, latest);
        publishAlignedScan(latest);
        publishStatus(latest.stamp);
        return;
    }

    const bool forced_relocalization = force_relocalization_.exchange(false);
    if (forced_relocalization && map_loaded_)
    {
        state_ = State::DEGRADED;
        recovery_outside_map_ = false;
        pending_hypothesis_valid_ = false;
        relocalization_confirmations_ = 0;
        last_relocalization_attempt_stamp_ = -1.0;
        status_.reason = "manual_relocalization_requested";
    }

    if (!map_loaded_)
    {
        state_ = State::WAITING_FOR_MAP;
        status_.reason = "map_not_loaded";
    }
    else if (state_ == State::RELOCALIZING || state_ == State::DEGRADED)
    {
        const bool recovery = has_map_from_odom_;
        const double retry_sec = recovery && recovery_outside_map_ ?
            outside_map_relocalization_retry_sec_ : relocalization_retry_sec_;
        if (last_relocalization_attempt_stamp_ < 0.0 ||
            stamp < last_relocalization_attempt_stamp_ ||
            stamp - last_relocalization_attempt_stamp_ >=
                retry_sec)
        {
            last_relocalization_attempt_stamp_ = stamp;
            pcl::PointCloud<PointType>::Ptr recovery_points;
            ros::Time raw_stamp, matched_odom_stamp;
            Eigen::Matrix4d matched_odom_from_body =
                Eigen::Matrix4d::Identity();
            double sync_dt = -1.0;
            const bool have_raw =
                use_raw_recovery_cloud_ &&
                getLatestRecoverySource(recovery_points, raw_stamp,
                                        matched_odom_stamp,
                                        matched_odom_from_body, sync_dt);
            RegistrationResult result;
            const LiveFrame *hypothesis_frame = &latest;
            LiveFrame raw_frame;
            if (have_raw)
            {
                ROS_INFO(
                    "[LocalizationReloc] source=raw_body raw_stamp=%.6f "
                    "matched_odom_stamp=%.6f sync_dt=%.4f",
                    raw_stamp.toSec(), matched_odom_stamp.toSec(), sync_dt);
                raw_frame = latest;
                raw_frame.stamp = raw_stamp;
                raw_frame.odom_from_body = matched_odom_from_body;
                raw_frame.points_body = recovery_points;
                hypothesis_frame = &raw_frame;
                result = attemptRelocalization(raw_frame, recovery_points);
            }
            else
            {
                // Fallback: latest single deskewed body scan. Still pose
                // independent with respect to multi-frame accumulation, but
                // it does depend on this frame's frontend deskew/pose.
                ROS_INFO("[LocalizationReloc] source=latest_body");
                result = attemptRelocalization(latest, latest.points_body);
            }
            observeRelocalizationHypothesis(result,
                                            hypothesis_frame->odom_from_body,
                                            recovery);
        }
    }
    else if (state_ == State::TRACKING)
    {
        const bool settling = last_recovery_stamp_ >= 0.0 &&
                              stamp >= last_recovery_stamp_ &&
                              stamp - last_recovery_stamp_ <
                                  recovery_settle_sec_;
        if (!settling)
            processTracking(latest, source);

        const bool global_probe_due = global_probe_interval_sec_ > 0.0 &&
            (last_global_probe_stamp_ < 0.0 ||
             stamp < last_global_probe_stamp_ ||
             stamp - last_global_probe_stamp_ >= global_probe_interval_sec_);
        if (global_probe_due && state_ == State::TRACKING)
        {
            global_probe_active_ = true;
            last_global_probe_stamp_ = stamp;
            RegistrationResult result =
                attemptRelocalization(latest, latest.points_body);
            if (result.valid)
            {
                Eigen::Matrix4d current;
                {
                    std::lock_guard<std::mutex> lock(transform_mutex_);
                    current = map_from_odom_;
                }
                const Eigen::Matrix4d candidate =
                    result.map_from_body * latest.odom_from_body.inverse();
                const Eigen::Matrix4d delta = current.inverse() * candidate;
                const bool severe =
                    delta.block<3, 1>(0, 3).norm() >
                        severe_global_translation_error_ ||
                    rotationAngleDeg(delta) > severe_global_rotation_error_deg_;
                if (severe)
                {
                    state_ = State::DEGRADED;
                    recovery_outside_map_ = false;
                    observeRelocalizationHypothesis(result,
                                                    latest.odom_from_body,
                                                    true);
                }
                else
                {
                    pending_hypothesis_valid_ = false;
                    relocalization_confirmations_ = 0;
                    status_.reason = "global_probe_consistent";
                }
            }
            global_probe_active_ = false;
        }
    }

    publishAlignedScan(latest);
    publishStatus(latest.stamp);
}

bool RobustMapLocalizer::forceRelocalizationService(
    std_srvs::Trigger::Request &, std_srvs::Trigger::Response &response)
{
    if (!map_loaded_)
    {
        response.success = false;
        response.message = "prior map is not loaded";
        return true;
    }
    force_relocalization_.store(true);
    response.success = true;
    response.message = "LighterBEV global relocalization requested";
    return true;
}

void RobustMapLocalizer::publishMapToOdom(const ros::Time &stamp)
{
    Eigen::Matrix4d transform;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (!has_map_from_odom_)
            return;
        transform = map_from_odom_;
    }
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        poseEigToROSTf(transform), stamp, map_frame_, odom_frame_));
}

void RobustMapLocalizer::publishAlignedScan(const LiveFrame &frame)
{
    Eigen::Matrix4d map_from_odom;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (!has_map_from_odom_)
            return;
        map_from_odom = map_from_odom_;
    }
    const Eigen::Matrix4d map_from_body =
        map_from_odom * frame.odom_from_body;
    pcl::PointCloud<PointType> aligned;
    pcl::transformPointCloud(*frame.points_body, aligned,
                             map_from_body.cast<float>());
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.stamp = frame.stamp;
    message.header.frame_id = map_frame_;
    corrected_cloud_pub_.publish(message);
}

void RobustMapLocalizer::publishStatus(const ros::Time &stamp)
{
    point_lio_sam_lighterbev::LocalizationStatus message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.state = static_cast<uint8_t>(state_);
    message.map_loaded = map_loaded_;
    message.localized = has_map_from_odom_;
    message.frontend_healthy = frontend_healthy_;
    message.global_probe_active = global_probe_active_;
    message.registration_score = status_.score;
    message.correspondence_ratio = status_.correspondence_ratio;
    message.mean_residual = status_.mean_residual;
    message.map_odom_translation_update = status_.correction_translation;
    message.map_odom_rotation_update_deg = status_.correction_rotation_deg;
    message.consecutive_failures = consecutive_tracking_failures_;
    message.relocalization_confirmations = relocalization_confirmations_;
    message.matched_keyframe = status_.matched_keyframe;
    message.reason = status_.reason;
    status_pub_.publish(message);
}

void RobustMapLocalizer::publishPriorMap()
{
    if (!prior_map_ || prior_map_->empty())
        return;
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*prior_map_, message);
    message.header.stamp = ros::Time::now();
    message.header.frame_id = map_frame_;
    prior_map_pub_.publish(message);
}

void RobustMapLocalizer::publishRelocalizationMatch(
    const cv::Mat &image, const ros::Time &stamp)
{
    if (!lighterbev_viz_enable_ || image.empty() ||
        relocalization_match_pub_.getTopic().empty())
        return;
    std_msgs::Header header;
    header.stamp = stamp.isZero() ? ros::Time::now() : stamp;
    header.frame_id = map_frame_;
    relocalization_match_pub_.publish(
        cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
}

bool RobustMapLocalizer::requestPointLioLocalMapReset()
{
    std_srvs::Trigger service;
    if (!pointlio_reset_client_.exists())
    {
        ROS_WARN("[Localization] /pointlio/reset_local_map unavailable; map->odom recovery still applied.");
        return false;
    }
    if (!pointlio_reset_client_.call(service) || !service.response.success)
    {
        ROS_WARN("[Localization] PointLIO local-map reset failed: %s",
                 service.response.message.c_str());
        return false;
    }
    ROS_WARN("[Localization] %s", service.response.message.c_str());
    return true;
}

Eigen::Matrix4d RobustMapLocalizer::odometryToMatrix(
    const nav_msgs::Odometry &message)
{
    const auto &orientation = message.pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
    pose(0, 3) = message.pose.pose.position.x;
    pose(1, 3) = message.pose.pose.position.y;
    pose(2, 3) = message.pose.pose.position.z;
    return pose;
}

nav_msgs::Odometry RobustMapLocalizer::matrixToOdometry(
    const Eigen::Matrix4d &pose, const ros::Time &stamp,
    const std::string &frame, const std::string &child)
{
    nav_msgs::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = frame;
    message.child_frame_id = child;
    const Eigen::Quaterniond quaternion(pose.block<3, 3>(0, 0));
    message.pose.pose.position.x = pose(0, 3);
    message.pose.pose.position.y = pose(1, 3);
    message.pose.pose.position.z = pose(2, 3);
    message.pose.pose.orientation.x = quaternion.x();
    message.pose.pose.orientation.y = quaternion.y();
    message.pose.pose.orientation.z = quaternion.z();
    message.pose.pose.orientation.w = quaternion.w();
    return message;
}

double RobustMapLocalizer::rotationAngleDeg(
    const Eigen::Matrix4d &transform)
{
    const double cosine = std::max(-1.0, std::min(
        1.0, 0.5 * (transform.block<3, 3>(0, 0).trace() - 1.0)));
    return std::acos(cosine) * 180.0 / kPi;
}

Eigen::Matrix4d RobustMapLocalizer::interpolateTransform(
    const Eigen::Matrix4d &from, const Eigen::Matrix4d &to,
    double ratio)
{
    ratio = std::max(0.0, std::min(1.0, ratio));
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    const Eigen::Quaterniond from_rotation(from.block<3, 3>(0, 0));
    const Eigen::Quaterniond to_rotation(to.block<3, 3>(0, 0));
    result.block<3, 3>(0, 0) =
        from_rotation.slerp(ratio, to_rotation).normalized().toRotationMatrix();
    result.block<3, 1>(0, 3) =
        (1.0 - ratio) * from.block<3, 1>(0, 3) +
        ratio * to.block<3, 1>(0, 3);
    return result;
}

pcl::PointCloud<PointType>::Ptr RobustMapLocalizer::downsample(
    const pcl::PointCloud<PointType>::Ptr &cloud, double voxel_size)
{
    if (!cloud || cloud->empty() || voxel_size <= 0.0)
        return cloud;
    pcl::VoxelGrid<PointType> filter;
    filter.setLeafSize(static_cast<float>(voxel_size),
                       static_cast<float>(voxel_size),
                       static_cast<float>(voxel_size));
    filter.setInputCloud(cloud);
    pcl::PointCloud<PointType>::Ptr output(new pcl::PointCloud<PointType>());
    filter.filter(*output);
    return output;
}
