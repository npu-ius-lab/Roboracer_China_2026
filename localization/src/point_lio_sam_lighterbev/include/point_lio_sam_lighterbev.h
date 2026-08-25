#ifndef POINT_LIO_SAM_LIGHTERBEV_MAIN_H
#define POINT_LIO_SAM_LIGHTERBEV_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
#include <tuple>
#include <mutex>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
///// ROS
#include <ros/ros.h>
#include <ros/package.h>              // get package_path
#include <rosbag/bag.h>               // save map
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h>  // to Quaternion_to_euler
#include <tf/transform_datatypes.h>   // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <tf/transform_broadcaster.h> // broadcaster
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/linear/NoiseModel.h>
///// coded headers
#include "loop_closure.h"
#include "pose_pcd.hpp"
#include "utilities.hpp"

///// BEV preprocess
#include "BEVProjector.hpp"
#include "point_lio_sam_lighterbev/BevFrame.h" //msgs
//cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace fs = boost::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class PointLioSamLighterBev
{
private:
    
    ///// basic params
    std::string map_frame_;
    std::string package_path_;
    std::string seq_name_;
    std::string default_save_root_;
    std::string dataset_name_;
    std::string src_dst_save_dir_;
    bool use_loop = true;
    double loop_timeout_ms_ = 0.0;
    double loop_max_score_ = 0.35;
    double loop_noise_variance_min_ = 0.05;
    double loop_noise_variance_max_ = 1.0;
    double loop_huber_k_ = 1.345;
    bool loop_bidirectional_check_ = true;
    double loop_bidirectional_max_translation_error_ = 0.8;
    double loop_bidirectional_max_rotation_error_deg_ = 1.5;
    ///// shared data - odom and pcd
    std::mutex realtime_pose_mutex_, keyframes_mutex_;
    std::mutex save_mutex_;
    std::mutex odom_pcd_callback_mutex_;
    std::mutex graph_mutex_, vis_mutex_, loop_closure_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    int current_keyframe_idx_ = 0;
    ///// graph and values
    bool is_initialized_ = false;
    bool loop_added_flag_ = false;     // for opt
    bool loop_added_flag_vis_ = false; // for vis
    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    double keyframe_thr_;
    double voxel_res_;
    double bev_range_ = 5.0;
    double bev_resolution_ = 0.05;
    std::string bev_input_mode_ = "single_frame";
    bool bev_downsample_enable_ = true;
    double bev_downsample_voxel_size_ = 0.1;
    bool bev_z_max_filter_enable_ = true;
    double bev_z_max_ = 2.0;
    bool bev_save_png_ = false;
    int sub_key_num_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_; // for vis
    std::pair<size_t, size_t> current_loop_idx_pair_;
    
    ///// visualize
    tf::TransformBroadcaster broadcaster_;
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;
    ///// results
    bool save_map_bag_ = false, save_map_pcd_ = false, save_in_kitti_format_ = false;
    bool save_raw_keyframes_on_save_dir_ = true;
    double save_aggregate_voxel_size_ = 0.1;
    int save_aggregate_downsample_interval_ = 20;
    ///// ros
    ros::NodeHandle nh_;
    ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_, path_pub_;
    ros::Publisher corrected_current_pcd_pub_, corrected_pcd_map_pub_, loop_detection_pub_;
    ros::Publisher realtime_pose_pub_;
    ros::Publisher debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    ros::Subscriber sub_save_flag_;
    ros::ServiceServer save_map_service_;
    ros::Timer loop_timer_, vis_timer_;
    /// pub to python train and test use LighterBEV
    ros::Publisher lighter_bev_pub_;
    ros::Publisher loop_img_vis;
    // odom, pcd sync, and save flag subscribers
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_ = nullptr;
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::string odom_topic_ = "/Odometry";
    std::string pcd_topic_ = "/cloud_registered";
    ///// Loop closure
    std::shared_ptr<LoopClosure> loop_closure_;
    ///save dir
    std::string bev_directory;
    std::string bev_output_dir_;
    
public:
    explicit PointLioSamLighterBev(const ros::NodeHandle &n_private);
    ~PointLioSamLighterBev();

private:
    // methods
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    // cb
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                         const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void saveFlagCallback(const std_msgs::String::ConstPtr &msg);
    bool saveMapService(std_srvs::Trigger::Request &request,
                        std_srvs::Trigger::Response &response);
    void loopTimerFunc(const ros::TimerEvent &event);
    void visTimerFunc(const ros::TimerEvent &event);
    void publishBevFrame(const cv::Mat& bev_cv_mat, const std::vector<Eigen::Vector3f>& points,const std::vector<int>& indices, bool correct_frame);
    void saveRawKeyframes(const std::string &raw_keyframes_dir);
    pcl::PointCloud<PointType>::Ptr buildDownsampledAggregatedMap();
};


#endif
