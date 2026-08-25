#ifndef POINT_LIO_SAM_LIGHTERBEV_LOOP_CLOSURE_H
#define POINT_LIO_SAM_LIGHTERBEV_LOOP_CLOSURE_H

///// C++ common headers
#include <tuple>
#include <vector>
#include <memory>
#include <limits>
#include <iostream>
#include <cmath>
#include <utility> // pair
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include "pcl/registration/icp.h"
#include "pcl/registration/gicp.h"
///// Eigen
#include <Eigen/Eigen>
///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// coded headers
#include "pose_pcd.hpp"
#include "utilities.hpp"
/// lighterbev
#include "Lighterbev.h"


using PcdPair = std::tuple<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>;

struct NanoGICPConfig
{
    int nano_thread_number_ = 0;
    int nano_correspondences_number_ = 15;
    int nano_max_iter_ = 32;
    int nano_ransac_max_iter_ = 5;
    double max_corr_dist_ = 2.0;
    double icp_score_thr_ = 10.0;
    double transformation_epsilon_ = 0.01;
    double euclidean_fitness_epsilon_ = 0.01;
    double ransac_outlier_rejection_threshold_ = 1.0;
};

struct LighterBEVConfig
{
    std::string model_path;
    double resolution = 0.05;
    int ninliers = 20;                        
    int iterations = 1000;                         
    double threshold = 0.5; //ransac
    double pr_threshold = 0.66; //place recognition
    double match_ratio_test = 0.9;
    int min_matches = 20;
    LighterBEVKeypointConfig keypoint_config;

};

struct ICPConfig
{
    double IcpFitnessScore = 0.3;
    int max_iters = 100;
    int max_correspondences = 150;

};

struct LoopClosureConfig
{
    bool enable_submap_matching_ = true;
    int num_submap_keyframes_ = 10;
    double voxel_res_ = 0.1;
    int key_excludes = 50;
    double time_excludes = 50.0;
    bool use_odom_pose_for_loop_init_ = true;
    
    ICPConfig icp_config_;
    LighterBEVConfig bev_config_;
    NanoGICPConfig gicp_config_;
};

struct RegistrationOutput
{
    bool is_valid_ = false;
    bool is_converged_ = false;
    double score_ = std::numeric_limits<double>::max();
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
};

class LoopClosure
{
private:
    LighterBEVManager bev_manager_;
    nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
    int closest_keyframe_idx_ = -1;
    pcl::PointCloud<PointType>::Ptr src_cloud_;
    pcl::PointCloud<PointType>::Ptr dst_cloud_;
    pcl::PointCloud<PointType> coarse_aligned_;
    pcl::PointCloud<PointType> aligned_;
    LoopClosureConfig config_;
    
public:
    explicit LoopClosure(const LoopClosureConfig &config);
    ~LoopClosure();
    void updateLighterBEV(BEVFrame& frame);

    std::pair<Eigen::Matrix4d, int> fetchCandidateKeyframeIdx(PosePcd &query_keyframe, std::vector<PosePcd> &keyframes);

    PcdPair setSrcAndDstCloud(const std::vector<PosePcd> &keyframes,
                              const int src_idx,
                              const int dst_idx,
                              const int submap_range,
                              const double voxel_res,
                              const bool enable_submap_matching);
    RegistrationOutput icpAlignment(const pcl::PointCloud<PointType> &src,
                                    const pcl::PointCloud<PointType> &dst);

    RegistrationOutput GicpAlignmentwithInit(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst,
                                             const Eigen::Matrix4d &init_reg);

    RegistrationOutput icpAlignmentwithInit(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst,
                                             const Eigen::Matrix4d &init_reg);

    RegistrationOutput performLoopClosure(const PosePcd &query_keyframe,
                                          const std::vector<PosePcd> &keyframes,
                                          const int closest_keyframe_idx);

    RegistrationOutput performLoopClosurev2(const PosePcd& query_keyframe,
                                  const std::vector<PosePcd>& keyframes,
                                  const int                   closest_idx,
                                  const Eigen::Matrix4d&      init_reg);
    
    RegistrationOutput pclIcpAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst);
    RegistrationOutput pclGicpAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst);
    RegistrationOutput pclGicpAlignmentWithInit(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst,
                                             const Eigen::Matrix4d &init_reg);
    RegistrationOutput pclGeneralizedIcpAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst);
    RegistrationOutput pclGeneralizedIcpAlignmentWithInit(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst,
                                             const Eigen::Matrix4d &init_reg);

    pcl::PointCloud<PointType> getSourceCloud();
    pcl::PointCloud<PointType> getTargetCloud();
    pcl::PointCloud<PointType> getCoarseAlignedCloud();
    pcl::PointCloud<PointType> getFinalAlignedCloud();
    int getClosestKeyframeidx();
    bool reloadBEVModel(const std::string &state_dict_path);
};

#endif
