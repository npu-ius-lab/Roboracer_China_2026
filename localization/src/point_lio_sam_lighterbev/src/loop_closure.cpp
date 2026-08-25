#include "loop_closure.h"

LoopClosure::LoopClosure(const LoopClosureConfig &config)
    : bev_manager_(config.bev_config_.model_path),
    config_(config)
{
    config_ = config;
    const auto &gc = config_.gicp_config_;
    bev_manager_.keyframes_to_exclude       = config.key_excludes;     
    bev_manager_.time_to_exclude            = config.time_excludes;

    bev_manager_.pr_threshold       = config.bev_config_.pr_threshold;     
    bev_manager_.max_iterations       = config.bev_config_.iterations;     
    bev_manager_.ransac_threshold       = config.bev_config_.threshold;
    bev_manager_.metric_scale_       = config.bev_config_.resolution;
    bev_manager_.setMatchingParams(static_cast<float>(config.bev_config_.match_ratio_test),
                                   config.bev_config_.min_matches);
    bev_manager_.setKeypointConfig(config.bev_config_.keypoint_config);
    
    ////// nano_gicp init
    nano_gicp_.setNumThreads(gc.nano_thread_number_);
    nano_gicp_.setCorrespondenceRandomness(gc.nano_correspondences_number_);
    nano_gicp_.setMaximumIterations(gc.nano_max_iter_);
    nano_gicp_.setRANSACIterations(gc.nano_ransac_max_iter_);
    nano_gicp_.setMaxCorrespondenceDistance(gc.max_corr_dist_);
    nano_gicp_.setTransformationEpsilon(gc.transformation_epsilon_);
    nano_gicp_.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon_);
    nano_gicp_.setRANSACOutlierRejectionThreshold(gc.ransac_outlier_rejection_threshold_);
    src_cloud_.reset(new pcl::PointCloud<PointType>);
    dst_cloud_.reset(new pcl::PointCloud<PointType>);
    
}

LoopClosure::~LoopClosure() {}

void LoopClosure::updateLighterBEV(BEVFrame& frame){
    bev_manager_.addNewKeyFrame(frame);
}

std::pair<Eigen::Matrix4d, int> LoopClosure::fetchCandidateKeyframeIdx(PosePcd &query_keyframe,
                                           std::vector<PosePcd> &keyframes)
{
    int loop_idx = bev_manager_.detectLoopClosureIDGivenScan(query_keyframe.bev_frame);
    Eigen::Matrix4d T4d = Eigen::Matrix4d::Identity();

    if (loop_idx >= 0)
    {
        // perform pose estimation
        std::pair<Eigen::Matrix4d, int> pose_pair = bev_manager_.poseEstimation(query_keyframe.bev_frame, keyframes[loop_idx].bev_frame);

        T4d = pose_pair.first;
        int ninliers         = pose_pair.second;
        if (ninliers < config_.bev_config_.ninliers)
        {
            std::cout << "Loop closure not valid: query keyframe " << query_keyframe.idx_
                      << ", matched keyframe " << loop_idx
                      << ", inliers " << ninliers << " < "
                      << config_.bev_config_.ninliers << std::endl;
            return {T4d, -1};
        }
        std::cout << "Loop closure found query keyframe: "<< query_keyframe.idx_ << "matched keyframe: "  << loop_idx << " number of ninliers " << ninliers << std::endl;
    }


    return {T4d, loop_idx};
}

PcdPair LoopClosure::setSrcAndDstCloud(const std::vector<PosePcd> &keyframes,
                                       const int src_idx,
                                       const int dst_idx,
                                       const int submap_range,
                                       const double voxel_res,
                                       const bool enable_submap_matching)
{
    pcl::PointCloud<PointType> dst_accum, src_accum;
    int num_approx = keyframes[src_idx].pcd_.size() * 2 * submap_range;
    src_accum.reserve(num_approx);
    dst_accum.reserve(num_approx);
    if (enable_submap_matching)
    {
        for (int i = src_idx - submap_range; i < src_idx + submap_range + 1; ++i)
        {
            if (i >= 0 && i < static_cast<int>(keyframes.size()))
            {
                src_accum += transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
            }
        }
        for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i)
        {
            if (i >= 0 && i < static_cast<int>(keyframes.size()))
            {
                dst_accum += transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
            }
        }
        

    }
    else
    {
        src_accum = transformPcd(keyframes[src_idx].pcd_, keyframes[src_idx].pose_corrected_eig_);
        for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i)
        {
            if (i >= 0 && i < static_cast<int>(keyframes.size()))
            {
                dst_accum += transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
            }
        }
    }
    return {*voxelizePcd(src_accum, voxel_res), *voxelizePcd(dst_accum, voxel_res)};
}

RegistrationOutput LoopClosure::icpAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst)
{
    return pclGicpAlignment(src, dst);
}

RegistrationOutput LoopClosure::GicpAlignmentwithInit(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst,
                                             const Eigen::Matrix4d &init_reg)
{
    return pclGicpAlignmentWithInit(src, dst, init_reg);
}

RegistrationOutput LoopClosure::icpAlignmentwithInit(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst,
                                             const Eigen::Matrix4d &init_reg)
{
    RegistrationOutput reg_output;
    aligned_.clear();
    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(config_.icp_config_.max_correspondences); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(config_.icp_config_.max_iters);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(src_cloud);
    icp.setInputTarget(dst_cloud);
    
    
    icp.align(aligned_, init_reg.cast<float>());

    // handle results
    reg_output.score_ = icp.getFitnessScore();
    std::cout << " init transformation in world: " << init_reg << std::endl;
    std::cout << " icp.getFinalTransformation(): " << icp.getFinalTransformation() << std::endl;

    // if matchness score is lower than threshold, (lower is better)
    if (icp.hasConverged() && reg_output.score_ < config_.icp_config_.IcpFitnessScore)
    {
        reg_output.is_valid_ = true;
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = icp.getFinalTransformation().cast<double>();
    }
    return reg_output;
}



RegistrationOutput LoopClosure::pclIcpAlignment(const pcl::PointCloud<PointType> &src,
                                             const pcl::PointCloud<PointType> &dst
                                             )
{
    RegistrationOutput reg_output;
    aligned_.clear();
    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(config_.icp_config_.max_correspondences); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(config_.icp_config_.max_iters);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(src_cloud);
    icp.setInputTarget(dst_cloud);
    icp.align(aligned_);

    // handle results
    reg_output.score_ = icp.getFitnessScore();
    std::cout << " score_ for icp: " << reg_output.score_ << std::endl;
    std::cout << " icp.getFinalTransformation(): " << icp.getFinalTransformation() << std::endl;

    
    // if matchness score is lower than threshold, (lower is better)
    if (icp.hasConverged() && reg_output.score_ < config_.icp_config_.IcpFitnessScore)
    {
        reg_output.is_valid_ = true;
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = icp.getFinalTransformation().cast<double>();
    }
    return reg_output;
}

RegistrationOutput LoopClosure::pclGicpAlignment(const pcl::PointCloud<PointType> &src,
                                                 const pcl::PointCloud<PointType> &dst)
{
    RegistrationOutput reg_output;
    aligned_.clear();

    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;
    
    nano_gicp_.setInputSource(src_cloud);
    nano_gicp_.calculateSourceCovariances();
    nano_gicp_.setInputTarget(dst_cloud);
    nano_gicp_.calculateTargetCovariances();
    nano_gicp_.align(aligned_);

    const double score = nano_gicp_.getFitnessScore(config_.gicp_config_.max_corr_dist_);
    reg_output.score_ = std::isfinite(score) ? score : std::numeric_limits<double>::max();

    if (nano_gicp_.hasConverged())
    {
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = nano_gicp_.getFinalTransformation().cast<double>();
        if (reg_output.score_ < config_.gicp_config_.icp_score_thr_)
        {
            reg_output.is_valid_ = true;
        }
    }

    return reg_output;
}

RegistrationOutput LoopClosure::pclGicpAlignmentWithInit(const pcl::PointCloud<PointType> &src,
                                                         const pcl::PointCloud<PointType> &dst,
                                                         const Eigen::Matrix4d &init_reg)
{
    RegistrationOutput reg_output;
    aligned_.clear();

    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;

    nano_gicp_.setInputSource(src_cloud);
    nano_gicp_.calculateSourceCovariances();
    nano_gicp_.setInputTarget(dst_cloud);
    nano_gicp_.calculateTargetCovariances();
    nano_gicp_.align(aligned_, init_reg.cast<float>());

    const double score = nano_gicp_.getFitnessScore(config_.gicp_config_.max_corr_dist_);
    reg_output.score_ = std::isfinite(score) ? score : std::numeric_limits<double>::max();

    if (nano_gicp_.hasConverged())
    {
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = nano_gicp_.getFinalTransformation().cast<double>();
        if (reg_output.score_ < config_.gicp_config_.icp_score_thr_)
        {
            reg_output.is_valid_ = true;
        }
    }

    return reg_output;
}

RegistrationOutput LoopClosure::pclGeneralizedIcpAlignment(const pcl::PointCloud<PointType> &src,
                                                           const pcl::PointCloud<PointType> &dst)
{
    RegistrationOutput reg_output;
    aligned_.clear();

    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setMaxCorrespondenceDistance(config_.gicp_config_.max_corr_dist_);
    gicp.setMaximumIterations(config_.gicp_config_.nano_max_iter_);
    gicp.setTransformationEpsilon(config_.gicp_config_.transformation_epsilon_);
    gicp.setEuclideanFitnessEpsilon(config_.gicp_config_.euclidean_fitness_epsilon_);
    gicp.setRANSACIterations(config_.gicp_config_.nano_ransac_max_iter_);
    gicp.setRANSACOutlierRejectionThreshold(config_.gicp_config_.ransac_outlier_rejection_threshold_);
    gicp.setInputSource(src_cloud);
    gicp.setInputTarget(dst_cloud);
    gicp.align(aligned_);

    const double score = gicp.getFitnessScore(config_.gicp_config_.max_corr_dist_);
    reg_output.score_ = std::isfinite(score) ? score : std::numeric_limits<double>::max();

    if (gicp.hasConverged())
    {
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = gicp.getFinalTransformation().cast<double>();
        if (reg_output.score_ < config_.gicp_config_.icp_score_thr_)
        {
            reg_output.is_valid_ = true;
        }
    }

    return reg_output;
}

RegistrationOutput LoopClosure::pclGeneralizedIcpAlignmentWithInit(const pcl::PointCloud<PointType> &src,
                                                                   const pcl::PointCloud<PointType> &dst,
                                                                   const Eigen::Matrix4d &init_reg)
{
    RegistrationOutput reg_output;
    aligned_.clear();

    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;

    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setMaxCorrespondenceDistance(config_.gicp_config_.max_corr_dist_);
    gicp.setMaximumIterations(config_.gicp_config_.nano_max_iter_);
    gicp.setTransformationEpsilon(config_.gicp_config_.transformation_epsilon_);
    gicp.setEuclideanFitnessEpsilon(config_.gicp_config_.euclidean_fitness_epsilon_);
    gicp.setRANSACIterations(config_.gicp_config_.nano_ransac_max_iter_);
    gicp.setRANSACOutlierRejectionThreshold(config_.gicp_config_.ransac_outlier_rejection_threshold_);
    gicp.setInputSource(src_cloud);
    gicp.setInputTarget(dst_cloud);
    gicp.align(aligned_, init_reg.cast<float>());

    const double score = gicp.getFitnessScore(config_.gicp_config_.max_corr_dist_);
    reg_output.score_ = std::isfinite(score) ? score : std::numeric_limits<double>::max();

    if (gicp.hasConverged())
    {
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = gicp.getFinalTransformation().cast<double>();
        if (reg_output.score_ < config_.gicp_config_.icp_score_thr_)
        {
            reg_output.is_valid_ = true;
        }
    }

    return reg_output;
}





RegistrationOutput LoopClosure::performLoopClosurev2(const PosePcd &query_keyframe,
                                                   const std::vector<PosePcd> &keyframes,
                                                   const int closest_keyframe_idx,
                                                   const Eigen::Matrix4d&      init_reg)
{
    RegistrationOutput reg_output;
    closest_keyframe_idx_ = closest_keyframe_idx;
    if (closest_keyframe_idx_ >= 0)
    {   
        PcdPair clouds = setSrcAndDstCloud(keyframes,
                                        query_keyframe.idx_,
                                        closest_keyframe_idx_,
                                        config_.num_submap_keyframes_,
                                        config_.voxel_res_,
                                        config_.enable_submap_matching_);
        
        const pcl::PointCloud<PointType>& src_cloud = std::get<0>(clouds);
        const pcl::PointCloud<PointType>& dst_cloud = std::get<1>(clouds);
        // Only for visualization
        *src_cloud_ = src_cloud;
        *dst_cloud_ = dst_cloud;
        const Eigen::Matrix4d &T_gq = config_.use_odom_pose_for_loop_init_
                                          ? query_keyframe.pose_eig_
                                          : query_keyframe.pose_corrected_eig_;
        const Eigen::Matrix4d &T_gr = config_.use_odom_pose_for_loop_init_
                                          ? keyframes[closest_keyframe_idx_].pose_eig_
                                          : keyframes[closest_keyframe_idx_].pose_corrected_eig_;

        Eigen::Matrix4d T_init_world = T_gr * init_reg * T_gq.inverse();

        std::cout << "\033[1;35mExecute LighterBEV init + Nano-GICP: " << src_cloud.size()
                  << " vs " << dst_cloud.size() << "\033[0m\n";

        coarse_aligned_.clear();
        coarse_aligned_ = transformPcd(src_cloud, T_init_world);
        RegistrationOutput fine_output = icpAlignment(coarse_aligned_, dst_cloud);
        reg_output = fine_output;
        reg_output.pose_between_eig_ = fine_output.pose_between_eig_ * T_init_world;
        return reg_output;
        
    }
    else
    {
        return reg_output; // dummy output whose `is_valid` is false
    }


}

RegistrationOutput LoopClosure::performLoopClosure(const PosePcd &query_keyframe,
                                                   const std::vector<PosePcd> &keyframes,
                                                   const int closest_keyframe_idx)
{   
    RegistrationOutput reg_output;
    closest_keyframe_idx_ = closest_keyframe_idx;
    if (closest_keyframe_idx_ >= 0)
    {   
        PcdPair clouds = setSrcAndDstCloud(keyframes,
                                        query_keyframe.idx_,
                                        closest_keyframe_idx_,
                                        config_.num_submap_keyframes_,
                                        config_.voxel_res_,
                                        config_.enable_submap_matching_);
        const pcl::PointCloud<PointType>& src_cloud = std::get<0>(clouds);
        const pcl::PointCloud<PointType>& dst_cloud = std::get<1>(clouds);

        // Only for visualization
        *src_cloud_ = src_cloud;
        *dst_cloud_ = dst_cloud;

        std::cout << "\033[1;35mExecute Nano-GICP: " << src_cloud.size() << " vs "
                  << dst_cloud.size() << "\033[0m\n";
        return icpAlignment(src_cloud, dst_cloud);
    }
    else
    {
        return reg_output; // dummy output whose `is_valid` is false
    }

}

pcl::PointCloud<PointType> LoopClosure::getSourceCloud()
{
    return *src_cloud_;
}

pcl::PointCloud<PointType> LoopClosure::getTargetCloud()
{
    return *dst_cloud_;
}

pcl::PointCloud<PointType> LoopClosure::getCoarseAlignedCloud()
{
    return coarse_aligned_;
}

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> LoopClosure::getFinalAlignedCloud()
{
    return aligned_;
}

int LoopClosure::getClosestKeyframeidx()
{
    return closest_keyframe_idx_;
}

bool LoopClosure::reloadBEVModel(const std::string &state_dict_path)
{
    if (state_dict_path.empty())
    {
        ROS_WARN("[LoopClosure] Skip reloading BEV model: empty checkpoint path.");
        return false;
    }

    const bool ok = bev_manager_.reload(state_dict_path);
    if (ok)
    {
        ROS_INFO_STREAM("[LoopClosure] BEV model reloaded from: " << state_dict_path);
    }
    else
    {
        ROS_ERROR_STREAM("[LoopClosure] Failed to reload BEV model from: " << state_dict_path);
    }
    return ok;
}
