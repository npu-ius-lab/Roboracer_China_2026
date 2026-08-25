/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

/***********************************************************************
 * BSD 3-Clause License
 * 
 * Copyright (c) 2020, SMRT-AIST
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#ifndef NANO_GICP_NANO_GICP_HPP
#define NANO_GICP_NANO_GICP_HPP

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/registration/registration.h>

#include <nano_gicp/lsq_registration.hpp>
#include <nano_gicp/gicp/gicp_settings.hpp>
#include <nano_gicp/nanoflann.hpp>
#include <ikd_Tree.h>

namespace nano_gicp {

template<typename PointSource, typename PointTarget>
class NanoGICP : public LsqRegistration<PointSource, PointTarget> {
public:
  using Scalar = float;
  using Matrix4 = typename pcl::Registration<PointSource, PointTarget, Scalar>::Matrix4;

  using PointCloudSource = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget = typename pcl::Registration<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

protected:
  using pcl::Registration<PointSource, PointTarget, Scalar>::reg_name_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::input_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::target_;
  using pcl::Registration<PointSource, PointTarget, Scalar>::corr_dist_threshold_;

public:
  NanoGICP();
  virtual ~NanoGICP() override;

  void setNumThreads(int n);
  void setCovarianceThreads(int n);
  void setCorrespondenceRandomness(int k);
  void setRegularizationMethod(RegularizationMethod method);

  virtual void swapSourceAndTarget() override;
  virtual void clearSource() override;
  virtual void clearTarget() override;

  virtual void setInputSource(const PointCloudSourceConstPtr& cloud) override;
  virtual void setSourceCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);
  virtual void setInputTarget(const PointCloudTargetConstPtr& cloud) override;
  virtual void setTargetCovariances(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covs);

  virtual void registerInputSource(const PointCloudSourceConstPtr& cloud);

  virtual bool calculateSourceCovariances();
  virtual bool calculateTargetCovariances();

  // Incremental ikd-Tree target mode. Points carry their stable index in the
  // `intensity` field (unused by GICP math) so Nearest_Search results can be
  // mapped back to target_covs_. setInputTargetIkd builds the tree from
  // scratch; add/exact-delete update it without rebuilding.
  void setInputTargetIkd(const PointCloudTargetConstPtr& cloud);
  void addTargetPointsIkd(const PointCloudTargetConstPtr& cloud);
  void deleteTargetPointsIkd(const PointCloudTargetConstPtr& cloud);
  void clearTargetIkd();
  bool ikdTargetMode() const { return ikd_target_mode_; }
  std::size_t activeTargetPoints() {
    if (!ikd_target_mode_ || !ikd_target_)
      return target_ ? target_->size() : 0;
    const int count = ikd_target_->validnum();
    // validnum() can temporarily return -1 while ikd performs its internal
    // asynchronous rebuild. Stored size is a conservative fallback for
    // diagnostics/compaction decisions in that short interval.
    return count >= 0 ? static_cast<std::size_t>(count)
                      : (target_ ? target_->size() : 0);
  }
  std::size_t storedTargetPoints() const {
    return target_ ? target_->size() : 0;
  }

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getSourceCovariances() const {
    return source_covs_;
  }

  const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& getTargetCovariances() const {
    return target_covs_;
  }

  // Final-correspondence statistics, populated by the last update_correspondences
  // call inside align(). Mirrors a separate KdTree nearest-neighbour pass with
  // the same corr_dist_threshold_ gate, without rebuilding any search structure.
  int numCorrespondences() const {
    int count = 0;
    for (int index : correspondences_) {
      if (index >= 0)
        ++count;
    }
    return count;
  }

  double meanCorrespondenceResidual() const {
    double sum = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < correspondences_.size(); ++i) {
      if (correspondences_[i] >= 0) {
        sum += std::sqrt(static_cast<double>(sq_distances_[i]));
        ++count;
      }
    }
    return count > 0 ? sum / static_cast<double>(count)
                     : std::numeric_limits<double>::infinity();
  }

  double meanSquaredCorrespondenceResidual() const {
    double sum = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < correspondences_.size(); ++i) {
      if (correspondences_[i] >= 0) {
        sum += static_cast<double>(sq_distances_[i]);
        ++count;
      }
    }
    return count > 0 ? sum / static_cast<double>(count)
                     : std::numeric_limits<double>::infinity();
  }

protected:
  virtual void computeTransformation(PointCloudSource& output, const Matrix4& guess) override;

  virtual void update_correspondences(const Eigen::Isometry3d& trans);

  virtual double linearize(const Eigen::Isometry3d& trans, Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>* b) override;

  virtual double compute_error(const Eigen::Isometry3d& trans) override;

  template<typename PointT>
  bool calculate_covariances(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, nanoflann::KdTreeFLANN<PointT>& kdtree, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>& covariances);

  // ikd-mode covariance computation for target indices [begin, end), using
  // the incremental tree instead of the static nanoflann kd-tree.
  void calculate_covariances_ikd(std::size_t begin, std::size_t end);

public:
  std::shared_ptr<nanoflann::KdTreeFLANN<PointSource>> source_kdtree_;
  std::shared_ptr<nanoflann::KdTreeFLANN<PointTarget>> target_kdtree_;

  std::shared_ptr<KD_TREE<PointTarget>> ikd_target_;
  typename pcl::PointCloud<PointTarget>::Ptr ikd_target_cloud_;
  bool ikd_target_mode_ = false;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> source_covs_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> target_covs_;

protected:
  int num_threads_;
  int covariance_num_threads_ = 0;
  int k_correspondences_;

  RegularizationMethod regularization_method_;

  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mahalanobis_;

  std::vector<int> correspondences_;
  std::vector<float> sq_distances_;
};
}  // namespace nano_gicp

#endif
