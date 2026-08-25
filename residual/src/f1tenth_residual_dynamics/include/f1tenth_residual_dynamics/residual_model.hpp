#pragma once

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>

namespace f1tenth_residual_dynamics {

class ResidualModel {
 public:
  static ResidualModel load(const std::string& yaml_path);

  Eigen::Vector3d predict(const Eigen::VectorXd& features) const;
  double confidence(const Eigen::VectorXd& features) const;
  Eigen::VectorXd makeFeatures(
      double vx, double vy, double yaw_rate, double physical_steering,
      const std::vector<std::array<double, 2>>& command_history) const;

  std::size_t featureCount() const { return feature_names_.size(); }
  const std::vector<std::string>& featureNames() const { return feature_names_; }
  const Eigen::Vector3d& outputLimits() const { return output_limits_; }
  int schemaVersion() const { return schema_version_; }
  const std::string& featureSet() const { return feature_set_; }

 private:
  int schema_version_{1};
  std::string feature_set_;
  std::vector<std::string> feature_names_;
  Eigen::VectorXd mean_, scale_;
  Eigen::VectorXd feature_abs_z_limit_;
  Eigen::MatrixXd coefficients_;
  Eigen::Vector3d output_limits_;
  double ood_fade_ratio_{1.5};
  double deployment_scale_{1.0};
  Eigen::Array3d active_outputs_{1.0, 1.0, 1.0};
  double gate_temperature_{20.0};
  double confidence_softness_{8.0};
};

}  // namespace f1tenth_residual_dynamics
