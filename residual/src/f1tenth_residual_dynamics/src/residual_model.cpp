#include "f1tenth_residual_dynamics/residual_model.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace f1tenth_residual_dynamics {
namespace {
Eigen::VectorXd vector(const YAML::Node& node) {
  Eigen::VectorXd output(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) output[i] = node[i].as<double>();
  return output;
}

Eigen::MatrixXd matrix(const YAML::Node& node) {
  Eigen::MatrixXd output(node.size(), 3);
  for (std::size_t row = 0; row < node.size(); ++row) {
    if (node[row].size() != 3) throw std::runtime_error("residual output size must be 3");
    for (int column = 0; column < 3; ++column)
      output(row, column) = node[row][column].as<double>();
  }
  return output;
}

double softplus(double value) {
  if (value > 30.0) return value;
  if (value < -30.0) return std::exp(value);
  return std::log1p(std::exp(value));
}
}  // namespace

ResidualModel ResidualModel::load(const std::string& yaml_path) {
  const auto root = YAML::LoadFile(yaml_path);
  if (!root["schema_version"])
    throw std::runtime_error("unsupported residual model schema");
  const int schema = root["schema_version"].as<int>();
  ResidualModel model;
  model.schema_version_ = schema;
  model.feature_names_ = root["feature_names"].as<std::vector<std::string>>();
  if (schema == 1) {
    model.feature_set_ = root["metadata"] && root["metadata"]["feature_set"]
        ? root["metadata"]["feature_set"].as<std::string>() : "markov_v1";
    model.mean_ = vector(root["feature_mean"]);
    model.scale_ = vector(root["feature_scale"]);
    model.coefficients_ = matrix(root["coefficients_feature_by_output"]);
    model.feature_abs_z_limit_ = root["feature_abs_z_limit"]
        ? vector(root["feature_abs_z_limit"])
        : Eigen::VectorXd::Constant(model.mean_.size(),
                                    std::numeric_limits<double>::infinity());
    model.ood_fade_ratio_ = root["ood_fade_ratio"]
        ? root["ood_fade_ratio"].as<double>() : 1.5;
    const auto limits = vector(root["output_limits"]);
    if (limits.size() != 3 || model.mean_.size() != model.coefficients_.rows() ||
        model.scale_.size() != model.coefficients_.rows() ||
        model.feature_abs_z_limit_.size() != model.coefficients_.rows() ||
        model.feature_names_.size() != static_cast<std::size_t>(model.coefficients_.rows()))
      throw std::runtime_error("inconsistent residual model dimensions");
    model.output_limits_ = limits;
    return model;
  }
  if (schema == 2) {
    model.feature_set_ = root["feature_set"]
        ? root["feature_set"].as<std::string>() : "markov_history_v2";
    model.mean_ = vector(root["mean"]);
    model.scale_ = vector(root["scale"]);
    model.coefficients_ = matrix(root["weights"]);
    model.feature_abs_z_limit_ = vector(root["envelope"]);
    model.output_limits_ = vector(root["output_limits"]);
    model.deployment_scale_ = root["deployment_scale"]
        ? root["deployment_scale"].as<double>() : 1.0;
    if (root["active_outputs"]) {
      for (int i = 0; i < 3; ++i) model.active_outputs_[i] =
          root["active_outputs"][i].as<int>() ? 1.0 : 0.0;
    }
    if (root["gate"]) {
      model.gate_temperature_ = root["gate"]["temperature"]
          ? root["gate"]["temperature"].as<double>() : model.gate_temperature_;
      model.confidence_softness_ = root["gate"]["confidence_softness"]
          ? root["gate"]["confidence_softness"].as<double>() : model.confidence_softness_;
    }
    if (model.output_limits_.size() != 3 ||
        model.mean_.size() + 1 != model.coefficients_.rows() ||
        model.scale_.size() != model.mean_.size() ||
        model.feature_abs_z_limit_.size() != model.mean_.size() ||
        model.feature_names_.size() != static_cast<std::size_t>(model.coefficients_.rows()))
      throw std::runtime_error("inconsistent residual v2 model dimensions");
    return model;
  }
  throw std::runtime_error("unsupported residual model schema");
}

Eigen::Vector3d ResidualModel::predict(const Eigen::VectorXd& features) const {
  if (schema_version_ == 1) {
    if (features.size() != mean_.size()) throw std::invalid_argument("residual feature size mismatch");
    Eigen::Vector3d output = ((features - mean_).array() / scale_.array()).matrix().transpose() * coefficients_;
    for (int i = 0; i < 3; ++i)
      output[i] = std::max(-output_limits_[i], std::min(output[i], output_limits_[i]));
    return confidence(features) * output;
  }
  if (features.size() != mean_.size() + 1) throw std::invalid_argument("residual v2 feature size mismatch");
  Eigen::VectorXd normalized(features.size());
  normalized[0] = features[0];
  normalized.tail(mean_.size()) =
      ((features.tail(mean_.size()) - mean_).array() / scale_.array()).matrix();
  Eigen::Vector3d raw = normalized.transpose() * coefficients_ * deployment_scale_;
  Eigen::Vector3d bounded;
  for (int i = 0; i < 3; ++i)
    bounded[i] = output_limits_[i] * std::tanh(raw[i] / output_limits_[i]);
  const auto ratio = (normalized.tail(mean_.size()).array().abs() /
                      feature_abs_z_limit_.array()).matrix();
  double q = 0.0;
  if (ratio.size() > 0) {
    const double temperature = std::max(gate_temperature_, 1.0e-3);
    double maximum = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < ratio.size(); ++i)
      maximum = std::max(maximum, temperature * ratio[i]);
    double sum = 0.0;
    for (int i = 0; i < ratio.size(); ++i)
      sum += std::exp(temperature * ratio[i] - maximum);
    q = (maximum + std::log(sum) - std::log(static_cast<double>(ratio.size()))) /
        temperature;
  }
  const double transition_temperature = std::max(gate_temperature_, 1.0e-3);
  const double excess = transition_temperature * (q - 1.0);
  const double sp = softplus(excess) / transition_temperature;
  const double conf = std::exp(-confidence_softness_ * sp * sp);
  return (conf * bounded.array() * active_outputs_).matrix();
}

double ResidualModel::confidence(const Eigen::VectorXd& features) const {
  if (schema_version_ == 2) {
    if (features.size() != mean_.size() + 1) throw std::invalid_argument("residual v2 feature size mismatch");
    Eigen::VectorXd normalized =
        ((features.tail(mean_.size()) - mean_).array() / scale_.array()).matrix();
    const auto ratio = (normalized.array().abs() / feature_abs_z_limit_.array()).matrix();
    if (ratio.size() == 0) return 1.0;
    const double temperature = std::max(gate_temperature_, 1.0e-3);
    double maximum = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < ratio.size(); ++i)
      maximum = std::max(maximum, temperature * ratio[i]);
    double sum = 0.0;
    for (int i = 0; i < ratio.size(); ++i)
      sum += std::exp(temperature * ratio[i] - maximum);
    const double q = (maximum + std::log(sum) -
        std::log(static_cast<double>(ratio.size()))) / temperature;
    const double transition_temperature = std::max(gate_temperature_, 1.0e-3);
    const double sp = softplus(transition_temperature * (q - 1.0)) /
                      transition_temperature;
    return std::exp(-confidence_softness_ * sp * sp);
  }
  if (features.size() != mean_.size()) throw std::invalid_argument("residual feature size mismatch");
  const auto normalized = ((features - mean_).array() / scale_.array()).abs();
  const double ratio = (normalized / feature_abs_z_limit_.array()).maxCoeff();
  const double denominator = std::max(ood_fade_ratio_ - 1.0, 1.0e-6);
  return std::max(0.0, std::min(1.0, (ood_fade_ratio_ - ratio) / denominator));
}

Eigen::VectorXd ResidualModel::makeFeatures(
    double vx, double vy, double yaw_rate, double delta,
    const std::vector<std::array<double, 2>>& history) const {
  if (history.empty()) throw std::invalid_argument("command history is empty");
  const double speed = history.front()[0], steer = history.front()[1];
  std::vector<double> values{
      1.0, vx, vy, yaw_rate, delta, speed, steer, speed - vx, steer - delta,
      vx * yaw_rate, vy * yaw_rate, std::abs(vx) * vy,
      std::abs(vx) * yaw_rate, vx * std::tan(std::max(-0.7, std::min(delta, 0.7))),
      vx * steer, vx * vx * steer};
  for (std::size_t index = 1; index < history.size(); ++index) {
    values.push_back(history[index][0]);
    values.push_back(history[index][1]);
  }
  if (values.size() != feature_names_.size())
    throw std::invalid_argument("command history does not match trained feature layout");
  return Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
}

}  // namespace f1tenth_residual_dynamics
