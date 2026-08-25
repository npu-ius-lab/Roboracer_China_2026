#include "f1tenth_residual_dynamics/residual_model.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
Eigen::VectorXd parseFeatures(const std::string& text) {
  std::stringstream stream(text);
  std::string item;
  std::vector<double> values;
  while (std::getline(stream, item, ',')) values.push_back(std::stod(item));
  return Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3) {
    std::cerr << "usage: residual_model_inspect MODEL.yaml [FEATURES_CSV]\n";
    return 2;
  }
  try {
    const auto model = f1tenth_residual_dynamics::ResidualModel::load(argv[1]);
    std::cout << "features=" << model.featureCount()
              << " schema=" << model.schemaVersion()
              << " feature_set=" << model.featureSet()
              << " limits=" << model.outputLimits().transpose() << "\n";
    if (argc == 3) {
      const auto features = parseFeatures(argv[2]);
      if (features.size() != static_cast<Eigen::Index>(model.featureCount()))
        throw std::runtime_error("FEATURES_CSV dimension does not match model");
      std::cout << std::setprecision(17)
                << "confidence=" << model.confidence(features) << "\n"
                << "prediction=" << model.predict(features).transpose() << "\n";
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
