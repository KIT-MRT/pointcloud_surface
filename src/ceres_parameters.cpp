#include "ceres_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation::ceres {

std::string Parameters::getName() const {
    return "ceres";
}

void Parameters::read_(const YAML::Node& n) {
}

void Parameters::write_(YAML::Emitter& e) const {
}
} // namespace pointcloud_surface::ground_surface_estimation::ceres