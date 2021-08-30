#include "plane_estimation_sac_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation::plane::sac {

std::string Parameters::getName() const {
    return "plane_sac";
}

void Parameters::read_(const YAML::Node& n) {
    read(n, "optimize_coefficients", optimize_coefficients);
    read(n, "inlier_threshold", inlier_threshold);
    read(n, "eps_angle", eps_angle);
    read(n, "max_iterations", max_iterations);
    read(n, "probability", probability);
}

void Parameters::write_(YAML::Emitter& e) const {
    write("optimize_coefficients", optimize_coefficients, e);
    write("inlier_threshold", inlier_threshold, e);
    write("eps_angle", eps_angle, e);
    write("max_iterations", max_iterations, e);
    write("probability", probability, e);
}
} // namespace pointcloud_surface::ground_surface_estimation::plane::sac