#pragma once

#include <util_yaml/parameter_base.hpp>

namespace pointcloud_surface::ground_surface_estimation::plane::sac {

struct Parameters final : public util_yaml::ParameterBase {

    std::string getName() const final;

    bool optimize_coefficients{true};
    float inlier_threshold{0.2};
    float eps_angle{0.01};
    size_t max_iterations{20};
    float probability{0.99};

private:
    void read_(const YAML::Node&) final;
    void write_(YAML::Emitter&) const final;
};
} // namespace pointcloud_surface::ground_surface_estimation::plane::sac