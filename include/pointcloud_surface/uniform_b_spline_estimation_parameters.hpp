#pragma once

#include <Eigen/Geometry>
#include <util_yaml/parameter_base.hpp>

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline {

enum class SmoothnessOrder { first = 0, second = 1 };

SmoothnessOrder toSmoothnessOrder(const std::string&);

std::string toString(const SmoothnessOrder&);

struct Parameters final : public util_yaml::ParameterBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string getName() const final;

    Eigen::Array2f distance_control_points{Eigen::Array2f::Constant(10)};
    float weight_smoothness{100};
    SmoothnessOrder smoothness_order{SmoothnessOrder::first};

private:
    void read_(const YAML::Node&) final;
    void write_(YAML::Emitter&) const final;
};
} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline