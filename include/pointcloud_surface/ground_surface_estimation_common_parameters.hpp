#pragma once

#include <Eigen/Geometry>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <util_yaml/parameter_base.hpp>

namespace pointcloud_surface::ground_surface_estimation::common {

struct Parameters final : public util_yaml::ParameterBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string getName() const final;

    Eigen::Hyperplane<float, 3> plane_initial{Eigen::Vector3f::UnitZ(), 0};
    Eigen::AlignedBox2f range{Eigen::Vector2f::Constant(-100), Eigen::Vector2f::Constant(100)};

private:
    void read_(const YAML::Node&) final;
    void write_(YAML::Emitter&) const final;
};


} // namespace pointcloud_surface::ground_surface_estimation::common