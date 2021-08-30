#pragma once

#include <ceres/problem.h>
#include <ceres/solver.h>
#include <util_yaml/parameter_base.hpp>

namespace pointcloud_surface::ground_surface_estimation::ceres {

struct Parameters final : public util_yaml::ParameterBase {

    std::string getName() const final;

    ::ceres::Solver::Options solver;
    ::ceres::Problem::Options problem;

private:
    void read_(const YAML::Node&) final;
    void write_(YAML::Emitter&) const final;
};
} // namespace pointcloud_surface::ground_surface_estimation::ceres