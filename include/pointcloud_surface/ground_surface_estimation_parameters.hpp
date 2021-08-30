#pragma once

#include <util_yaml/parameter_base.hpp>

#include "ceres_parameters.hpp"
#include "ground_surface_estimation_common_parameters.hpp"
#include "plane_estimation_sac_parameters.hpp"
#include "point_labelers.hpp"
#include "uniform_b_spline_estimation_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation {

enum class Method {
    plane_ceres = 0,
    plane_sac = 1,
    uniform_b_spline_ceres = 2,
    uniform_b_spline_eigen = 3,
    polynomial_ceres = 4,
    none
};

Method toMethod(const std::string&);

std::string toString(const Method&);

struct Parameters final : public util_yaml::ParameterBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::string getName() const final;

    Method method{Method::plane_ceres};
    size_t num_iterations{3};
    PointLabeler point_labeler{PointLabeler::constant};
    float distance_threshold{1};
    float mu{1};
    float asymmetry_ratio{1};

    common::Parameters common;
    ground_surface_estimation::ceres::Parameters ceres;
    plane::sac::Parameters plane_sac;
    uniform_b_spline::Parameters uniform_b_spline;

private:
    void read_(const YAML::Node&) final;
    void write_(YAML::Emitter&) const final;
};
} // namespace pointcloud_surface::ground_surface_estimation