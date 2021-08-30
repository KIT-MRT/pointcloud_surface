#pragma once

#include <ceres/problem.h>
#include <uniform_bspline_ceres/uniform_bspline_ceres.hpp>

#include "ceres_parameters.hpp"
#include "uniform_b_spline_estimation.hpp"

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::ceres {

struct Estimator final : public uniform_b_spline::Estimator {

    using Base = uniform_b_spline::Estimator;
    using Base::Representation;
    using Parameters = uniform_b_spline::Parameters;

    Estimator(const Parameters&, const ground_surface_estimation::ceres::Parameters&, const common::Parameters&);

    void estimate() final;

private:
    void addMeasurements_(int num_measurements) final;
    void clearMeasurements_() final;

    ground_surface_estimation::ceres::Parameters p_ceres_;
    std::unique_ptr<::ceres::Problem> problem_{nullptr};
    ubs::UniformBSplineCeres<Representation> spline_ceres_{spline_};
};
} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::ceres