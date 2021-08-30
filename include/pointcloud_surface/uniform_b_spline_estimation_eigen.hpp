#pragma once

#include <uniform_bspline_eigen/uniform_bspline_eigen.hpp>
#include <util_eigen/alignment.hpp>

#include "uniform_b_spline_estimation.hpp"

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::eigen {

struct Estimator final : public uniform_b_spline::Estimator {

    using Base = uniform_b_spline::Estimator;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using typename Base::Representation;
    using Parameters = uniform_b_spline::Parameters;

    Estimator(const Parameters&, const common::Parameters&);

    void estimate() final;

private:
    void addMeasurements_(int num_measurements) final;
    void clearMeasurements_() final;

    EigenAlignedVec<Eigen::Vector2d> x_;
    Eigen::VectorXd y_;
    Eigen::VectorXd w_;
    ubs::UniformBSplineEigen<Representation> spline_eigen_{Base::spline_};
};
} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::eigen