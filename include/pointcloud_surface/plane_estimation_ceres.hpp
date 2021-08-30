#pragma once

#include <ceres/loss_function.h>
#include <ceres/problem.h>
#include <util_ceres/plane.h>

#include "ceres_parameters.hpp"
#include "ground_surface_estimation_base.hpp"

namespace pointcloud_surface::ground_surface_estimation::plane::ceres {

class Estimator final : public common::Estimator {

    using Base = common::Estimator;
    using Plane = util_ceres::Plane<>;

public:
    using Parameters = ground_surface_estimation::ceres::Parameters;

    Estimator(const Parameters&, const common::Parameters&);

    void estimate() final;

    std::unique_ptr<ground_surface::GroundSurface> getGroundSurface() const final;

    void reset() final;

private:
    void addMeasurements_(int num_measurements) final;
    void clearMeasurements_() final;

    Parameters p_;
    std::unique_ptr<::ceres::Problem> problem_{nullptr};
    Plane plane_;
};
} // namespace pointcloud_surface::ground_surface_estimation::plane::ceres