#pragma once

#include <ceres/problem.h>

#include "ceres_parameters.hpp"
#include "ground_surface_estimation_base.hpp"
#include "ground_surface_polynomial_representation.hpp"

namespace pointcloud_surface::ground_surface_estimation::polynomial::ceres {

class Estimator final : public common::Estimator {

    using Base = common::Estimator;

public:
    template <typename T>
    using Representation = ground_surface::polynomial::Representation<T>;

public:
    Estimator(const ground_surface_estimation::ceres::Parameters&, const common::Parameters&);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void estimate() final;

    std::unique_ptr<ground_surface::GroundSurface> getGroundSurface() const final;

private:
    void addMeasurements_(int num_measurements) final;
    void clearMeasurements_() final;

    ground_surface_estimation::ceres::Parameters p_ceres_;
    std::unique_ptr<::ceres::Problem> problem_{nullptr};
    Representation<double> poly_ceres_{Representation<double>::Zero()};
};
} // namespace pointcloud_surface::ground_surface_estimation::polynomial::ceres