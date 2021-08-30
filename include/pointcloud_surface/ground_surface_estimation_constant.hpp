#pragma once

#include "ground_surface_estimation.hpp"

namespace pointcloud_surface::ground_surface_estimation::constant {

class Estimator final : public ground_surface_estimation::Estimator {
    using Base = ground_surface_estimation::Estimator;

public:
    Estimator(const Parameters&);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void addMeasurements(const Measurements&) final;

    void estimate() final;
};
} // namespace pointcloud_surface::ground_surface_estimation::constant