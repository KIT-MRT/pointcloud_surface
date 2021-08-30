#pragma once

#include <memory>

#include "ground_surface_estimation.hpp"

namespace pointcloud_surface::ground_surface_estimation::tls {

class Estimator final : public ground_surface_estimation::Estimator {
    using Base = ground_surface_estimation::Estimator;

public:
    Estimator(const Parameters&);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void addMeasurements(const Measurements&) final;

    void clearMeasurements() final;

    void estimate() final;

private:
    void filterMeasurements(const float mu, Measurements&, Weights&) const;

    Measurements measurements_;
};
} // namespace pointcloud_surface::ground_surface_estimation::tls