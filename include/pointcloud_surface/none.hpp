#pragma once

#include "ground_surface_estimation_base.hpp"

namespace pointcloud_surface::ground_surface_estimation::none {

class Estimator final : public common::Estimator {

    using Base = common::Estimator;

public:
    Estimator(const common::Parameters&);

    void estimate() final;

    std::unique_ptr<ground_surface::GroundSurface> getGroundSurface() const final;

private:
    void addMeasurements_(int num_measurements) final;
    void clearMeasurements_() final;
};
} // namespace pointcloud_surface::ground_surface_estimation::none