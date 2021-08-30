#include "none.hpp"

#include "ground_surface_plane.hpp"

namespace pointcloud_surface::ground_surface_estimation::none {

Estimator::Estimator(const common::Parameters& p) : Base{p} {
}

void Estimator::addMeasurements_(const int) {
}

void Estimator::estimate() {
}

std::unique_ptr<ground_surface::GroundSurface> Estimator::getGroundSurface() const {
    return std::make_unique<ground_surface::Plane>(Base::p_.plane_initial, Base::p_.range);
}

void Estimator::clearMeasurements_() {
}
} // namespace pointcloud_surface::ground_surface_estimation::none