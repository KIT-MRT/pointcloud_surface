#include "ground_surface_estimation_constant.hpp"

namespace pointcloud_surface::ground_surface_estimation::constant {

Estimator::Estimator(const Parameters& p) : Base{p} {
}

void Estimator::addMeasurements(const Measurements& measurements) {
    estimator_->addMeasurements(measurements);
}

void Estimator::estimate() {
    for (size_t it = 0; it < p_.num_iterations; it++) {
        estimator_->estimate();
    }
}
} // namespace pointcloud_surface::ground_surface_estimation::constant