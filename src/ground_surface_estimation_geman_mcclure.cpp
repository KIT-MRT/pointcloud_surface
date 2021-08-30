#include "ground_surface_estimation_geman_mcclure.hpp"

#include <glog/logging.h>

#include "ground_surface_point_label.hpp"
#include "point_labelers.hpp"

namespace pointcloud_surface::ground_surface_estimation::gmc {

using Labeler = point_labelers::GemanMcClure;

Estimator::Estimator(const Parameters& p) : Base{p} {
}

void Estimator::addMeasurements(const Measurements& measurements) {
    estimator_->addMeasurements(
        measurements,
        ground_surface::label(
            estimator_->getGroundSurface(), Labeler{p_.distance_threshold, p_.mu}, p_.asymmetry_ratio, measurements));
}

void Estimator::estimate() {
    mu_ = p_.mu;
    for (size_t it = 0; it < p_.num_iterations && mu_ > 1; it++) {
        DLOG(INFO) << "Iteration: " << it << ", mu: " << mu_;
        estimator_->estimate();
        mu_ /= 1.4;
        estimator_->setWeights(ground_surface::label(estimator_->getGroundSurface(),
                                                     Labeler{p_.distance_threshold, mu_},
                                                     p_.asymmetry_ratio,
                                                     estimator_->getMeasurements()));
    }
}
} // namespace pointcloud_surface::ground_surface_estimation::gmc