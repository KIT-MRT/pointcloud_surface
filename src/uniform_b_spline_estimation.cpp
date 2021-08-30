#include "uniform_b_spline_estimation.hpp"

#include <glog/logging.h>

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline {

Estimator::Estimator(const Parameters& p, const common::Parameters& p_common) : Base{p_common}, p_{p} {
}

std::unique_ptr<ground_surface::GroundSurface> Estimator::getGroundSurface() const {
    return std::make_unique<ground_surface::UniformBSpline>(spline_);
}

void Estimator::reset() {
    initializeSpline();
    this->clearMeasurements();
}

void Estimator::initializeSpline() {
    const Eigen::Array2i num_control_points{
        (Base::p_.range.sizes().array() / p_.distance_control_points).ceil().template cast<int>()};
    spline_.setControlPoints(ground_surface::uniform_b_spline::ControlPoints::Constant(
        num_control_points.x(), num_control_points.y(), -Base::p_.plane_initial.offset()));
    spline_.setBounds(Base::p_.range.min().cast<double>(), Base::p_.range.max().cast<double>());
}

void Estimator::setWeights_() {
    /**
     * Currently, weights have to be readded to the ceres problem together with the measurements which is not optimal.
     * \todo Change this
     */
    clearMeasurements_();
    addMeasurements_(measurements_.cols());
}

} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline