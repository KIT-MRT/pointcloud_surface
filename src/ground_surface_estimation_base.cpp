#include "ground_surface_estimation_base.hpp"

#include <glog/logging.h>

namespace pointcloud_surface::ground_surface_estimation::common {

Estimator::Estimator(const Parameters& p) : p_{p} {
}

void Estimator::addMeasurements(const Measurements& measurements) {
    addMeasurements(measurements, Eigen::VectorXf::Ones(measurements.cols()));
}

void Estimator::addMeasurements(const Measurements& measurements, const Weights& weights) {
    CHECK_EQ(measurements.cols(), weights.size());
    const auto new_size{measurements_.cols() + measurements.cols()};
    measurements_.conservativeResize(3, new_size);
    measurements_.rightCols(measurements.cols()) = measurements;
    weights_.conservativeResize(new_size);
    weights_.tail(measurements.cols()) = weights;
    addMeasurements_(measurements.cols());
}

const Estimator::Measurements& Estimator::getMeasurements() const {
    return measurements_;
}

const Estimator::Weights& Estimator::getWeights() const {
    return weights_;
}

void Estimator::setWeights(const Weights& weights) {
    CHECK_EQ(weights.size(), weights_.size());
    weights_ = weights;
    setWeights_();
}

void Estimator::set(const Parameters& p) {
    p_ = p;
    reset();
}

const Parameters& Estimator::getParameters() const {
    return p_;
}

bool Estimator::hasMeasurements() const {
    return static_cast<bool>(measurements_.cols());
}

void Estimator::clearMeasurements() {
    measurements_.resize(3, 0);
    weights_.resize(0);
    clearMeasurements_();
}

void Estimator::reset() {
}

void Estimator::setWeights_() {
}

} // namespace pointcloud_surface::ground_surface_estimation::common