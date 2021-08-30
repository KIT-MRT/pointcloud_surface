#include "ground_surface_estimation.hpp"

#include "none.hpp"
#include "plane_estimation_ceres.hpp"
#include "plane_estimation_sac.hpp"
#include "polynomial_estimation_ceres.hpp"
#include "uniform_b_spline_estimation_ceres.hpp"
#include "uniform_b_spline_estimation_eigen.hpp"

namespace pointcloud_surface::ground_surface_estimation {

std::unique_ptr<common::Estimator> createImpl(const ground_surface_estimation::Parameters& p) {
    using Method = ground_surface_estimation::Method;
    switch (p.method) {
    case Method::plane_sac:
        return std::make_unique<plane::sac::Estimator>(p.plane_sac, p.common);
    case Method::plane_ceres:
        return std::make_unique<plane::ceres::Estimator>(p.ceres, p.common);
    case Method::uniform_b_spline_ceres:
        return std::make_unique<uniform_b_spline::ceres::Estimator>(p.uniform_b_spline, p.ceres, p.common);
    case Method::uniform_b_spline_eigen:
        return std::make_unique<uniform_b_spline::eigen::Estimator>(p.uniform_b_spline, p.common);
    case Method::polynomial_ceres:
        return std::make_unique<polynomial::ceres::Estimator>(p.ceres, p.common);
    default: ///< none
        return std::make_unique<none::Estimator>(p.common);
    }
}

Estimator::Estimator(const Parameters& p) : p_{p}, estimator_{createImpl(p)}, mu_{p.mu} {
}

ground_surface::GroundSurface::Ptr Estimator::getGroundSurface() const {
    return estimator_->getGroundSurface();
}

bool Estimator::hasMeasurements() const {
    return estimator_->hasMeasurements();
}

void Estimator::clearMeasurements() {
    estimator_->clearMeasurements();
}

const Estimator::Measurements& Estimator::getMeasurements() const {
    return estimator_->getMeasurements();
}

const Eigen::VectorXf& Estimator::getWeights() const {
    return estimator_->getWeights();
}

const float& Estimator::getMu() const {
    return mu_;
}

void Estimator::reset() {
    mu_ = p_.mu;
    clearMeasurements();
    estimator_->reset();
}
} // namespace pointcloud_surface::ground_surface_estimation