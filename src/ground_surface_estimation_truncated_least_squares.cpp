#include "ground_surface_estimation_truncated_least_squares.hpp"

#include "ground_surface_point_label.hpp"
#include "point_labelers.hpp"

namespace pointcloud_surface::ground_surface_estimation::tls {

using Labeler = point_labelers::TruncatedLeastSquares;

Estimator::Estimator(const Parameters& p) : Base{p} {
}

void Estimator::filterMeasurements(const float mu, Measurements& m_out, Weights& w_out) const {
    const Eigen::VectorXf weights{ground_surface::label(
        estimator_->getGroundSurface(), Labeler{p_.distance_threshold, mu}, p_.asymmetry_ratio, measurements_)};
    m_out.resize(3, measurements_.cols());
    w_out.resize(weights.size());
    size_t num_valid_samples{0};
    for (size_t c = 0; c < measurements_.cols(); c++) {
        if (weights[c] > 0.f) {
            m_out.col(num_valid_samples) = measurements_.col(c);
            w_out[num_valid_samples] = weights[c];
            num_valid_samples++;
        }
    }
    m_out.conservativeResize(3, num_valid_samples);
    w_out.conservativeResize(num_valid_samples);
}

void Estimator::addMeasurements(const Measurements& measurements) {
    measurements_.conservativeResize(3, measurements_.cols() + measurements.cols());
    measurements_.rightCols(measurements.cols()) = measurements;

    Measurements m_filtered;
    Weights w_filtered;
    filterMeasurements(p_.mu, m_filtered, w_filtered);
    estimator_->addMeasurements(m_filtered, w_filtered);
}

void Estimator::clearMeasurements() {
    measurements_.resize(3, 0);
    Base::clearMeasurements();
}

void Estimator::estimate() {

    /**
     * Initialize convexity parameter (mu)
     */
    mu_ = p_.mu;
    for (size_t it = 0; it < p_.num_iterations; it++) {
        estimator_->estimate();

        /**
         * Recompute and update weights
         */
        mu_ *= 1.4;
        Eigen::Matrix3Xf m_filtered;
        Eigen::VectorXf w_filtered;
        filterMeasurements(mu_, m_filtered, w_filtered);
        estimator_->clearMeasurements();
        estimator_->addMeasurements(m_filtered, w_filtered);
    }
    estimator_->clearMeasurements();
    estimator_->addMeasurements(
        measurements_,
        ground_surface::label(
            estimator_->getGroundSurface(), Labeler{p_.distance_threshold, mu_}, p_.asymmetry_ratio, measurements_));
}
} // namespace pointcloud_surface::ground_surface_estimation::tls