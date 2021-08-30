#include "uniform_b_spline_estimation_eigen.hpp"

#include <Eigen/CholmodSupport>

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::eigen {

Estimator::Estimator(const Parameters& p_uniform_b_spline, const common::Parameters& p_common)
        : Base{p_uniform_b_spline, p_common} {
    Base::reset();
}

void Estimator::addMeasurements_(const int num_measurements) {
    auto num_valid_measurements{x_.size()};
    x_.resize(measurements_.cols());
    y_.conservativeResize(measurements_.cols());
    w_.conservativeResize(weights_.size());
    for (int c = measurements_.cols() - num_measurements; c < measurements_.cols(); c++) {
        const Eigen::Vector3d p{measurements_.col(c).template cast<double>()};
        const Eigen::Vector2d& p2d{p.template head<2>()};
        if (Base::spline_.inRange(p2d)) {
            x_[num_valid_measurements] = p2d;
            y_[num_valid_measurements] = p.z();
            w_[num_valid_measurements] = weights_[c];
            num_valid_measurements++;
        }
    }
    x_.resize(num_valid_measurements);
    y_.conservativeResize(num_valid_measurements);
    w_.conservativeResize(num_valid_measurements);
}

void Estimator::estimate() {
    CHECK_EQ(x_.size(), y_.size());
    CHECK_EQ(y_.size(), w_.size());
    bool success{false};
    if (Base::p_.weight_smoothness > 0) {
        Eigen::CholmodSimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        switch (Base::p_.smoothness_order) {
        case SmoothnessOrder::first:
            success = spline_eigen_.template solveWeighted<1>(
                std::begin(x_), std::end(x_), y_.data(), w_.data(), solver, true, Base::p_.weight_smoothness);
            break;
        case SmoothnessOrder::second:
            success = spline_eigen_.template solveWeighted<2>(
                std::begin(x_), std::end(x_), y_.data(), w_.data(), solver, true, Base::p_.weight_smoothness);
            break;
        }
    } else {
        success = spline_eigen_.solveWeighted(std::begin(x_), std::end(x_), y_.data(), w_.data());
    }
    LOG_IF(WARNING, !success) << "Optimization failed.";
}

void Estimator::clearMeasurements_() {
    x_.clear();
    y_.resize(0);
    w_.resize(0);
}
} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::eigen