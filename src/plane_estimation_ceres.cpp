#include "plane_estimation_ceres.hpp"

#include <glog/logging.h>
#include <util_ceres/residual_point_to_plane.hpp>
#include <util_ceres/residual_points_to_plane.hpp>

#include "ground_surface_plane.hpp"

namespace pointcloud_surface::ground_surface_estimation::plane::ceres {

Estimator::Estimator(const Parameters& p, const common::Parameters& p_common) : Base{p_common}, p_{p} {
    reset();
}

void Estimator::addMeasurements_(const int num_measurements) {
    problem_->AddResidualBlock(
        util_ceres::residuals::PointsToPlane<float, 3>::create(measurements_.rightCols(num_measurements)),
        new ::ceres::TrivialLoss,
        plane_.data());
}

void Estimator::estimate() {
    ::ceres::Solver::Summary summary;
    ::ceres::Solve(p_.solver, problem_.get(), &summary);
    DLOG(INFO) << summary.FullReport();
    if (!summary.IsSolutionUsable()) {
        LOG(ERROR) << "Solution not usable";
    }
}

std::unique_ptr<ground_surface::GroundSurface> Estimator::getGroundSurface() const {
    return std::make_unique<ground_surface::Plane>(plane_.get().template cast<float>(), Base::p_.range);
}

void Estimator::clearMeasurements_() {
    problem_ = std::make_unique<::ceres::Problem>(p_.problem);
    plane_.set(*problem_);
}

void Estimator::reset() {
    plane_.set(Base::p_.plane_initial.template cast<double>());
    clearMeasurements();
}
} // namespace pointcloud_surface::ground_surface_estimation::plane::ceres