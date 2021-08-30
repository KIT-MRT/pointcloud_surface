#include "plane_estimation_sac.hpp"

#include <pcl/common/angles.h>

#include "ground_surface_plane.hpp"

namespace pointcloud_surface::ground_surface_estimation::plane::sac {

Estimator::Estimator(const Parameters& p, const common::Parameters& p_common) : Base{p_common}, p_{p} {
    segmentation_.setOptimizeCoefficients(p.optimize_coefficients);
    segmentation_.setModelType(pcl::SACMODEL_PLANE);
    segmentation_.setAxis(p_common.plane_initial.normal());
    segmentation_.setDistanceThreshold(p.inlier_threshold);
    segmentation_.setEpsAngle(pcl::deg2rad(p.eps_angle));
    segmentation_.setMaxIterations(p.max_iterations);
    segmentation_.setProbability(p.probability);
    segmentation_.setInputCloud(this->cloud_);
    reset();
}

void Estimator::addMeasurements_(const int num_measurements) {
    cloud_->resize(measurements_.cols());
    cloud_->getMatrixXfMap().topRows<3>().rightCols(num_measurements) = measurements_.rightCols(num_measurements);
}

void Estimator::estimate() {
    pcl::PointIndices indices;
    pcl::ModelCoefficients coefficients;
    segmentation_.segment(indices, coefficients);
    const auto& p{coefficients.values};
    plane_ = Plane{Eigen::Vector3f{p[0], p[1], p[2]}, p[3]};
}

std::unique_ptr<ground_surface::GroundSurface> Estimator::getGroundSurface() const {
    return std::make_unique<ground_surface::Plane>(plane_, Base::p_.range);
}

void Estimator::clearMeasurements_() {
    cloud_->clear();
}

void Estimator::reset() {
    clearMeasurements();
    plane_ = Base::p_.plane_initial;
}
} // namespace pointcloud_surface::ground_surface_estimation::plane::sac