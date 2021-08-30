#pragma once

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "ground_surface_estimation_base.hpp"
#include "plane_estimation_sac_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation::plane::sac {

class Estimator : public common::Estimator {

    using Base = common::Estimator;
    using Plane = Eigen::Hyperplane<float, 3>;

public:
    using Parameters = sac::Parameters;

public:
    Estimator(const Parameters&, const common::Parameters&);

    void estimate() final;

    std::unique_ptr<ground_surface::GroundSurface> getGroundSurface() const final;

    void reset() final;

private:
    void addMeasurements_(int) final;
    void clearMeasurements_() final;

    Parameters p_;
    pcl::SACSegmentation<pcl::PointXYZ> segmentation_;
    const typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{new pcl::PointCloud<pcl::PointXYZ>};
    Plane plane_;
};
} // namespace pointcloud_surface::ground_surface_estimation::plane::sac