#pragma once

#include <memory>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>

#include "ground_surface.hpp"

namespace pointcloud_surface::ground_surface {

template <typename Point_>
void convert(const GroundSurface::Ptr& ground_surface,
             const Eigen::AlignedBox2f& range,
             const float sampling_distance,
             pcl::PointCloud<Point_>& cloud) {
    const Eigen::Vector2i points_per_dim{(range.sizes() / sampling_distance).cast<int>()};
    cloud.resize(points_per_dim.prod());
    for (int x = 0; x < points_per_dim.x(); x++) {
        for (int y = 0; y < points_per_dim.y(); y++) {
            auto& p{cloud.points[x * points_per_dim.y() + y]};
            p.x = x * sampling_distance + range.min().x();
            p.y = y * sampling_distance + range.min().y();
            p.z = ground_surface->height(p.getVector3fMap().template head<2>());
        }
    }
}
} // namespace pointcloud_surface::ground_surface
