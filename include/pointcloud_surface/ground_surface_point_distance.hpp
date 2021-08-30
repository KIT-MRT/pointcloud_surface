#pragma once

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <tbb/parallel_for.h>

#include "ground_surface.hpp"

namespace pointcloud_surface::ground_surface {

template <typename FieldAccessor_, typename Point_>
void distance(const GroundSurface::Ptr& ground_surface, pcl::PointCloud<Point_>& in_out) {
    tbb::parallel_for<size_t>(0, in_out.size(), [&](const size_t& c) {
        FieldAccessor_::get(in_out.points[c]) = ground_surface->distance(in_out.points[c].getVector3fMap());
    });
}

template <typename Point_>
Eigen::ArrayXf distance(const GroundSurface::Ptr& ground_surface, const pcl::PointCloud<Point_>& in) {
    Eigen::ArrayXf d(in.size());
    tbb::parallel_for<size_t>(
        0, in.size(), [&](const size_t& c) { d[c] = ground_surface->distance(in.points[c].getVector3fMap()); });
    return d;
}

inline Eigen::ArrayXf distance(const GroundSurface::Ptr& ground_surface, const Eigen::Matrix3Xf& in) {
    Eigen::ArrayXf d(in.cols());
    tbb::parallel_for<size_t>(0, in.cols(), [&](const auto& c) { d[c] = ground_surface->distance(in.col(c)); });
    return d;
}

template <typename FieldAccessor_, typename PointOut_, typename Point_ = PointOut_>
void distanceFromReference(const GroundSurface::Ptr& ground_surface,
                           const pcl::PointCloud<Point_>& reference,
                           pcl::PointCloud<PointOut_>& in_out) {
    CHECK_EQ(in_out.size(), reference.size());
    tbb::parallel_for<size_t>(0, reference.size(), [&](const size_t& c) {
        FieldAccessor_::get(in_out.points[c]) = ground_surface->distance(reference.points[c].getVector3fMap());
    });
}
} // namespace pointcloud_surface::ground_surface
