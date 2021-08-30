#pragma once

#include <pcl/point_cloud.h>
#include <tbb/parallel_for.h>

#include "ground_surface.hpp"
#include "point_labelers.hpp"

namespace pointcloud_surface::ground_surface {

template <typename FieldAccessor_, typename Labeler_, typename Point_>
void label(const GroundSurface::Ptr& ground_surface,
           const Labeler_& labeler,
           const float asymmetry_ratio,
           pcl::PointCloud<Point_>& in_out) {
    tbb::parallel_for<size_t>(0, in_out.size(), [&](const auto& c) {
        auto& p{in_out.points[c]};
        auto d{ground_surface->distance(p.getVector3fMap())};
        if (d > 0) {
            d *= asymmetry_ratio;
        }
        FieldAccessor_::get(p) = labeler(d);
    });
}

template <typename FieldAccessor_, typename Point_>
void label(const Eigen::VectorXf& labels, pcl::PointCloud<Point_>& in_out) {
    CHECK_EQ(labels.size(), in_out.size());
    tbb::parallel_for<size_t>(
        0, in_out.size(), [&](const auto& c) { FieldAccessor_::get(in_out.points[c]) = labels[c]; });
}

template <typename Labeler_, typename Point_>
Eigen::VectorXf label(const GroundSurface::Ptr& ground_surface,
                      const Labeler_& labeler,
                      const pcl::PointCloud<Point_>& in,
                      const float asymmetry_ratio) {
    Eigen::VectorXf out(in.size());
    tbb::parallel_for<size_t>(0, in.size(), [&](const auto& c) {
        auto d{ground_surface->distance(in.points[c].getVector3fMap())};
        if (d > 0) {
            d *= asymmetry_ratio;
        }
        out[c] = labeler(d);
    });
    return out;
}

template <typename Labeler_>
Eigen::VectorXf label(const GroundSurface::Ptr& ground_surface,
                      const Labeler_& labeler,
                      const float asymmetry_ratio,
                      const Eigen::Matrix3Xf& in) {
    Eigen::VectorXf out(in.cols());
    tbb::parallel_for<size_t>(0, in.cols(), [&](const auto& c) {
        auto d{ground_surface->distance(in.col(c))};
        if (d > 0) {
            d *= asymmetry_ratio;
        }
        out[c] = labeler(d);
    });
    return out;
}

template <typename FieldAccessor_, typename Point_>
void label(const GroundSurface::Ptr& ground_surface,
           const PointLabeler& point_labeler,
           const float p,
           const float mu,
           const float asymmetry_ratio,
           pcl::PointCloud<Point_>& in_out) {
    switch (point_labeler) {
    case PointLabeler::constant:
        label<FieldAccessor_>(ground_surface, point_labelers::Constant<1>(), asymmetry_ratio, in_out);
        break;
    case PointLabeler::geman_mcclure:
        label<FieldAccessor_>(ground_surface, point_labelers::GemanMcClure(p, mu), asymmetry_ratio, in_out);
        break;
    case PointLabeler::truncated_least_squares:
        label<FieldAccessor_>(ground_surface, point_labelers::TruncatedLeastSquares(p, mu), asymmetry_ratio, in_out);
        break;
    default:
        label<FieldAccessor_>(ground_surface, point_labelers::Constant<1>(), asymmetry_ratio, in_out);
        break;
    }
}

template <typename Point_>
Eigen::VectorXf label(const GroundSurface::Ptr& ground_surface,
                      const PointLabeler& point_labeler,
                      const float p,
                      const float mu,
                      const float asymmetry_ratio,
                      const pcl::PointCloud<Point_>& in) {
    switch (point_labeler) {
    case PointLabeler::constant:
        return label(ground_surface, point_labelers::Constant<1>(), asymmetry_ratio, in);
    case PointLabeler::geman_mcclure:
        return label(ground_surface, point_labelers::GemanMcClure(p, mu), asymmetry_ratio, in);
    case PointLabeler::truncated_least_squares:
        return label(ground_surface, point_labelers::TruncatedLeastSquares(p, mu), asymmetry_ratio, in);
    default:
        return label(ground_surface, point_labelers::Constant<1>(), asymmetry_ratio, in);
    }
}
} // namespace pointcloud_surface::ground_surface
