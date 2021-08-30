#pragma once

#include <pcl/point_cloud.h>

#include "ground_surface.hpp"

namespace pointcloud_surface::ground_surface {

template <typename Point_>
void makeUnordered(pcl::PointCloud<Point_>& in_out) {
    in_out.height = 1;
    in_out.width = in_out.points.size();
    in_out.is_dense = false;
}

/**
 * Remove all points where the comparison between that point's ground distance and a threshold is true.
 * If Cmp_=std::less<>, then all points with distance below threshold are removed.
 * If Cmp_=std::greater<>, then all points with distance above threshold are removed.
 * @tparam Point_ Point type
 * @tparam Cmp_ Comparison type
 * @param ground_surface Ground surface object
 * @param threshold The threshold used for comparison
 * @param in_out Point cloud where points are possibly removed
 */
template <typename Cmp_, typename Point_>
void filter(const GroundSurface::Ptr& ground_surface, const float threshold, pcl::PointCloud<Point_>& in_out) {
    using namespace std;
    Cmp_ cmp;
    in_out.points.erase(
        remove_if(begin(in_out.points),
                  end(in_out.points),
                  [&](const Point_& p) { return cmp(ground_surface->distance(p.getVector3fMap()), threshold); }),
        end(in_out.points));
    makeUnordered(in_out);
}
} // namespace pointcloud_surface::ground_surface
