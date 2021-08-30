#include "ground_surface_estimation_factory.hpp"

#include "ground_surface_estimation_constant.hpp"
#include "ground_surface_estimation_geman_mcclure.hpp"
#include "ground_surface_estimation_truncated_least_squares.hpp"

namespace pointcloud_surface::ground_surface_estimation {

std::unique_ptr<Estimator> create(const Parameters& p) {
    switch (p.point_labeler) {
    case PointLabeler::geman_mcclure:
        return std::make_unique<gmc::Estimator>(p);
    case PointLabeler::truncated_least_squares:
        return std::make_unique<tls::Estimator>(p);
    default:
        return std::make_unique<constant::Estimator>(p);
    }
}
} // namespace pointcloud_surface::ground_surface_estimation