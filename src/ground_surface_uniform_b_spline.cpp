#include "ground_surface_uniform_b_spline.hpp"

namespace pointcloud_surface::ground_surface {

UniformBSpline::UniformBSpline(const Representation& representation)
        : GroundSurface{GroundSurface::Type::uniform_b_spline}, representation_{representation} {
}

const UniformBSpline::Representation& UniformBSpline::getRepresentation() const {
    return representation_;
}

Eigen::AlignedBox2f UniformBSpline::getRange() const {
    return {representation_.getLowerBound().cast<float>(), representation_.getUpperBound().cast<float>()};
}

float UniformBSpline::height(const Eigen::Vector2f& p) const {
    const Eigen::Vector2d pd{p.cast<double>()};
    if (representation_.inRange(pd)) {
        return representation_.evaluate(pd);
    }
    return std::numeric_limits<float>::quiet_NaN();
}

Eigen::Vector3f UniformBSpline::normal(const Eigen::Vector2f& p) const {
    return Eigen::Vector3f::UnitZ();
}

std::string UniformBSpline::toString() const {
    std::ostringstream oss;
    oss << "Spline control points: " << std::endl << representation_.getControlPoints();
    return oss.str();
}
} // namespace pointcloud_surface::ground_surface
