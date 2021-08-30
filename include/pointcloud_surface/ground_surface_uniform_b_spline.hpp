#pragma once

#include <limits>
#include <sstream>

#include "ground_surface.hpp"
#include "ground_surface_uniform_b_spline_representation.hpp"

namespace pointcloud_surface::ground_surface {

struct UniformBSpline : public GroundSurface {

    using Representation = uniform_b_spline::Representation;

    explicit UniformBSpline(const Representation&);

    const Representation& getRepresentation() const;

    Eigen::AlignedBox2f getRange() const final;

    float height(const Eigen::Vector2f& p) const final;

    Eigen::Vector3f normal(const Eigen::Vector2f& p) const final;

private:
    std::string toString() const final;

    Representation representation_;
};


} // namespace pointcloud_surface::ground_surface
