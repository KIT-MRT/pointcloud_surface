#pragma once

#include <sstream>

#include "ground_surface.hpp"
#include "ground_surface_plane_representation.hpp"

namespace pointcloud_surface::ground_surface {

class Plane : public GroundSurface {

public:
    using Representation = plane::Representation;

public:
    explicit Plane(const Representation& representation, const Eigen::AlignedBox2f& range)
            : GroundSurface{GroundSurface::Type::plane}, representation_{representation}, range_{range} {
    }

    const Representation& getRepresentation() const {
        return representation_;
    }

    Eigen::AlignedBox2f getRange() const final {
        return range_;
    }

    float height(const Eigen::Vector2f& p) const final {
        return -(representation_.offset() + representation_.normal().head<2>().dot(p)) / representation_.normal().z();
    }

    Eigen::Vector3f normal(const Eigen::Vector2f& p) const final {
        return representation_.normal();
    }

private:
    std::string toString() const final {
        std::ostringstream oss;
        oss << "Plane: " << representation_.coeffs().transpose();
        return oss.str();
    }

    Representation representation_{Eigen::Vector3f::UnitZ(), 0};
    Eigen::AlignedBox2f range_;
};


} // namespace pointcloud_surface::ground_surface
