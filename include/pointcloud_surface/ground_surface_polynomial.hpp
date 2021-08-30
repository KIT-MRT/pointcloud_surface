#pragma once

#include <sstream>

#include "ground_surface.hpp"
#include "ground_surface_polynomial_representation.hpp"

namespace pointcloud_surface::ground_surface {

class Polynomial : public GroundSurface {

public:
    using Representation = polynomial::Representation<float>;

public:
    explicit Polynomial(const Representation& representation, const Eigen::AlignedBox2f& range)
            : GroundSurface{GroundSurface::Type::polynomial}, representation_{representation}, range_{range} {
    }

    const Representation& getRepresentation() const {
        return representation_;
    }

    Eigen::AlignedBox2f getRange() const final {
        return range_;
    }

    float height(const Eigen::Vector2f& p) const final {
        return height<float>(representation_.data(), p);
    }

    Eigen::Vector3f normal(const Eigen::Vector2f& p) const final {
        return Eigen::Vector3f::Zero(); ///< \todo
    }

    template <typename T>
    static T height(const T* c, const Eigen::Matrix<T, 2, 1>& p) {
        using namespace Eigen;
        Matrix<T, 8, 1> t;
        t[0] = 1;
        t[1] = p.x();
        t[2] = p.y();
        t[3] = p.x() * p.y();
        t[4] = std::pow(p.x(), 2);
        t[5] = std::pow(p.y(), 2);
        t[6] = std::pow(p.x(), 2) * p.y();
        t[7] = p.x() * std::pow(p.y(), 2);
        return Map<const Matrix<T, 1, 8>>{c} * t;
    }

private:
    std::string toString() const final {
        std::ostringstream oss;
        oss << "Polynomial: " << representation_.transpose();
        return oss.str();
    }

    Representation representation_{Representation::Zero()};
    Eigen::AlignedBox2f range_;
};


} // namespace pointcloud_surface::ground_surface
