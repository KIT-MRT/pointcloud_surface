#pragma once

#include <memory>
#include <Eigen/Geometry>

namespace pointcloud_surface::ground_surface {

class GroundSurface {

public:
    using Ptr = std::unique_ptr<GroundSurface>;
    using ConstPtr = std::unique_ptr<const GroundSurface>;
    enum class Type { plane = 0, uniform_b_spline = 1, polynomial = 2, undefined };

public:
    explicit GroundSurface(const Type& type) : type_{type} {
    }

    virtual ~GroundSurface() = default;

    const Type& getType() const {
        return type_;
    }

    virtual Eigen::AlignedBox2f getRange() const = 0;

    float distance(const Eigen::Vector3f& p) const {
        return p.z() - height(p.head<2>());
    }

    virtual float height(const Eigen::Vector2f& p) const = 0;

    virtual Eigen::Vector3f normal(const Eigen::Vector2f& p) const = 0;

    friend std::ostream& operator<<(std::ostream& os, const GroundSurface& ground_surface) {
        return os << ground_surface.toString();
    }

private:
    virtual std::string toString() const = 0;

    Type type_{Type::undefined};
};
} // namespace pointcloud_surface::ground_surface
