#pragma once

#include <memory>
#include <Eigen/Core>

#include "ground_surface_estimation_base.hpp"
#include "ground_surface_estimation_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation {

struct Estimator {

    using Measurements = Eigen::Matrix3Xf;
    using Weights = Eigen::VectorXf;

    Estimator(const Parameters&);

    virtual ~Estimator() = default;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void estimate() = 0;

    virtual void addMeasurements(const Measurements&) = 0;

    ground_surface::GroundSurface::Ptr getGroundSurface() const;

    virtual bool hasMeasurements() const;

    virtual void clearMeasurements();

    const Measurements& getMeasurements() const;

    const Weights& getWeights() const;

    const float& getMu() const;

    void reset();

protected:
    Parameters p_;
    std::unique_ptr<common::Estimator> estimator_;
    float mu_{1};
};
} // namespace pointcloud_surface::ground_surface_estimation