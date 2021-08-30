#pragma once

#include <Eigen/Core>
#include <glog/logging.h>

#include "ground_surface.hpp"
#include "ground_surface_estimation_common_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation::common {

struct Estimator {

    using Measurements = Eigen::Matrix3Xf;
    using Weights = Eigen::VectorXf;

    Estimator(const Parameters& p);

    virtual ~Estimator() = default;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void addMeasurements(const Measurements&);

    void addMeasurements(const Measurements&, const Weights&);

    const Measurements& getMeasurements() const;

    const Weights& getWeights() const;

    void setWeights(const Weights&);

    void set(const Parameters&);

    const Parameters& getParameters() const;

    bool hasMeasurements() const;

    virtual void estimate() = 0;

    virtual void clearMeasurements();

    virtual void reset();

    virtual ground_surface::GroundSurface::Ptr getGroundSurface() const = 0;

protected:
    virtual void addMeasurements_(int num_measurements) = 0;
    virtual void clearMeasurements_() = 0;
    virtual void setWeights_();

    Parameters p_;
    Measurements measurements_;
    Weights weights_;
};
} // namespace pointcloud_surface::ground_surface_estimation::common