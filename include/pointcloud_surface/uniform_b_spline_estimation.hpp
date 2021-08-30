#pragma once

#include <uniform_bspline/uniform_bspline.hpp>

#include "ground_surface_estimation_base.hpp"
#include "ground_surface_uniform_b_spline.hpp"
#include "uniform_b_spline_estimation_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline {

class Estimator : public common::Estimator {

    using Base = common::Estimator;

public:
    using Representation = ground_surface::uniform_b_spline::Representation;
    using Parameters = uniform_b_spline::Parameters;

    Estimator(const Parameters&, const common::Parameters&);

    std::unique_ptr<ground_surface::GroundSurface> getGroundSurface() const final;

    void reset() final;

protected:
    void initializeSpline();

    Parameters p_;
    Representation spline_;

private:
    void setWeights_() final;
};
} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline