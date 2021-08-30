#pragma once

#include <Eigen/Core>
#include <uniform_bspline/uniform_bspline.hpp>
#include <util_eigen/alignment.hpp>

namespace pointcloud_surface::ground_surface::uniform_b_spline {

using GridPoint = Eigen::Vector2d;
using GridPoints = EigenAlignedVec<GridPoint>;

using GridValue = double;
using GridValues = Eigen::VectorXd;

using ControlPoints = Eigen::MatrixXd;

using Representation = ::ubs::UniformBSpline<double, 2, GridPoint, GridValue, ControlPoints>;

} // namespace pointcloud_surface::ground_surface::uniform_b_spline