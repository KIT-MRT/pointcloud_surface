#pragma once

#include <Eigen/Core>

namespace pointcloud_surface::ground_surface::polynomial {

template <typename T>
using Representation = Eigen::Matrix<T, 8, 1>;
} // namespace pointcloud_surface::ground_surface::polynomial