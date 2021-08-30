#pragma once

#include <memory>

#include "ground_surface_estimation.hpp"

namespace pointcloud_surface::ground_surface_estimation {

std::unique_ptr<Estimator> create(const ground_surface_estimation::Parameters&);
} // namespace pointcloud_surface::ground_surface_estimation