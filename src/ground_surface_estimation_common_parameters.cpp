#include "ground_surface_estimation_common_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation::common {

std::string Parameters::getName() const {
    return "common";
}

void Parameters::read_(const YAML::Node& n) {
    read(n, "plane_normal_x", plane_initial.normal().x());
    read(n, "plane_normal_y", plane_initial.normal().y());
    read(n, "plane_normal_z", plane_initial.normal().z());
    plane_initial.normal().normalize();
    read(n, "plane_offset", plane_initial.offset());

    read(n, "range_min_x", range.min().x());
    read(n, "range_min_y", range.min().y());
    read(n, "range_max_x", range.max().x());
    read(n, "range_max_y", range.max().y());
}

void Parameters::write_(YAML::Emitter& e) const {
    write("plane_normal_x", plane_initial.normal().x(), e);
    write("plane_normal_y", plane_initial.normal().y(), e);
    write("plane_normal_z", plane_initial.normal().z(), e);
    write("plane_offset", plane_initial.offset(), e);

    write("range_min_x", range.min().x(), e);
    write("range_min_y", range.min().y(), e);
    write("range_max_x", range.max().x(), e);
    write("range_max_y", range.max().y(), e);
}
} // namespace pointcloud_surface::ground_surface_estimation::common