#include "uniform_b_spline_estimation_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline {

SmoothnessOrder toSmoothnessOrder(const std::string& str) {
    if (str == "first") {
        return SmoothnessOrder::first;
    }
    if (str == "second") {
        return SmoothnessOrder::second;
    }
    throw std::runtime_error("Invalid string '" + str + "'");
}

std::string toString(const SmoothnessOrder& in) {
    switch (in) {
    case SmoothnessOrder::first:
        return "first";
    case SmoothnessOrder::second:
        return "second";
    default:
        return "first";
    }
}

std::string Parameters::getName() const {
    return "uniform_b_spline";
}

void Parameters::read_(const YAML::Node& n) {
    read(n, "distance_control_points_x", distance_control_points.x());
    read(n, "distance_control_points_y", distance_control_points.y());
    read(n, "weight_smoothness", weight_smoothness);

    std::string str;
    read(n, "smoothness_order", str);
    smoothness_order = toSmoothnessOrder(str);
}

void Parameters::write_(YAML::Emitter& e) const {
    write("distance_control_points_x", distance_control_points.x(), e);
    write("distance_control_points_y", distance_control_points.y(), e);
    write("weight_smoothness", weight_smoothness, e);
    write("smoothness_order", toString(smoothness_order), e);
}
} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline