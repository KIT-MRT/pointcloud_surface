#include "ground_surface_estimation_parameters.hpp"

namespace pointcloud_surface::ground_surface_estimation {

Method toMethod(const std::string& str) {
    if (str == "plane_ceres") {
        return Method::plane_ceres;
    }
    if (str == "plane_sac") {
        return Method::plane_sac;
    }
    if (str == "uniform_b_spline_ceres") {
        return Method::uniform_b_spline_ceres;
    }
    if (str == "uniform_b_spline_eigen") {
        return Method::uniform_b_spline_eigen;
    }
    if (str == "polynomial_ceres") {
        return Method::polynomial_ceres;
    }
    if (str == "none") {
        return Method::none;
    }
    throw std::runtime_error("Invalid string '" + str + "'");
}

std::string toString(const Method& method) {
    switch (method) {
    case Method::plane_ceres:
        return "plane_ceres";
    case Method::plane_sac:
        return "plane_sac";
    case Method::uniform_b_spline_ceres:
        return "uniform_b_spline_ceres";
    case Method::uniform_b_spline_eigen:
        return "uniform_b_spline_eigen";
    case Method::polynomial_ceres:
        return "polynomial_ceres";
    case Method::none:
        return "none";
    default:
        return "none";
    }
}

std::string Parameters::getName() const {
    return "ground_surface_estimation";
}

void Parameters::read_(const YAML::Node& n) {
    method = static_cast<Method>(n["method"].as<int>());
    read(n, "num_iterations", num_iterations);
    point_labeler = static_cast<PointLabeler>(n["point_labeler"].as<int>());
    read(n, "distance_threshold", distance_threshold);
    read(n, "mu", mu);
    read(n, "asymmetry_ratio", asymmetry_ratio);

    common.read(n);
    ceres.read(n);
    plane_sac.read(n);
    uniform_b_spline.read(n);
}

void Parameters::write_(YAML::Emitter& e) const {
    write("method", static_cast<int>(method), e);
    write("num_iterations", num_iterations, e);
    write("point_labeler", static_cast<int>(point_labeler), e);
    write("distance_threshold", distance_threshold, e);
    write("mu", mu, e);
    write("asymmetry_ratio", asymmetry_ratio, e);

    common.write(e);
    ceres.write(e);
    plane_sac.write(e);
    uniform_b_spline.write(e);
}

} // namespace pointcloud_surface::ground_surface_estimation
