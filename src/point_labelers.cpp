#include "point_labelers.hpp"

#include <glog/logging.h>

namespace pointcloud_surface {

std::string toString(const PointLabeler& in) {
    switch (in) {
    case PointLabeler::constant:
        return "constant";
    case PointLabeler::geman_mcclure:
        return "geman_mcclure";
    case PointLabeler::truncated_least_squares:
        return "truncated_least_squares";
    default:
        return "threshold";
    }
}

PointLabeler toPointLabeler(const std::string& in) {
    if (in == "constant") {
        return PointLabeler::constant;
    }
    if (in == "geman_mcclure") {
        return PointLabeler::geman_mcclure;
    }
    if (in == "truncated_least_squares") {
        return PointLabeler::truncated_least_squares;
    }
    LOG(FATAL) << "Unknown point classifier '" << in << "'. Using 'constant'.";
    return PointLabeler::constant;
}
} // namespace pointcloud_surface