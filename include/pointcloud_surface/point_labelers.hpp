#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace pointcloud_surface {

enum class PointLabeler { constant = 0, geman_mcclure = 1, truncated_least_squares = 2 };

std::string toString(const PointLabeler&);

PointLabeler toPointLabeler(const std::string&);

namespace point_labelers {

template <int c>
struct Constant {
    constexpr float operator()(const float) const {
        return c;
    }
};

template <typename Cmp_ = std::less<>>
struct Threshold {
    Threshold(const float threshold) : threshold_{threshold} {
    }
    inline float operator()(const float d) const {
        return static_cast<float>(cmp_(d, threshold_));
    }

private:
    Cmp_ cmp_;
    float threshold_;
}; // namespace point_labelers

struct GemanMcClure {
    GemanMcClure(const float c, const float mu) : c_{c}, mu_{mu} {
        mu_c_sq_ = std::pow(mu_ * c_, 2);
    }
    inline float operator()(const float d) const {
        return std::pow(mu_c_sq_ / (std::pow(d, 2) + mu_c_sq_), 2);
    }

private:
    float c_;
    float mu_;
    float mu_c_sq_;
};

struct TruncatedLeastSquares {
    TruncatedLeastSquares(const float c, const float mu) : c_{c}, mu_{mu} {
        lb_ = mu_ / (mu_ + 1) * std::pow(c_, 2);
        ub_ = (mu_ + 1) / mu_ * std::pow(c_, 2);
        t_ = c_ * std::sqrt(mu_ * (mu_ + 1));
    }
    inline float operator()(const float d) const {
        const auto d_sq{std::pow(d, 2)};
        if (ub_ < d_sq) {
            return 0;
        }
        if (d_sq < lb_) {
            return 1;
        }
        return t_ / std::abs(d) - mu_;
    }

private:
    float c_;
    float mu_;
    float lb_;
    float ub_;
    float t_;
};

} // namespace point_labelers
} // namespace pointcloud_surface
