#include "uniform_b_spline_estimation_ceres.hpp"

#include <ceres/loss_function.h>

namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::ceres {

struct Residual {

    using Representation = ground_surface::uniform_b_spline::Representation;
    using Evaluator = ubs::UniformBSplineCeresEvaluator<Representation>;

    Residual(const Evaluator& evaluator, const double& measurement) : evaluator_(evaluator), measurement_{measurement} {
    }

    template <typename T>
    bool operator()(const T* c0,
                    const T* c1,
                    const T* c2,
                    const T* c3,
                    const T* c4,
                    const T* c5,
                    const T* c6,
                    const T* c7,
                    const T* c8,
                    T* residual) const {
        evaluator_.evaluate(c0, c1, c2, c3, c4, c5, c6, c7, c8, residual);
        *residual -= static_cast<T>(measurement_);
        return true;
    }

    static ::ceres::CostFunction* create(const Evaluator& evaluator, const double& measurement) {
        return new ::ceres::AutoDiffCostFunction<Residual, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1>(
            new Residual(evaluator, measurement));
    }

private:
    Evaluator evaluator_;
    double measurement_;
};

Estimator::Estimator(const Parameters& p_uniform_b_spline,
                     const ground_surface_estimation::ceres::Parameters& p_ceres,
                     const common::Parameters& p_common)
        : Base{p_uniform_b_spline, p_common}, p_ceres_{p_ceres} {
    Base::reset();
}

void Estimator::addMeasurements_(const int num_measurements) {
    std::vector<double*> parameters(static_cast<size_t>(spline_ceres_.ControlPointsSupport));
    for (int c = measurements_.cols() - num_measurements; c < measurements_.cols(); c++) {
        const Eigen::Vector3d p{measurements_.col(c).cast<double>()};
        const Eigen::Vector2d& p2d{p.head<2>()};
        if (Base::spline_.inRange(p2d)) {
            const auto data{spline_ceres_.getPointData(p2d)};
            spline_ceres_.fillParameterPointers(data, std::begin(parameters), std::end(parameters));
            problem_->AddResidualBlock(
                Residual::create(spline_ceres_.getEvaluator(data), p.z()),
                new ::ceres::ScaledLoss(new ::ceres::TrivialLoss, weights_[c], ::ceres::Ownership::TAKE_OWNERSHIP),
                parameters);
        }
    }
}

void Estimator::estimate() {
    ::ceres::Solver::Summary summary;
    ::ceres::Solve(p_ceres_.solver, problem_.get(), &summary);
    DLOG(INFO) << summary.FullReport();
    if (!summary.IsSolutionUsable()) {
        LOG(ERROR) << "Solution not usable";
    }
}

void Estimator::clearMeasurements_() {
    problem_ = std::make_unique<::ceres::Problem>(p_ceres_.problem);
    spline_ceres_.template addSmoothnessResiduals<2>(*problem_, Base::p_.weight_smoothness);
}
} // namespace pointcloud_surface::ground_surface_estimation::uniform_b_spline::ceres