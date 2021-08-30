#include "polynomial_estimation_ceres.hpp"

#include <ceres/autodiff_cost_function.h>
#include <ceres/loss_function.h>

#include "ground_surface_polynomial.hpp"

namespace pointcloud_surface::ground_surface_estimation::polynomial::ceres {

struct Residual {

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int dimension = 8;

    Residual(const Eigen::Array3Xf& m, const Eigen::ArrayXf& weights) : heights_{m.row(2)}, weights_{weights} {
        measurements_.resize(dimension, m.cols());
        measurements_.row(0).setOnes();
        measurements_.row(1) = m.row(0);                     ///< x
        measurements_.row(2) = m.row(1);                     ///< x
        measurements_.row(3) = m.row(0) * m.row(1);          ///< x y
        measurements_.row(4) = m.row(0).square();            ///< x^2
        measurements_.row(5) = m.row(1).square();            ///< y^2
        measurements_.row(6) = m.row(0).square() * m.row(1); ///< x^2 y
        measurements_.row(7) = m.row(0) * m.row(1).square(); ///< x y^2
        DLOG(INFO) << "Measurements:\n" << measurements_.leftCols(15);
        DLOG(INFO) << "Heights:\n" << heights_.head<15>().transpose();
        DLOG(INFO) << "Weights:\n" << weights_.head<15>().transpose();
        DLOG_ASSERT(measurements_.cols() == heights_.size());
        DLOG_ASSERT(heights_.size() == weights.size());
    }

    void updateWeights(const Eigen::ArrayXf& weights) {
        weights_ = weights;
    }

    template <typename T>
    bool operator()(const T* c, T* residual) const {
        using namespace Eigen;
        Map<Array<T, Dynamic, 1>>{residual, heights_.size()} =
            ((Map<const Matrix<T, 1, dimension>>{c} * measurements_.cast<T>()).transpose().array() -
             heights_.cast<T>()) *
            weights_.cast<T>();
        return true;
    }

    static ::ceres::CostFunction* create(const Eigen::Array3Xf& measurements, const Eigen::ArrayXf& weights) {
        return new ::ceres::AutoDiffCostFunction<Residual, ::ceres::DYNAMIC, dimension>(
            new Residual(measurements, weights), measurements.cols());
    }

private:
    Eigen::Matrix<float, dimension, Eigen::Dynamic> measurements_;
    Eigen::ArrayXf heights_;
    Eigen::ArrayXf weights_;
};

Estimator::Estimator(const ground_surface_estimation::ceres::Parameters& p_ceres, const common::Parameters& p_common)
        : Base{p_common}, p_ceres_{p_ceres} {
    reset();
    clearMeasurements_();
}

void Estimator::addMeasurements_(const int num_measurements) {
    problem_->AddResidualBlock(
        Residual::create(measurements_.rightCols(num_measurements), weights_.tail(num_measurements)),
        new ::ceres::TrivialLoss,
        poly_ceres_.data());
}

void Estimator::estimate() {
    ::ceres::Solver::Summary summary;
    ::ceres::Solve(p_ceres_.solver, problem_.get(), &summary);
    DLOG(INFO) << summary.FullReport();
    if (!summary.IsSolutionUsable()) {
        LOG(ERROR) << "Solution not usable";
    }
}

std::unique_ptr<ground_surface::GroundSurface> Estimator::getGroundSurface() const {
    return std::make_unique<ground_surface::Polynomial>(poly_ceres_.template cast<float>(), Base::p_.range);
}

void Estimator::clearMeasurements_() {
    poly_ceres_[0] = -p_.plane_initial.offset();
    problem_ = std::make_unique<::ceres::Problem>(p_ceres_.problem);
}
} // namespace pointcloud_surface::ground_surface_estimation::polynomial::ceres