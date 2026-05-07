#include "mppi/MPPIController.hpp"
#include <cmath>
#include <omp.h>

namespace mppi {

namespace {
constexpr int kOpenMpRolloutThreshold = 100;
}

MPPIController::MPPIController(
    const Config& config,
    const Dynamics& dynamics,
    const RunningCost& runningCost,
    const TerminalCost* terminalCost
)
    : cfg_(config),
      dynamics_(dynamics),
      runningCost_(runningCost),
      terminalCost_(terminalCost),
      rng_(cfg_.seed),
      normal_(0.0, 1.0) {
    initializeControl();
}

void MPPIController::initializeControl() {
    U_ = Matrix::Zero(cfg_.T, cfg_.nu);

    noiseData_.assign(sampleSize() * cfg_.K, 0.0);
    perturbedData_.assign(sampleSize() * cfg_.K, 0.0);

    costs_ = Vector::Zero(cfg_.K);
    weights_ = Vector::Zero(cfg_.K);
}

std::size_t MPPIController::sampleSize() const {
    return static_cast<std::size_t>(cfg_.T) * static_cast<std::size_t>(cfg_.nu);
}

std::size_t MPPIController::sampleOffset(int k) const {
    return static_cast<std::size_t>(k) * sampleSize();
}

MPPIController::MatrixView MPPIController::noiseSample(int k) {
    return MatrixView(noiseData_.data() + sampleOffset(k), cfg_.T, cfg_.nu);
}

MPPIController::ConstMatrixView MPPIController::noiseSample(int k) const {
    return ConstMatrixView(noiseData_.data() + sampleOffset(k), cfg_.T, cfg_.nu);
}

MPPIController::MatrixView MPPIController::perturbedSample(int k) {
    return MatrixView(perturbedData_.data() + sampleOffset(k), cfg_.T, cfg_.nu);
}

MPPIController::ConstMatrixView MPPIController::perturbedSample(int k) const {
    return ConstMatrixView(perturbedData_.data() + sampleOffset(k), cfg_.T, cfg_.nu);
}

Vector MPPIController::clampControl(const Vector& u) const {
    Vector out = u;

    if (cfg_.uMin.size() == u.size()) {
        out = out.cwiseMax(cfg_.uMin);
    }
    if (cfg_.uMax.size() == u.size()) {
        out = out.cwiseMin(cfg_.uMax);
    }

    return out;
}

void MPPIController::sampleNoise() {
    for (int k = 0; k < cfg_.K; ++k) {
        auto noise = noiseSample(k);
        auto perturbed = perturbedSample(k);

        for (int t = 0; t < cfg_.T; ++t) {
            Vector eps(cfg_.nu);

            for (int i = 0; i < cfg_.nu; ++i) {
                double mean = cfg_.noiseMu(i);
                double stddev = std::sqrt(cfg_.noiseSigmaDiag(i));
                eps(i) = mean + stddev * normal_(rng_);
            }

            noise.row(t) = eps.transpose();

            Vector u = U_.row(t).transpose() + eps;
            u = clampControl(u);
            perturbed.row(t) = u.transpose();
        }
    }
}

double MPPIController::rolloutCost(
    const Vector& initialState,
    ConstMatrixView actionSequence
) const {
    Vector state = initialState;
    double totalCost = 0.0;

    for (int t = 0; t < cfg_.T; ++t) {
        Vector action = cfg_.uScale * actionSequence.row(t).transpose();
        state = dynamics_.propagate(state, action, t);
        totalCost += runningCost_.evaluate(state, action, t);
    }

    if (terminalCost_ != nullptr) {
        totalCost += terminalCost_->evaluate(state);
    }

    return totalCost;
}

void MPPIController::computeWeights() {
    double beta = costs_.minCoeff();

    double eta = 0.0;
    for (int k = 0; k < cfg_.K; ++k) {
        weights_(k) = std::exp(-(costs_(k) - beta) / cfg_.lambda);
        eta += weights_(k);
    }

    if (eta > 1e-12) {
        weights_ /= eta;
    }
}

void MPPIController::updateNominalTrajectory() {
    for (int t = 0; t < cfg_.T; ++t) {
        Vector delta = Vector::Zero(cfg_.nu);

        for (int k = 0; k < cfg_.K; ++k) {
            delta += weights_(k) * noiseSample(k).row(t).transpose();
        }

        Vector updated = U_.row(t).transpose() + delta;
        updated = clampControl(updated);
        U_.row(t) = updated.transpose();
    }
}

Vector MPPIController::command(const Vector& state) {
    shiftNominalTrajectory();
    sampleNoise();

    if (cfg_.K >= kOpenMpRolloutThreshold) {
        #pragma omp parallel for
        for (int k = 0; k < cfg_.K; ++k) {
            costs_(k) = rolloutCost(
                state,
                static_cast<const MPPIController*>(this)->perturbedSample(k)
            );
        }
    } else {
        for (int k = 0; k < cfg_.K; ++k) {
            costs_(k) = rolloutCost(
                state,
                static_cast<const MPPIController*>(this)->perturbedSample(k)
            );
        }
    }

    computeWeights();
    updateNominalTrajectory();

    return U_.row(0).transpose();
}

void MPPIController::shiftNominalTrajectory() {
    for (int t = 0; t < cfg_.T - 1; ++t) {
        U_.row(t) = U_.row(t + 1);
    }
    U_.row(cfg_.T - 1) = cfg_.uInit.transpose();
}

void MPPIController::changeHorizon(int newHorizon) {
    if (newHorizon == cfg_.T) return;

    Matrix newU = Matrix::Zero(newHorizon, cfg_.nu);

    int minT = std::min(newHorizon, cfg_.T);
    for (int t = 0; t < minT; ++t) {
        newU.row(t) = U_.row(t);
    }

    for (int t = minT; t < newHorizon; ++t) {
        newU.row(t) = cfg_.uInit.transpose();
    }

    cfg_.T = newHorizon;
    U_ = newU;

    noiseData_.assign(sampleSize() * cfg_.K, 0.0);
    perturbedData_.assign(sampleSize() * cfg_.K, 0.0);
    costs_ = Vector::Zero(cfg_.K);
    weights_ = Vector::Zero(cfg_.K);
}

void MPPIController::reset() {
    initializeControl();
}

const Matrix& MPPIController::getActionSequence() const {
    return U_;
}

std::string MPPIController::getParams() const {
    return "K=" + std::to_string(cfg_.K) +
           " T=" + std::to_string(cfg_.T) +
           " lambda=" + std::to_string(cfg_.lambda);
}

}  // namespace mppi
