#include "mppi/SMPPIController.hpp"
#include <omp.h>

namespace mppi {

namespace {
constexpr int kOpenMpRolloutThreshold = 100;
}

SMPPIController::SMPPIController(
    const Config& config,
    const Dynamics& dynamics,
    const RunningCost& runningCost,
    const TerminalCost* terminalCost,
    double wActionSeqCost,
    double deltaT
)
    : MPPIController(config, dynamics, runningCost, terminalCost),
      wActionSeqCost_(wActionSeqCost),
      deltaT_(deltaT) {
    actionSequence_ = Matrix::Zero(config.T, config.nu);
    U_ = Matrix::Zero(config.T, config.nu);  // lifted control space
    sampledActionsData_.assign(sampleSize() * cfg_.K, 0.0);
}

SMPPIController::MatrixView SMPPIController::sampledActions(int k) {
    return MatrixView(sampledActionsData_.data() + sampleOffset(k), cfg_.T, cfg_.nu);
}

SMPPIController::ConstMatrixView SMPPIController::sampledActions(int k) const {
    return ConstMatrixView(sampledActionsData_.data() + sampleOffset(k), cfg_.T, cfg_.nu);
}

double SMPPIController::smoothnessCost(ConstMatrixView actions) const {
    double total = 0.0;

    for (int t = 1; t < actions.rows(); ++t) {
        Vector diff = actions.row(t).transpose() - actions.row(t - 1).transpose();
        total += diff.squaredNorm();
    }

    return wActionSeqCost_ * total;
}

void SMPPIController::shiftNominalTrajectory() {
    for (int t = 0; t < cfg_.T - 1; ++t) {
        U_.row(t) = U_.row(t + 1);
        actionSequence_.row(t) = actionSequence_.row(t + 1);
    }

    U_.row(cfg_.T - 1) = cfg_.uInit.transpose();

    if (cfg_.T >= 2) {
        actionSequence_.row(cfg_.T - 1) = actionSequence_.row(cfg_.T - 2);
    } else {
        actionSequence_.row(cfg_.T - 1).setZero();
    }
}

Vector SMPPIController::command(const Vector& state) {
    shiftNominalTrajectory();

    const std::size_t requiredSize = sampleSize() * static_cast<std::size_t>(cfg_.K);
    if (sampledActionsData_.size() != requiredSize) {
        sampledActionsData_.assign(requiredSize, 0.0);
    }

    for (int k = 0; k < cfg_.K; ++k) {
        auto actions = sampledActions(k);
        auto noise = noiseSample(k);

        for (int t = 0; t < cfg_.T; ++t) {
            Vector eps(cfg_.nu);

            for (int i = 0; i < cfg_.nu; ++i) {
                double mean = cfg_.noiseMu(i);
                double stddev = std::sqrt(cfg_.noiseSigmaDiag(i));
                eps(i) = mean + stddev * normal_(rng_);
            }

            Vector control = U_.row(t).transpose() + eps;
            control = clampControl(control);

            Vector action = actionSequence_.row(t).transpose() + control * deltaT_;
            action = clampControl(action);

            actions.row(t) = action.transpose();

            noise.row(t) = (control - U_.row(t).transpose()).transpose();
        }
    }

    if (cfg_.K >= kOpenMpRolloutThreshold) {
        #pragma omp parallel for
        for (int k = 0; k < cfg_.K; ++k) {
            const auto actions =
                static_cast<const SMPPIController*>(this)->sampledActions(k);
            costs_(k) = rolloutCost(state, actions);
            costs_(k) += smoothnessCost(actions);
        }
    } else {
        for (int k = 0; k < cfg_.K; ++k) {
            const auto actions =
                static_cast<const SMPPIController*>(this)->sampledActions(k);
            costs_(k) = rolloutCost(state, actions);
            costs_(k) += smoothnessCost(actions);
        }
    }

    computeWeights();

    for (int t = 0; t < cfg_.T; ++t) {
        Vector delta = Vector::Zero(cfg_.nu);

        for (int k = 0; k < cfg_.K; ++k) {
            delta += weights_(k) * noiseSample(k).row(t).transpose();
        }

        Vector updatedControl = U_.row(t).transpose() + delta;
        updatedControl = clampControl(updatedControl);
        U_.row(t) = updatedControl.transpose();
    }

    for (int t = 0; t < cfg_.T; ++t) {
        Vector action = actionSequence_.row(t).transpose() + U_.row(t).transpose() * deltaT_;
        action = clampControl(action);
        actionSequence_.row(t) = action.transpose();
    }

    return actionSequence_.row(0).transpose();
}

void SMPPIController::reset() {
    MPPIController::reset();
    actionSequence_ = Matrix::Zero(cfg_.T, cfg_.nu);
    U_ = Matrix::Zero(cfg_.T, cfg_.nu);
    sampledActionsData_.assign(sampleSize() * cfg_.K, 0.0);
}

const Matrix& SMPPIController::getActionSequence() const {
    return actionSequence_;
}

}  // namespace mppi
