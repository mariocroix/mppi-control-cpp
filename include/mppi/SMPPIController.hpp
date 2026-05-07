#pragma once

#include "MPPIController.hpp"

namespace mppi {

    class SMPPIController : public MPPIController {
    public:
        SMPPIController(
            const Config& config,
            const Dynamics& dynamics,
            const RunningCost& runningCost,
            const TerminalCost* terminalCost = nullptr,
            double wActionSeqCost = 1.0,
            double deltaT = 1.0
        );

        Vector command(const Vector& state) override;
        void reset() override;

        const Matrix& getActionSequence() const;
        void shiftNominalTrajectory();

    private:
        double wActionSeqCost_;
        double deltaT_;

        Matrix actionSequence_;  // actual commanded actions, T x nu
        std::vector<double> sampledActionsData_;

        MatrixView sampledActions(int k);
        ConstMatrixView sampledActions(int k) const;

        double smoothnessCost(ConstMatrixView actions) const;
    };

}  // namespace mppi
