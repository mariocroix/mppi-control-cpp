#pragma once

#include <random>
#include <vector>
#include <string>

#include "Config.hpp"
#include "Dynamics.hpp"
#include "Cost.hpp"
#include "Types.hpp"

namespace mppi {

    class MPPIController {
    public:
        MPPIController(
            const Config& config,
            const Dynamics& dynamics,
            const RunningCost& runningCost,
            const TerminalCost* terminalCost = nullptr
        );

        Vector command(const Vector& state);
        void reset();

        const Matrix& getActionSequence() const;
        void shiftNominalTrajectory();
        void changeHorizon(int newHorizon);
        std::string getParams() const;

    private:
        Config cfg_;
        const Dynamics& dynamics_;
        const RunningCost& runningCost_;
        const TerminalCost* terminalCost_;

        Matrix U_;                      // T x nu
        std::vector<Matrix> noise_;     // K elements, each T x nu
        std::vector<Matrix> perturbed_; // K elements, each T x nu
        Vector costs_;                  // K
        Vector weights_;                // K

        std::mt19937 rng_;
        std::normal_distribution<double> normal_;

        void initializeControl();
        void sampleNoise();
        Vector clampControl(const Vector& u) const;
        double rolloutCost(const Vector& initialState, const Matrix& actionSequence) const;
        void computeWeights();
        void updateNominalTrajectory();
    };

}  // namespace mppi