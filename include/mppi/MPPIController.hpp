#pragma once

#include <random>
#include <vector>
#include <string>
#include <cstddef>

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

        virtual ~MPPIController() = default;

        virtual Vector command(const Vector& state);
        virtual void reset();

        const Matrix& getActionSequence() const;
        void shiftNominalTrajectory();
        void changeHorizon(int newHorizon);
        std::string getParams() const;

    protected:
        using RowMajorMatrix =
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using MatrixView = Eigen::Map<RowMajorMatrix>;
        using ConstMatrixView = Eigen::Map<const RowMajorMatrix>;

        Config cfg_;
        const Dynamics& dynamics_;
        const RunningCost& runningCost_;
        const TerminalCost* terminalCost_;

        Matrix U_;                      // T x nu
        std::vector<double> noiseData_;     // K contiguous T x nu samples
        std::vector<double> perturbedData_; // K contiguous T x nu samples
        Vector costs_;                  // K
        Vector weights_;                // K

        std::mt19937 rng_;
        std::normal_distribution<double> normal_;

        void initializeControl();
        void sampleNoise();
        Vector clampControl(const Vector& u) const;
        double rolloutCost(const Vector& initialState, ConstMatrixView actionSequence) const;
        void computeWeights();
        void updateNominalTrajectory();

        std::size_t sampleSize() const;
        std::size_t sampleOffset(int k) const;
        MatrixView noiseSample(int k);
        ConstMatrixView noiseSample(int k) const;
        MatrixView perturbedSample(int k);
        ConstMatrixView perturbedSample(int k) const;
    };

}  // namespace mppi
