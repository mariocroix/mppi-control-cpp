#pragma once

#include "mppi/Config.hpp"
#include "mppi/Types.hpp"

namespace mppi {

    class CUDAPendulumMPPI {
    public:
        explicit CUDAPendulumMPPI(const Config& config);
        ~CUDAPendulumMPPI();

        Vector command(const Vector& state);
        const Matrix& getActionSequence() const;
        void reset();

    private:
        Config cfg_;
        Matrix UHost_;

        double* d_U_ = nullptr;
        double* d_noise_ = nullptr;
        double* d_costs_ = nullptr;
        double* d_weights_ = nullptr;
        void* d_rngStates_ = nullptr;

        void allocateDevice();
        void freeDevice();
        void copyUToDevice();
        void copyUToHost();
    };

}  // namespace mppi