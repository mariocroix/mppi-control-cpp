#pragma once

#include "Dynamics.hpp"

namespace mppi {

    class LinearDynamics : public Dynamics {
    public:
        explicit LinearDynamics(const Matrix& B) : B_(B) {}

        Vector propagate(
            const Vector& state,
            const Vector& action,
            int
        ) const override {
            return state + B_ * action;
        }

    private:
        Matrix B_;
    };

}  // namespace mppi