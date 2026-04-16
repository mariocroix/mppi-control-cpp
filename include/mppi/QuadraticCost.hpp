#pragma once

#include "Cost.hpp"

namespace mppi {

    class QuadraticCost : public RunningCost {
    public:
        explicit QuadraticCost(const Vector& goal) : goal_(goal) {}

        double evaluate(
            const Vector& state,
            const Vector&,
            int
        ) const override {
            Vector dx = goal_ - state;
            return dx.squaredNorm();
        }

    private:
        Vector goal_;
    };

}  // namespace mppi