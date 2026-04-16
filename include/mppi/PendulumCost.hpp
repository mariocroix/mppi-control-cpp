#pragma once

#include <cmath>

#include "Cost.hpp"

namespace mppi {

    class PendulumCost : public RunningCost {
    public:
        double evaluate(
            const Vector& state,
            const Vector& action,
            int
        ) const override {
            double theta = state(0);
            double thetaDot = state(1);
            double u = action(0);

            double angle = angleNormalize(theta);

            return angle * angle + 0.1 * thetaDot * thetaDot + 0.001 * u * u;
        }

    private:
        static double angleNormalize(double x) {
            while (x > M_PI) {
                x -= 2.0 * M_PI;
            }
            while (x < -M_PI) {
                x += 2.0 * M_PI;
            }
            return x;
        }
    };

}  // namespace mppi