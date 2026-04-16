#pragma once

#include <cmath>

#include "Cost.hpp"

namespace mppi {

    class PendulumTerminalCost : public TerminalCost {
    public:
        double evaluate(const Vector& state) const override {
            double theta = state(0);
            double thetaDot = state(1);

            double angle = angleNormalize(theta);

            // stronger penalty than running cost
            return 10.0 * angle * angle + 0.5 * thetaDot * thetaDot;
        }

    private:
        static double angleNormalize(double x) {
            while (x > M_PI) x -= 2.0 * M_PI;
            while (x < -M_PI) x += 2.0 * M_PI;
            return x;
        }
    };

} // namespace mppi