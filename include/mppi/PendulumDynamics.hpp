#pragma once

#include <algorithm>
#include <cmath>

#include "Dynamics.hpp"

namespace mppi {

    class PendulumDynamics : public Dynamics {
    public:
        Vector propagate(
            const Vector& state,
            const Vector& action,
            int
        ) const override {
            double theta = state(0);
            double thetaDot = state(1);
            double u = action(0);

            const double g = 10.0;
            const double m = 1.0;
            const double l = 1.0;
            const double dt = 0.05;

            u = std::clamp(u, -2.0, 2.0);

            double newThetaDot =
                thetaDot + (3.0 * g / (2.0 * l) * std::sin(theta)
                            + 3.0 / (m * l * l) * u) * dt;

            newThetaDot = std::clamp(newThetaDot, -8.0, 8.0);

            double newTheta = theta + newThetaDot * dt;

            Vector next(2);
            next << newTheta, newThetaDot;
            return next;
        }
    };

}  // namespace mppi